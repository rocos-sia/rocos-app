// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "move_line_offline.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

#include "logger.hpp"

namespace rocos {

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveLineOffline::MoveLineOffline(const Frame& pose_start,
                                 const Frame& pose_goal,
                                 ModelInterface* model,
                                 double v_limit,
                                 double a_limit,
                                 double j_limit,
                                 double dt)
    : MotionInterface(model)
    , dt_(dt)
    , profile_(dt)
    , model_(model)
    , pose_start_(pose_start)
    , pose_goal_(pose_goal)
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {
    // 缓存四元数，避免每周期 GetQuaternion
    double qs[4], qe[4];
    pose_start_.M.GetQuaternion(qs[0], qs[1], qs[2], qs[3]);
    pose_goal_.M.GetQuaternion(qe[0], qe[1], qe[2], qe[3]);
    q_start_ = Eigen::Quaterniond(qs[3], qs[0], qs[1], qs[2]);  // w,x,y,z
    q_end_   = Eigen::Quaterniond(qe[3], qe[0], qe[1], qe[2]);

    // 半球面对齐：dot<0 时翻转 q_end_，保证 slerp 走最短弧
    if (q_start_.dot(q_end_) < 0.0) {
        q_end_.coeffs() *= -1.0;
    }

    // ─── 从四元数差提取角-轴表示（pause 时算角速度 ω = axis * angle * s_dot）───
    Eigen::Quaterniond q_diff = q_end_ * q_start_.conjugate();
    // 归一化防止数值误差导致 |w| > 1
    double w = std::clamp(q_diff.w(), -1.0, 1.0);
    total_angle_ = 2.0 * std::acos(w);

    double sin_half = std::sqrt(1.0 - w * w);
    if (sin_half > 1e-12) {
        rot_axis_ = Eigen::Vector3d(q_diff.x(), q_diff.y(), q_diff.z()) / sin_half;
    } else {
        // 旋转角度 ≈ 0，方向无所谓
        rot_axis_ = Eigen::Vector3d::UnitZ();
    }
}

MoveLineOffline::~MoveLineOffline() = default;

// ============================================================================
// 初始关节位置
// ============================================================================

void MoveLineOffline::SetInitialJointPosition(const JntArray& q) {
    q_init_ = q;
}

// ============================================================================
// 参数校验
// ============================================================================

Result MoveLineOffline::ValidateParameters() const {
    // 1. model 必须提供
    if (model_ == nullptr) {
        return Result::IllegalParameter;
    }

    // 2. dt 合法性
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }

    // 3. 一维标量限制：有限性 + 正值性
    if (!std::isfinite(v_limit_) || !std::isfinite(a_limit_) || !std::isfinite(j_limit_)) {
        return Result::ParameterNanOrInf;
    }
    if (v_limit_ <= 0.0 || a_limit_ <= 0.0 || j_limit_ <= 0.0) {
        return Result::IllegalParameter;
    }

    // 4. 起止位姿：位置和旋转分量有限性检查
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(pose_start_.p(i)) || !std::isfinite(pose_goal_.p(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(pose_start_.M(r, c)) ||
                !std::isfinite(pose_goal_.M(r, c))) {
                return Result::ParameterNanOrInf;
            }
        }
    }

    // 5. 初始关节位置必须已设置
    if (q_init_.rows() == 0) {
        return Result::IllegalParameter;
    }

    return Result::NoError;
}

// ============================================================================
// 归一化限制计算（复用 MoveLine 逻辑）
// ============================================================================

bool MoveLineOffline::computeNormalizedLimits() {
    const double translation = (pose_goal_.p - pose_start_.p).Norm();

    const double cos_half = std::abs(q_start_.dot(q_end_));
    const double rotation = 2.0 * std::acos(std::clamp(cos_half, 0.0, 1.0));

    const double r_length = kEquivalentRadius * rotation;
    path_length_ = std::max(translation, r_length);

    if (path_length_ < kPathEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0;
        norm_a_ = 1.0;
        norm_j_ = 1.0;
        return true;
    }

    has_motion_ = true;
    norm_v_ = v_limit_ / path_length_;
    norm_a_ = a_limit_ / path_length_;
    norm_j_ = j_limit_ / path_length_;

    return true;
}

// ============================================================================
// 位姿插值：平移线性 + 旋转 Slerp (Eigen)
// ============================================================================

Frame MoveLineOffline::interpolatePose(double s) const {
    Frame result;

    // 平移：线性
    result.p = pose_start_.p + (pose_goal_.p - pose_start_.p) * s;

    // 旋转：Eigen::Quaterniond::slerp
    Eigen::Quaterniond q_interp = q_start_.slerp(s, q_end_);
    result.M = KDL::Rotation::Quaternion(
        q_interp.x(), q_interp.y(), q_interp.z(), q_interp.w());

    return result;
}

// ============================================================================
// s_dot → 笛卡尔 Twist
//
// 直线路径上，平移方向恒定、旋转轴恒定，因此速度计算退化：
//   v     = (p_goal - p_start) * s_dot             [m/s]
//   omega = rot_axis_ * total_angle_ * s_dot       [rad/s]
// ============================================================================

Twist MoveLineOffline::computeCartesianTwist(double s_dot) const {
    // 平移速度
    KDL::Vector v = (pose_goal_.p - pose_start_.p) * s_dot;

    // 旋转角速度
    KDL::Vector omega(rot_axis_.x() * total_angle_ * s_dot,
                       rot_axis_.y() * total_angle_ * s_dot,
                       rot_axis_.z() * total_angle_ * s_dot);

    return Twist(v, omega);
}

// ============================================================================
// J⁺ · twist → q_dot（Eigen SVD 伪逆）
//
// 因为全轨迹 IK 已通过，Jacobian 沿路径非奇异，
// 最小奇异值远大于 0，SVD 伪逆数值稳定。
// ============================================================================

bool MoveLineOffline::solveJointVelocity(const JntArray& q,
                                          const Twist& twist,
                                          JntArray& q_dot_out) const {
    // 1. 计算 Jacobian
    Jacobian J;
    if (model_->GetJacobian(q, J) != Result::NoError) {
        return false;
    }

    // 2. Twist → Eigen 6-vector
    Eigen::Matrix<double, 6, 1> twist_vec;
    twist_vec << twist.vel.data[0], twist.vel.data[1], twist.vel.data[2],
                 twist.rot.data[0], twist.rot.data[1], twist.rot.data[2];

    // 3. SVD 伪逆求解 J · q_dot = twist
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd q_dot = svd.solve(twist_vec);

    // 4. 写回 JntArray
    if (q_dot_out.rows() != static_cast<unsigned int>(q_dot.size())) {
        q_dot_out.resize(q_dot.size());
    }
    q_dot_out.data = q_dot;

    return true;
}

// ============================================================================
// Reset：离线规划 + 逐点 IK 验证
//
// 流程：
//   1. 参数校验
//   2. 归一化限制
//   3. 整条 S 曲线跑完 → 收集所有 (s, s_dot)
//   4. 逐点笛卡尔插值 → IK 求解（前一个解 warm-start）
//   5. 任一 IK 失败 → PlanError，清空轨迹
// ============================================================================

Result MoveLineOffline::Reset() {
    const auto log_ptr = Logger::getInstance("MoveLineOffline");

    // 1. 参数校验
    const Result validation = ValidateParameters();
    if (validation != Result::NoError) {
        Logger::logCode(LogLevel::Error, "MoveLineOffline", static_cast<int>(validation),
            "MoveLineOffline parameter validation failed: result={}, dt={} s, velocity={} m/s, acceleration={} m/s^2, jerk={} m/s^3, model_ptr={}.",
            static_cast<int>(validation), dt_, v_limit_, a_limit_, j_limit_,
            static_cast<const void*>(model_));
        return validation;
    }

    // 2. 归一化限制
    if (!computeNormalizedLimits()) {
        log_ptr->error("MoveLineOffline normalized limit computation failed: model_ptr={}.",
                       static_cast<const void*>(model_));
        return Result::PlanError;
    }
    if (!has_motion_) {
        return Result::PlanFinished;
    }

    n_joints_ = model_->GetJointNum();
    if (n_joints_ <= 0) {
        log_ptr->error("MoveLineOffline planning failed: model returned invalid joint count={}.", n_joints_);
        return Result::PlanError;
    }

    // 3. 离线跑完整条 S 曲线，收集 (s, s_dot)
    trajectory_.clear();
    profile_.Reset(0.0, 0.0, 0.0);
    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) {
        log_ptr->error(
            "MoveLineOffline velocity profile start failed: normalized velocity={} 1/s, acceleration={} 1/s^2, jerk={} 1/s^3, dt={} s.",
            norm_v_, norm_a_, norm_j_, dt_);
        return Result::PlanError;
    }

    while (true) {
        if ((max_planning_steps_ && trajectory_.size() >= max_planning_steps_) ||
            std::chrono::steady_clock::now() >= planning_deadline_) {
            trajectory_.clear();
            return Result::MotionLimit;
        }
        const int rc = profile_.Update();
        if (rc < 0) {
            trajectory_.clear();
            log_ptr->error("MoveLineOffline velocity profile update failed: profile_result={}, trajectory_size={}.",
                           rc, trajectory_.size());
            return Result::PlanError;
        }

        TrajectoryPoint pt;
        pt.s     = profile_.position();
        pt.s_dot = profile_.velocity();
        trajectory_.push_back(std::move(pt));

        if (rc == 0) break;  // Profile Finished
    }

    // 确保终点 s=1.0 且 s_dot=0
    if (!trajectory_.empty()) {
        auto& last = trajectory_.back();
        last.s     = 1.0;
        last.s_dot = 0.0;
    }

    // 4. 逐点 IK 验证（前一个解 warm-start）
    JntArray q_prev = q_init_;
    for (auto& pt : trajectory_) {
        if (std::chrono::steady_clock::now() >= planning_deadline_) {
            trajectory_.clear();
            return Result::MotionLimit;
        }
        Frame frame = interpolatePose(pt.s);
        JntArray q_out(n_joints_);

        Result r = model_->InverseKinematics(q_prev, frame, q_out);
        if (r != Result::NoError) {
            Logger::logCode(LogLevel::Error, "MoveLineOffline", static_cast<int>(r),
                "MoveLineOffline trajectory inverse kinematics failed: result={}, normalized path position={}, trajectory_size={}, "
                "q_init_rows={}, model_ptr={}",
                static_cast<int>(r), pt.s, trajectory_.size(),
                q_init_.rows(), static_cast<const void*>(model_));
            trajectory_.clear();
            return Result::PlanError;
        }

        pt.q = std::move(q_out);
        q_prev = pt.q;
    }

    // 5. 初始化执行状态
    index_     = 0;
    mode_      = Mode::Normal;
    q_current_ = trajectory_[0].q;

    return Result::NoError;
}

// ============================================================================
// 生成当前周期参考值（内含单步推进）
//
// Normal:     索引 +1，输出预存关节位置
// Integrate:  profile_.Update() → Jacobian 积分 → 输出关节位置
// ============================================================================

Result MoveLineOffline::GenerateRef(Reference& ref_out) {
    if (!has_motion_ || trajectory_.empty()) {
        return Result::PlanError;
    }

    if (mode_ == Mode::Normal) {
        index_++;
        if (index_ >= trajectory_.size()) {
            ref_out = trajectory_.back().q;
            return Result::PlanFinished;
        }
        ref_out = trajectory_[index_].q;
        return Result::NoError;
    }

    // ─── Mode::Integrate：Jacobian 积分模式 ───
    const int rc = profile_.Update();
    if (rc < 0) {
        return Result::PlanError;
    }

    const double s_dot = profile_.velocity();
    Twist twist = computeCartesianTwist(s_dot);

    JntArray q_dot(n_joints_);
    if (!solveJointVelocity(q_current_, twist, q_dot)) {
        return Result::PlanError;
    }

    q_current_.data += q_dot.data * dt_;
    ref_out = q_current_;

    return (rc == 0) ? Result::PlanFinished : Result::NoError;
}

// ============================================================================
// 切换到积分模式（Normal → Integrate 的内部过渡）
// ============================================================================

Result MoveLineOffline::switchToIntegrateMode() {
    if (mode_ == Mode::Integrate) {
        return Result::NoError;  // 已在积分模式
    }
    if (trajectory_.empty()) {
        return Result::PlanError;
    }

    // 越界 clamp 到终点：在已完成的轨迹上 Pause/Stop
    // → 从 s=1.0, s_dot=0 开始减速，等价于无操作，行为有定义
    if (index_ >= trajectory_.size()) {
        index_ = trajectory_.size() - 1;
    }

    const auto& pt = trajectory_[index_];

    // 从当前轨迹点重新初始化 profile，使后续 Pause/Stop 从此状态开始减速
    profile_.Reset(pt.s, pt.s_dot);
    q_current_ = pt.q;
    mode_      = Mode::Integrate;

    return Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
//
// 关键设计：
//   - Normal 模式下首次 Pause/Stop → 先切到 Integrate 模式再减速
//   - Integrate 模式下直接对 profile 减速/恢复
//   - Resume 后继续 Jacobian 积分到终点（不回到 Normal 回放模式）
// ============================================================================

Result MoveLineOffline::Pause() {
    if (!has_motion_) {
        return Result::NoError;
    }

    // Normal → Integrate（保留当前轨迹状态）
    Result r = switchToIntegrateMode();
    if (r != Result::NoError) {
        return r;
    }

    if (!profile_.Pause(norm_a_, norm_j_)) {
        return Result::PlanError;
    }
    return Result::NoError;
}

Result MoveLineOffline::Resume() {
    if (!has_motion_) {
        return Result::NoError;
    }
    // Resume 后继续 Jacobian 积分到终点，不切回 Normal
    profile_.Resume();
    return Result::NoError;
}

Result MoveLineOffline::Stop() {
    if (!has_motion_) {
        return Result::NoError;
    }

    // Normal → Integrate（保留当前轨迹状态）
    Result r = switchToIntegrateMode();
    if (r != Result::NoError) {
        return r;
    }

    if (!profile_.Stop(norm_a_, norm_j_)) {
        return Result::PlanError;
    }
    return Result::NoError;
}

}  // namespace rocos
