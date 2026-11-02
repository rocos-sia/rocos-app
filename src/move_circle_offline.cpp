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

#include "move_circle_offline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

// ============================================================================
// 圆弧几何（从 MoveCircle 内联移植）
// ============================================================================

bool MoveCircleOffline::buildArcFrame(const Frame& start,
                                       const Frame& center,
                                       Frame& arc_out,
                                       double& radius) {
    const rocos::Vector c = center.p;

    rocos::Vector axis_x = start.p - c;
    radius = axis_x.Normalize();
    if (radius < kArcEpsilon) return false;

    rocos::Vector axis_z = center.M.UnitZ();
    const double proj = rocos::dot(axis_z, axis_x);
    axis_z -= proj * axis_x;
    if (axis_z.Normalize() < kArcEpsilon) {
        axis_z = rocos::Vector(0, 0, 1);
        if (std::abs(rocos::dot(axis_z, axis_x)) > 0.999)
            axis_z = rocos::Vector(0, 1, 0);
        axis_z -= rocos::dot(axis_z, axis_x) * axis_x;
        axis_z.Normalize();
    }

    rocos::Vector axis_y = axis_z * axis_x;
    axis_y.Normalize();

    arc_out = rocos::Frame(rocos::Rotation(axis_x, axis_y, axis_z), c);
    return true;
}

bool MoveCircleOffline::solveThreePoints(const Frame& start,
                                          const Frame& via,
                                          const Frame& goal,
                                          Frame& arc_out,
                                          double& radius,
                                          double& theta) {
    rocos::Vector v1 = via.p - start.p;
    rocos::Vector v2 = goal.p - start.p;
    if (v1.Normalize() < kArcEpsilon) return false;
    if (v2.Normalize() < kArcEpsilon) return false;

    rocos::Vector axis_z = v1 * v2;
    if (axis_z.Normalize() < kArcEpsilon) return false;

    rocos::Vector axis_x = v1;
    rocos::Vector axis_y = axis_z * axis_x;
    axis_y.Normalize();

    const rocos::Vector d1 = via.p - start.p;
    const rocos::Vector d2 = goal.p - start.p;
    const double bx = rocos::dot(d1, axis_x);
    const double cx = rocos::dot(d2, axis_x);
    const double cy = rocos::dot(d2, axis_y);
    if (std::abs(cy) < kArcEpsilon) return false;

    const double h = ((cx - bx / 2.0) * (cx - bx / 2.0) + cy * cy -
                      (bx / 2.0) * (bx / 2.0)) / (2.0 * cy);
    const rocos::Vector center = start.p + axis_x * (bx / 2.0) + axis_y * h;

    rocos::Vector arc_x = start.p - center;
    radius = arc_x.Normalize();
    if (radius < kArcEpsilon) return false;

    rocos::Vector arc_z = axis_z;
    arc_z -= rocos::dot(arc_z, arc_x) * arc_x;
    if (arc_z.Normalize() < kArcEpsilon) return false;

    rocos::Vector arc_y = arc_z * arc_x;
    arc_y.Normalize();

    arc_out = rocos::Frame(rocos::Rotation(arc_x, arc_y, arc_z), center);

    const rocos::Vector end_local = arc_out.Inverse() * goal.p;
    double t = std::atan2(end_local(1), end_local(0));
    if (t < 0.0) t += 2.0 * M_PI;
    theta = t;
    return true;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveCircleOffline::MoveCircleOffline(const Frame& pose_start,
                                     const Frame& center_frame,
                                     double theta,
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
    , theta_(theta)
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {
    if (!buildArcFrame(pose_start_, center_frame, arc_frame_, radius_)) {
        theta_  = 0.0;
        radius_ = 0.0;
        arc_frame_ = pose_start_;
    }
}

MoveCircleOffline::MoveCircleOffline(const Frame& pose_start,
                                     const Frame& pose_via,
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
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {
    if (!solveThreePoints(pose_start_, pose_via, pose_goal,
                          arc_frame_, radius_, theta_)) {
        theta_  = 0.0;
        radius_ = 0.0;
        arc_frame_ = pose_start_;
    }
}

MoveCircleOffline::~MoveCircleOffline() = default;

// ============================================================================
// 初始关节位置
// ============================================================================

void MoveCircleOffline::SetInitialJointPosition(const JntArray& q) {
    q_init_ = q;
}

// ============================================================================
// 参数校验
// ============================================================================

Result MoveCircleOffline::ValidateParameters() const {
    if (model_ == nullptr) {
        return Result::IllegalParameter;
    }
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }
    if (!std::isfinite(theta_)) {
        return Result::ParameterNanOrInf;
    }
    if (!std::isfinite(v_limit_) || !std::isfinite(a_limit_) || !std::isfinite(j_limit_)) {
        return Result::ParameterNanOrInf;
    }
    if (v_limit_ <= 0.0 || a_limit_ <= 0.0 || j_limit_ <= 0.0) {
        return Result::IllegalParameter;
    }
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(pose_start_.p(i)) || !std::isfinite(arc_frame_.p(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(pose_start_.M(r, c)) ||
                !std::isfinite(arc_frame_.M(r, c))) {
                return Result::ParameterNanOrInf;
            }
        }
    }
    if (q_init_.rows() == 0) {
        return Result::IllegalParameter;
    }
    return Result::NoError;
}

// ============================================================================
// 归一化限制
// ============================================================================

bool MoveCircleOffline::computeNormalizedLimits() {
    if (radius_ < kArcEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0; norm_a_ = 1.0; norm_j_ = 1.0;
        return true;
    }

    path_length_ = radius_ * std::abs(theta_);

    if (path_length_ < kArcEpsilon) {
        has_motion_ = false;
        norm_v_ = 1.0; norm_a_ = 1.0; norm_j_ = 1.0;
        return true;
    }

    has_motion_ = true;
    norm_v_ = v_limit_ / path_length_;
    norm_a_ = a_limit_ / path_length_;
    norm_j_ = j_limit_ / path_length_;
    return true;
}

// ============================================================================
// 圆弧位姿插值：p(s) = arc_frame × (R·cos(sθ), R·sin(sθ), 0)  姿态=pose_start.M
// ============================================================================

Frame MoveCircleOffline::interpolatePose(double s) const {
    const double angle = s * theta_;
    const rocos::Vector arc_pos(radius_ * std::cos(angle),
                               radius_ * std::sin(angle),
                               0.0);
    return rocos::Frame(pose_start_.M, arc_frame_ * arc_pos);
}

// ============================================================================
// s_dot → 切向 Twist
//
// 位置导数（世界坐标）：dp/ds = arc_frame_.M × R·θ·(-sin(sθ), cos(sθ), 0)
// v = dp/ds × s_dot
// ω = 0（圆弧保持初始姿态）
// ============================================================================

Twist MoveCircleOffline::computeCartesianTwist(double s_dot) const {
    const double angle = profile_.position() * theta_;

    // 切向单位向量（局部坐标 → 世界）
    const rocos::Vector tangent_local(-std::sin(angle), std::cos(angle), 0.0);
    const rocos::Vector tangent_world = arc_frame_.M * tangent_local;

    // v = R·θ·s_dot（θ 带符号决定方向，不用 abs(path_length_)）
    const double speed = radius_ * theta_ * s_dot;
    const rocos::Vector v = tangent_world * speed;

    return Twist(v, rocos::Vector::Zero());
}

// ============================================================================
// J⁺ · twist → q_dot
// ============================================================================

bool MoveCircleOffline::solveJointVelocity(const JntArray& q,
                                            const Twist& twist,
                                            JntArray& q_dot_out) const {
    Jacobian J;
    if (model_->GetJacobian(q, J) != Result::NoError) {
        return false;
    }

    Eigen::Matrix<double, 6, 1> twist_vec;
    twist_vec << twist.vel.data[0], twist.vel.data[1], twist.vel.data[2],
                 twist.rot.data[0], twist.rot.data[1], twist.rot.data[2];

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        J.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd q_dot = svd.solve(twist_vec);

    if (q_dot_out.rows() != static_cast<unsigned int>(q_dot.size())) {
        q_dot_out.resize(q_dot.size());
    }
    q_dot_out.data = q_dot;
    return true;
}

// ============================================================================
// Reset：离线规划 + 逐点 IK 验证
// ============================================================================

Result MoveCircleOffline::Reset() {
    const Result validation = ValidateParameters();
    if (validation != Result::NoError) return validation;

    if (!computeNormalizedLimits()) return Result::PlanError;
    if (!has_motion_)             return Result::PlanFinished;

    n_joints_ = model_->GetJointNum();
    if (n_joints_ <= 0) return Result::PlanError;

    // ── 跑完整条 S 曲线 ──
    trajectory_.clear();
    profile_.Reset(0.0, 0.0, 0.0);
    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) return Result::PlanError;

    while (true) {
        const int rc = profile_.Update();
        if (rc < 0) {
            trajectory_.clear();
            return Result::PlanError;
        }

        TrajectoryPoint pt;
        pt.s     = profile_.position();
        pt.s_dot = profile_.velocity();
        trajectory_.push_back(std::move(pt));

        if (rc == 0) break;
    }

    // 终点 s=1.0
    if (!trajectory_.empty()) {
        auto& last = trajectory_.back();
        last.s     = 1.0;
        last.s_dot = 0.0;
    }

    // ── 逐点 IK ──
    JntArray q_prev = q_init_;
    for (auto& pt : trajectory_) {
        Frame frame = interpolatePose(pt.s);
        JntArray q_out(n_joints_);

        Result r = model_->InverseKinematics(q_prev, frame, q_out);
        if (r != Result::NoError) {
            trajectory_.clear();
            return Result::PlanError;
        }

        pt.q = std::move(q_out);
        q_prev = pt.q;
    }

    index_     = 0;
    mode_      = Mode::Normal;
    q_current_ = trajectory_[0].q;

    return Result::NoError;
}

// ============================================================================
// 生成当前周期参考值（内含单步推进）
// ============================================================================

Result MoveCircleOffline::GenerateRef(Reference& ref_out) {
    if (!has_motion_ || trajectory_.empty()) return Result::PlanError;

    if (mode_ == Mode::Normal) {
        index_++;
        if (index_ >= trajectory_.size()) {
            ref_out = trajectory_.back().q;
            return Result::PlanFinished;
        }
        ref_out = trajectory_[index_].q;
        return Result::NoError;
    }

    // Mode::Integrate
    const int rc = profile_.Update();
    if (rc < 0) return Result::PlanError;

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
// Normal → Integrate 切换
// ============================================================================

Result MoveCircleOffline::switchToIntegrateMode() {
    if (mode_ == Mode::Integrate) return Result::NoError;
    if (trajectory_.empty())      return Result::PlanError;

    if (index_ >= trajectory_.size()) {
        index_ = trajectory_.size() - 1;
    }

    const auto& pt = trajectory_[index_];
    profile_.Reset(pt.s, pt.s_dot);
    q_current_ = pt.q;
    mode_      = Mode::Integrate;

    return Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
// ============================================================================

Result MoveCircleOffline::Pause() {
    if (!has_motion_) return Result::NoError;

    Result r = switchToIntegrateMode();
    if (r != Result::NoError) return r;

    if (!profile_.Pause(norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

Result MoveCircleOffline::Resume() {
    if (!has_motion_) return Result::NoError;
    profile_.Resume();
    return Result::NoError;
}

Result MoveCircleOffline::Stop() {
    if (!has_motion_) return Result::NoError;

    Result r = switchToIntegrateMode();
    if (r != Result::NoError) return r;

    if (!profile_.Stop(norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

}  // namespace rocos
