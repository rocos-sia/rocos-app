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

#include "move_line.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

namespace {
// 路径长度阈值：低于此值视为已到位
constexpr double kPathEpsilon = 1e-7;
}  // namespace

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveLine::MoveLine(const Frame& pose_start,
                   const Frame& pose_goal,
                   double v_limit,
                   double a_limit,
                   double j_limit,
                   double dt,
                   ModelInterface* model)
    : MotionInterface(model)
    , dt_(dt)
    , profile_(dt)
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
}

MoveLine::~MoveLine() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveLine::ValidateParameters() const {
    // 1. dt 合法性
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }

    // 2. 一维标量限制：有限性 + 正值性
    if (!std::isfinite(v_limit_) || !std::isfinite(a_limit_) || !std::isfinite(j_limit_)) {
        return Result::ParameterNanOrInf;
    }
    if (v_limit_ <= 0.0 || a_limit_ <= 0.0 || j_limit_ <= 0.0) {
        return Result::IllegalParameter;
    }

    // 3. 起止位姿：位置和旋转分量有限性检查
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(pose_start_.p(i)) || !std::isfinite(pose_goal_.p(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    // 旋转矩阵 9 个分量
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(pose_start_.M(r, c)) ||
                !std::isfinite(pose_goal_.M(r, c))) {
                return Result::ParameterNanOrInf;
            }
        }
    }

    return Result::NoError;
}

// ============================================================================
// 归一化限制计算
// ============================================================================

bool MoveLine::computeNormalizedLimits() {
    // 平移距离
    const double translation = (pose_goal_.p - pose_start_.p).Norm();

    // 旋转角度：从四元数点积 |cos(θ/2)| = |dot(q1,q2)|
    const double cos_half = std::abs(q_start_.dot(q_end_));
    const double rotation = 2.0 * std::acos(std::clamp(cos_half, 0.0, 1.0));

    // 等效路径长度：max(平移, 等效半径 × 旋转弧度)
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

Frame MoveLine::interpolatePose(double s) const {
    Frame result;

    // 平移：线性
    result.p = pose_start_.p + (pose_goal_.p - pose_start_.p) * s;

    // 旋转：Eigen::Quaterniond::slerp（使用构造时缓存的四元数）
    Eigen::Quaterniond q_interp = q_start_.slerp(s, q_end_);
    result.M = rocos::Rotation::Quaternion(
        q_interp.x(), q_interp.y(), q_interp.z(), q_interp.w());

    return result;
}

// ============================================================================
// 校验 + 初始化（合并 support + Reset，父类指针统一入口）
// ============================================================================

Result MoveLine::Reset() {
    const Result validation = ValidateParameters();
    if (validation != Result::NoError) {
        return validation;
    }

    if (!computeNormalizedLimits()) {
        return Result::PlanError;
    }

    if (!has_motion_) {
        return Result::PlanFinished;
    }

    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) {
        return Result::PlanError;
    }

    return Result::NoError;
}

// ============================================================================
// 生成当前周期的笛卡尔参考位姿（内含 OTG 单步推进）
// ============================================================================

Result MoveLine::GenerateRef(Reference& ref_out) {
    if (!has_motion_) {
        return Result::PlanFinished;
    }

    const int rc = profile_.Update();
    if (rc < 0) {
        return Result::PlanError;
    }

    const double s = profile_.position();
    ref_out = interpolatePose(s);
    return (rc == 0) ? Result::PlanFinished : Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
// ============================================================================

Result MoveLine::Pause() {
    if (!has_motion_) {
        return Result::NoError;
    }
    if (!profile_.Pause(norm_a_, norm_j_)) {
        return Result::PlanError;
    }
    return Result::NoError;
}

Result MoveLine::Resume() {
    if (!has_motion_) {
        return Result::NoError;
    }
    profile_.Resume();
    return Result::NoError;
}

Result MoveLine::Stop() {
    if (!has_motion_) {
        return Result::NoError;
    }
    if (!profile_.Stop(norm_a_, norm_j_)) {
        return Result::PlanError;
    }
    return Result::NoError;
}

}  // namespace rocos
