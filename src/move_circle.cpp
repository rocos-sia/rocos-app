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

#include "move_circle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocos {

namespace {
constexpr double kArcEpsilon = 1e-7;
}  // namespace

// ============================================================================
// 内部重建圆弧局部坐标系
//
// 仅信任外部传入的圆心位置 p 和圆弧平面法向（Z 轴），主动重建正交标架、
// 强制 X 轴 = (pose_start.p - center) 方向。保证 s=0 时 arc_pos = (R,0,0)
// 精确映射到 pose_start_.p，消除起始跳变风险和 TNB 标架歧义。
// ============================================================================

bool MoveCircle::buildArcFrame(const Frame& pose_start,
                               const Frame& center_frame,
                               Frame& arc_frame_out,
                               double& radius_out) {
    const rocos::Vector center = center_frame.p;

    rocos::Vector axis_x = pose_start.p - center;
    radius_out = axis_x.Normalize();
    if (radius_out < kArcEpsilon) return false;

    rocos::Vector axis_z = center_frame.M.UnitZ();
    const double proj = rocos::dot(axis_z, axis_x);
    axis_z -= proj * axis_x;
    if (axis_z.Normalize() < kArcEpsilon) {
        axis_z = rocos::Vector(0, 0, 1);
        if (std::abs(rocos::dot(axis_z, axis_x)) > 0.999) axis_z = rocos::Vector(0, 1, 0);
        axis_z -= rocos::dot(axis_z, axis_x) * axis_x;
        axis_z.Normalize();
    }

    rocos::Vector axis_y = axis_z * axis_x;
    axis_y.Normalize();

    arc_frame_out = rocos::Frame(rocos::Rotation(axis_x, axis_y, axis_z), center);
    return true;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveCircle::MoveCircle(const Frame& pose_start,
                       const Frame& center_frame,
                       double theta,
                       double v_limit,
                       double a_limit,
                       double j_limit,
                       double dt,
                       ModelInterface* model)
    : MotionInterface(model)
    , dt_(dt)
    , profile_(dt)
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

MoveCircle::MoveCircle(const Frame& pose_start,
                       const Frame& pose_via,
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
    , v_limit_(v_limit)
    , a_limit_(a_limit)
    , j_limit_(j_limit) {
    if (!solveThreePoints(pose_start_, pose_via, pose_goal,
                          arc_frame_, radius_, theta_)) {
        theta_ = 0.0;
        radius_ = 0.0;
        arc_frame_ = pose_start_;
    }
}

MoveCircle::~MoveCircle() = default;

// ============================================================================
// 三点解算圆弧：圆心 + 转角 → 正交标架
// ============================================================================

bool MoveCircle::solveThreePoints(const Frame& start,
                                   const Frame& via,
                                   const Frame& goal,
                                   Frame& arc_frame_out,
                                   double& radius_out,
                                   double& theta_out) {
    // ── Step 1: 解圆心 ──
    rocos::Vector v1 = via.p - start.p;
    rocos::Vector v2 = goal.p - start.p;
    if (v1.Normalize() < kArcEpsilon) return false;
    if (v2.Normalize() < kArcEpsilon) return false;

    // Start → Via → Goal 右手螺旋
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

    // ── Step 2: 构建正交标架（X 指向起点）──
    rocos::Vector arc_x = start.p - center;
    radius_out = arc_x.Normalize();
    if (radius_out < kArcEpsilon) return false;

    rocos::Vector arc_z = axis_z;
    arc_z -= rocos::dot(arc_z, arc_x) * arc_x;
    if (arc_z.Normalize() < kArcEpsilon) return false;

    rocos::Vector arc_y = arc_z * arc_x;
    arc_y.Normalize();

    arc_frame_out = rocos::Frame(rocos::Rotation(arc_x, arc_y, arc_z), center);

    // ── Step 3: 转角 ──
    const rocos::Vector end_local = arc_frame_out.Inverse() * goal.p;
    double theta = std::atan2(end_local(1), end_local(0));
    if (theta < 0.0) theta += 2.0 * M_PI;
    theta_out = theta;
    return true;
}

// ============================================================================
// 参数校验
// ============================================================================

Result MoveCircle::ValidateParameters() const {
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        return Result::ParameterNanOrInf;
    }
    if (!std::isfinite(theta_)) {
        return Result::ParameterNanOrInf;
    }
    // 不做 theta 阈值拦截：小角度由 computeNormalizedLimits 设 has_motion_=false
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
    return Result::NoError;
}

// ============================================================================
// 归一化限制计算
// ============================================================================

bool MoveCircle::computeNormalizedLimits() {
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
// 圆弧插值：arc_pos = (R·cos(sθ), R·sin(sθ), 0) in local frame, then to world
// ============================================================================

Frame MoveCircle::interpolateCircular(double s) const {
    const double angle = s * theta_;
    const rocos::Vector arc_pos(radius_ * std::cos(angle),
                               radius_ * std::sin(angle),
                               0.0);
    return rocos::Frame(pose_start_.M, arc_frame_ * arc_pos);
}

// ============================================================================
// 校验 + 初始化
// ============================================================================

Result MoveCircle::Reset() {
    const Result validation = ValidateParameters();
    if (validation != Result::NoError) return validation;

    if (!computeNormalizedLimits()) return Result::PlanError;
    if (!has_motion_)             return Result::PlanFinished;

    if (!profile_.Start(norm_v_, norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

// ============================================================================
// 笛卡尔参考输出（内含 OTG 单步推进）
// ============================================================================

Result MoveCircle::GenerateRef(Reference& ref_out) {
    if (!has_motion_) return Result::PlanFinished;

    const int rc = profile_.Update();
    if (rc < 0) return Result::PlanError;

    ref_out = interpolateCircular(profile_.position());
    return (rc == 0) ? Result::PlanFinished : Result::NoError;
}

// ============================================================================
// 暂停 / 继续 / 停止
// ============================================================================

Result MoveCircle::Pause() {
    if (!has_motion_) return Result::NoError;
    if (!profile_.Pause(norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

Result MoveCircle::Resume() {
    if (!has_motion_) return Result::NoError;
    profile_.Resume();
    return Result::NoError;
}

Result MoveCircle::Stop() {
    if (!has_motion_) return Result::NoError;
    if (!profile_.Stop(norm_a_, norm_j_)) return Result::PlanError;
    return Result::NoError;
}

}  // namespace rocos
