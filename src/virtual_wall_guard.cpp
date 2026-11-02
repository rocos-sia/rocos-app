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

#include "virtual_wall_guard.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace rocos {

// ============================================================================
// 构造 / 析构
// ============================================================================

VirtualWallGuard::VirtualWallGuard(
    std::unique_ptr<ControllerInterface> inner,
    ModelInterface* model,
    HardwareInterface* hardware)
    : inner_(std::move(inner))
    , model_(model)
    , hardware_(hardware) {}

VirtualWallGuard::~VirtualWallGuard() {
    if (hardware_ != nullptr) {
        // 安全兜底：先同步硬件，再将目标位置设为当前位置，切换到 CSP 模式，防止析构时飞车
        hardware_->WaitForSignal();
        hardware_->SetPosition(hardware_->GetPosition());
        hardware_->SetMode(8);
    }
}

Result VirtualWallGuard::SetReady() {
    // 透传给内部控制器；若无 inner_ 则用自身 hardware_ 兜底
    if (inner_) {
        return inner_->SetReady();
    }
    if (hardware_ != nullptr) {
        hardware_->WaitForSignal();
        hardware_->SetPosition(hardware_->GetPosition());
        hardware_->SetMode(8);
    }
    return Result::NoError;
}

// ============================================================================
// ControllerInterface 接口 — 全部透传给 inner_
// ============================================================================

bool VirtualWallGuard::Reset() {
    if (inner_) inner_->Reset();
    return true;
}

Result VirtualWallGuard::SetHardware(HardwareInterface* hardware) {
    hardware_ = hardware;
    if (inner_) return inner_->SetHardware(hardware);
    return Result::NoError;
}

Result VirtualWallGuard::SetModel(ModelInterface* model) {
    model_ = model;
    if (inner_) return inner_->SetModel(model);
    return Result::NoError;
}

Result VirtualWallGuard::UpdateCmd(const JntArray& q_cmd) {
    if (inner_) return inner_->UpdateCmd(q_cmd);
    return Result::NoError;
}

// ============================================================================
// TCP 位置计算
// ============================================================================

Result VirtualWallGuard::computeTargetTcp(const Reference& ref_in,
                                           Vector& tcp_out) const {
    if (auto* jnt = std::get_if<JntArray>(&ref_in)) {
        if (model_ == nullptr) return Result::ParameterPointerEqualsNullptr;
        Frame f;
        Result r = model_->ForwardKinematics(*jnt, f);
        if (r != Result::NoError) return r;
        tcp_out = f.p;
        return Result::NoError;
    }

    if (auto* frame = std::get_if<Frame>(&ref_in)) {
        tcp_out = frame->p;
        return Result::NoError;
    }

    return Result::MoveUnknown;
}

Result VirtualWallGuard::computeCurrentTcp(Vector& tcp_out) const {
    if (hardware_ == nullptr) return Result::ParameterPointerEqualsNullptr;
    if (model_ == nullptr)    return Result::ParameterPointerEqualsNullptr;

    JntArray q_curr = hardware_->GetPosition();
    if (q_curr.rows() == 0) return Result::JointStateError;

    Frame f_curr;
    Result r = model_->ForwardKinematics(q_curr, f_curr);
    if (r != Result::NoError) return r;

    tcp_out = f_curr.p;
    return Result::NoError;
}

// ============================================================================
// targetPosToRef — 将修改后的 TCP 位置转回 Reference
// ============================================================================

Reference VirtualWallGuard::targetPosToRef(
    const Vector& new_pos, const Reference& original_ref) const {

    if (auto* frame = std::get_if<Frame>(&original_ref)) {
        // Frame 引用：直接替换位置，姿态不变
        Frame out = *frame;
        out.p = new_pos;
        return out;
    }

    if (std::holds_alternative<JntArray>(original_ref)) {
        // JntArray 引用：需通过 IK 反算关节角
        if (hardware_ == nullptr || model_ == nullptr)
            return original_ref;  // 无法做 IK，保持原样

        JntArray q_curr = hardware_->GetPosition();
        if (q_curr.rows() == 0)
            return original_ref;

        // 构建目标 Frame（保持当前姿态不变）
        Frame f_curr;
        if (model_->ForwardKinematics(q_curr, f_curr) != Result::NoError)
            return original_ref;

        Frame target_frame(f_curr.M, new_pos);
        JntArray q_out(q_curr.rows());
        Result ik_res = model_->InverseKinematics(q_curr, target_frame, q_out);

        if (ik_res == Result::NoError)
            return q_out;

        // IK 失败 → 用关节空间线性缩放兜底
        const auto& q_target = std::get<JntArray>(original_ref);
        double scale = 0.5;  // 保守回退一半
        JntArray q_fallback(q_target.rows());
        for (unsigned int i = 0; i < q_target.rows(); ++i)
            q_fallback(i) = q_curr(i) + scale * (q_target(i) - q_curr(i));
        return q_fallback;
    }

    return original_ref;
}

// ============================================================================
// fallbackGenerateCmd — 无 current_pos 时的兜底逻辑
//   由于无法计算运动方向，退化为纯绝对位置检查
// ============================================================================

Result VirtualWallGuard::fallbackGenerateCmd(const Reference& ref_in,
                                              JntArray& q_cmd) {
    Vector target_pos;
    Result res = computeTargetTcp(ref_in, target_pos);
    if (res != Result::NoError)
        return inner_->GenerateCmd(ref_in, q_cmd);

    for (const auto& wall : walls_) {
        const double d = signedDistance(wall, target_pos);
        if (d >= 0.0)
            return Result::SlopoverVirtualWall;
    }

    return inner_->GenerateCmd(ref_in, q_cmd);
}

// ============================================================================
// 四元数 slerp（scaleBack 对 Frame 使用）
// ============================================================================

static std::array<double, 4> slerpQuat(
    const std::array<double, 4>& q0,
    const std::array<double, 4>& q1,
    double t) {

    double cos_theta = q0[0] * q1[0] + q0[1] * q1[1]
                     + q0[2] * q1[2] + q0[3] * q1[3];

    std::array<double, 4> q1_mod = q1;
    if (cos_theta < 0.0) {
        for (auto& v : q1_mod) v = -v;
        cos_theta = -cos_theta;
    }

    constexpr double kSlerpEps = 0.9995;
    if (cos_theta > kSlerpEps) {
        std::array<double, 4> result;
        for (int i = 0; i < 4; ++i)
            result[i] = q0[i] + t * (q1_mod[i] - q0[i]);
        double norm = 0.0;
        for (double v : result) norm += v * v;
        norm = std::sqrt(norm);
        for (double& v : result) v /= norm;
        return result;
    }

    const double theta = std::acos(cos_theta);
    const double sin_theta = std::sin(theta);
    const double w0 = std::sin((1.0 - t) * theta) / sin_theta;
    const double w1 = std::sin(t * theta) / sin_theta;

    return {{
        w0 * q0[0] + w1 * q1_mod[0],
        w0 * q0[1] + w1 * q1_mod[1],
        w0 * q0[2] + w1 * q1_mod[2],
        w0 * q0[3] + w1 * q1_mod[3]
    }};
}

// ============================================================================
// GenerateCmd — 核心：基于运动方向的虚拟墙检测
//
// 决策矩阵（对每个墙）：
//   d_curr   d_target   v_n        → 动作
//   < 0      < 0        任意       → ✅ 全速放行
//   < 0      ≥ 0        > 0        → ✂️ 投影到边界（从安全区往外冲 → 允许滑）
//   < 0      ≥ 0        ≤ 0        → ✂️ 投影到边界（不应发生，兜底）
//   ≥ 0      ≥ 0        > 0        → ❌ 硬拦截（已在外，继续往外）
//   ≥ 0      ≥ 0        ≤ 0        → ✂️ 投影到边界（从外退回 → 允许）
//   < 0      -wd~0      > 0        → ⚠️ 只缩减法向分量（切向全速）
// ============================================================================

Result VirtualWallGuard::GenerateCmd(const Reference& ref_in,
                                      JntArray& q_cmd) {
    if (!enabled_ || walls_.empty())
        return inner_->GenerateCmd(ref_in, q_cmd);

    // 计算目标 TCP
    Vector target_pos;
    if (computeTargetTcp(ref_in, target_pos) != Result::NoError)
        return inner_->GenerateCmd(ref_in, q_cmd);

    // 计算当前 TCP（如无硬件 → 退化为纯绝对位置检查）
    Vector current_pos;
    if (computeCurrentTcp(current_pos) != Result::NoError)
        return fallbackGenerateCmd(ref_in, q_cmd);

    // 逐墙检查
    bool target_modified = false;

    for (const auto& wall : walls_) {
        const double d_curr   = signedDistance(wall, current_pos);
        const double d_target = signedDistance(wall, target_pos);

        // ── 目标在禁止区 ──
        if (d_target >= 0.0) {
            const Vector n = wallNormal(wall, current_pos);
            const Vector delta = target_pos - current_pos;
            const double v_n = rocos::dot(delta, n);  // 法向速度分量

            if (d_curr >= 0.0 && v_n > 0.0) {
                // 已在禁止区 + 继续向禁止方向移动 → 硬拦截
                return Result::SlopoverVirtualWall;
            }

            // 其他情况：投影到边界
            //   (d_curr < 0):         从安全区溢出 → 拉回到边界，允许退回
            //   (d_curr ≥ 0, v_n ≤ 0): 已在外但向回走 → 拉到边界，允许沿墙滑
            target_pos = projectOntoBoundary(wall, target_pos);
            target_modified = true;
            continue;
        }

        // ── 目标在减速区 且 正在向墙移动 ──
        const double wd = getWarningDistance(wall);
        if (d_target > -wd) {
            const Vector n = wallNormal(wall, current_pos);
            const Vector delta = target_pos - current_pos;
            const double v_n = rocos::dot(delta, n);

            if (v_n > 0.0) {
                // 向禁止侧移动 → 只缩减法向分量，切向保持全速
                const double scale = std::abs(d_target) / wd;
                const Vector delta_normal = v_n * n;
                const Vector delta_tangential = delta - delta_normal;
                target_pos = current_pos
                           + delta_tangential            // 切向全速
                           + scale * delta_normal;       // 法向减速
                target_modified = true;
            }
            // v_n ≤ 0 → 正在离开墙，不在减速区内减速
        }
    }

    if (target_modified) {
        Reference modified_ref = targetPosToRef(target_pos, ref_in);
        return inner_->GenerateCmd(modified_ref, q_cmd);
    }

    return inner_->GenerateCmd(ref_in, q_cmd);
}

// ============================================================================
// 虚拟墙管理
// ============================================================================

void VirtualWallGuard::AddWall(const WallVariant& wall) {
    walls_.push_back(wall);
}

void VirtualWallGuard::ClearWalls() {
    walls_.clear();
}

size_t VirtualWallGuard::GetWallCount() const {
    return walls_.size();
}

void VirtualWallGuard::SetEnabled(bool enabled) {
    enabled_ = enabled;
}

bool VirtualWallGuard::IsEnabled() const {
    return enabled_;
}

}  // namespace rocos
