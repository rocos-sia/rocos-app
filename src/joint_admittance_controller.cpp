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

#include "joint_admittance_controller.hpp"

#include <cmath>
#include <variant>

namespace rocos {

JointAdmittanceController::~JointAdmittanceController() {
    try {
        if (hardware_ != nullptr && !hardware_->SafetyStopRequested()) {
            hardware_->WaitForSignal();
            adm_initialized_ = false;
            static_cast<void>(UpdateCmd(hardware_->GetPosition()));
        }

    } catch (...) {
        if (hardware_) hardware_->RequestSafetyStop();
    }
}

Result JointAdmittanceController::SetReady() {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_->WaitForSignal();
    adm_initialized_ = false;
    return UpdateCmd(hardware_->GetPosition());
}

// ==========================================================================
// 基础接口
// ==========================================================================

bool JointAdmittanceController::Reset() {
    mode_set_ = false;
    adm_initialized_ = false;
    has_last_cmd_ = false;  // 清除上一周期指令基线，下一次 UpdateCmd 重新建立
    return true;
}

Result JointAdmittanceController::SetHardware(HardwareInterface* hardware) {
    if (hardware == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_ = hardware;
    return Result::NoError;
}

Result JointAdmittanceController::SetModel(ModelInterface* model) {
    if (model == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    model_ = model;
    return Result::NoError;
}

// ==========================================================================
// GenerateCmd — 将 Reference 转为期望关节角 q_des（与 PositionController 一致）
// ==========================================================================

Result JointAdmittanceController::GenerateCmd(const Reference& ref_in,
                                              JntArray& q_cmd) {
    // 分支1: 关节空间参考值 — 直接透传
    if (auto* q_ref = std::get_if<JntArray>(&ref_in)) {
        if (q_ref->rows() == 0) {
            return Result::MoveInput;
        }

        for (unsigned int i = 0; i < q_ref->rows(); ++i) {
            if (!std::isfinite((*q_ref)(i))) {
                return Result::ParameterNanOrInf;
            }
        }

        if (hardware_ != nullptr) {
            JntArray q_curr = hardware_->GetPosition();
            if (q_curr.rows() != q_ref->rows()) {
                return Result::UnmatchedJointsNumber;
            }
        }

        q_cmd = *q_ref;
        return Result::NoError;
    }

    // 分支2: 笛卡尔空间参考值 — 通过逆运动学转为关节角
    if (auto* frame_ref = std::get_if<Frame>(&ref_in)) {
        if (hardware_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }
        if (model_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }

        JntArray q_curr = hardware_->GetPosition();
        if (q_curr.rows() == 0) {
            return Result::JointStateError;
        }

        JntArray q_out;
        q_out.resize(q_curr.rows());
        Result ik_res = model_->InverseKinematics(q_curr, *frame_ref, q_out);
        if (ik_res != Result::NoError) {
            return ik_res;
        }

        for (unsigned int i = 0; i < q_out.rows(); ++i) {
            if (!std::isfinite(q_out(i))) {
                return Result::IkCalcFail;
            }
        }

        q_cmd = q_out;
        return Result::NoError;
    }

    return Result::MoveUnknown;
}

// ==========================================================================
// UpdateCmd — 关节空间导纳控制
//
//   1. τ_ext = τ_act - τ_grav - τ_offset
//   2. M·q̈_adm + B·q̇_adm = τ_ext
//      → q̈_adm = (τ_ext - B·q̇_adm) / M
//   3. 半隐式欧拉积分: q̇_adm += q̈_adm·dt,  q_adm += q̇_adm·dt
//   4. q_out = q_des + q_adm
//   5. 以 CSP 模式下发位置
// ==========================================================================

Result JointAdmittanceController::UpdateCmd(const JntArray& q_des) {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const unsigned int n = q_des.rows();
    if (n == 0) {
        return Result::MoveInput;
    }

    // ---- 增益维度校验（已显式设置的增益必须与 q_des 维度一致） ----
    if ((M_.rows() > 0 && M_.rows() != n) ||
        (B_.rows() > 0 && B_.rows() != n) ||
        (tau_offset_.rows() > 0 && tau_offset_.rows() != n)) {
        return Result::UnmatchedJointsNumber;
    }
    // ---- 参数懒初始化（对尚未设置的增益赋默认值） ----
    if (M_.rows() == 0) {
        M_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            M_(i) = kDefaultInertia;
        }
    }
    if (B_.rows() == 0) {
        B_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            B_(i) = kDefaultDamping;
        }
    }
    if (tau_offset_.rows() == 0) {
        tau_offset_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            tau_offset_(i) = 0.0;
        }
    }

    // ---- 导纳积分状态初始化 ----
    if (!adm_initialized_) {
        q_adm_.resize(n);
        q_dot_adm_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            q_adm_(i) = 0.0;
            q_dot_adm_(i) = 0.0;
        }
        adm_initialized_ = true;
    }

    if (dt_ <= 0.0 || !std::isfinite(dt_)) {
        return Result::IllegalParameter;
    }

    // ---- 读取实际状态 ----
    JntArray q_act = hardware_->GetPosition();
    JntArray q_dot_act = hardware_->GetVelocity();   // 仅用于日志/调试，不参与导纳计算
    JntArray tau_act = hardware_->GetTorque();

    if (q_act.rows() != n || tau_act.rows() != n) {
        return Result::JointStateError;
    }

    // ---- 重力补偿 ----
    JntArray tau_grav;
    if (model_ != nullptr) {
        tau_grav.resize(n);
        JntArray zero_vec(n);
        for (unsigned int i = 0; i < n; ++i) {
            zero_vec(i) = 0.0;
        }
        Wrenches f_ext;  // 空 = 无外力旋量

        Result id_res = model_->InverseDynamics(q_act, zero_vec, zero_vec,
                                                 f_ext, tau_grav);
        if (id_res != Result::NoError) {
            return id_res;
        }
    } else {
        // 无模型时跳过重力补偿
        tau_grav.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            tau_grav(i) = 0.0;
        }
    }

    // ---- 导纳控制律 ----
    // τ_ext = τ_act - τ_grav - τ_offset
    // q̈_adm = (τ_ext - B·q̇_adm) / M
    // 半隐式欧拉: q̇_adm ← q̇_adm + q̈_adm·dt,  q_adm ← q_adm + q̇_adm·dt
    JntArray q_out(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double tau_ext = tau_act(i) - tau_grav(i) - tau_offset_(i);
        const double q_ddot_adm = (tau_ext - B_(i) * q_dot_adm_(i)) / M_(i);

        q_dot_adm_(i) += q_ddot_adm * dt_;
        q_adm_(i)     += q_dot_adm_(i) * dt_;

        q_out(i) = q_des(i) + q_adm_(i);
    }

    // ---- 校验输出有效性 ----
    for (unsigned int i = 0; i < n; ++i) {
        if (!std::isfinite(q_out(i))) {
            return Result::IkCalcFail;
        }
    }

    const auto coupled = hardware_->ApplyPositionCoupling(q_out);
    if (coupled != Result::NoError) return coupled;
    const Result valid = ValidatePositionCommand(q_out);
    if (valid != Result::NoError) {
        return valid;
    }

    // 校验通过后记录本周期指令，作为下一周期指令速度校验的基线
    q_last_cmd_ = q_out;
    has_last_cmd_ = true;

    // ---- 下发位置指令（CSP 模式） ----
    if (!mode_set_) {
        hardware_->SetMode(CSP_MODE);
        mode_set_ = true;
    }

    hardware_->SetPosition(q_out);
    return Result::NoError;
}

Result JointAdmittanceController::ValidatePositionCommand(const JntArray& q_cmd) {
    if (hardware_ == nullptr || model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const JntArray q_actual = hardware_->GetPosition();
    const auto& q_lower = model_->GetPosLowerLimit();
    const auto& q_upper = model_->GetPosUpperLimit();
    const auto& q_vel_limit = model_->GetVelocityLimit();
    const unsigned int joint_num = static_cast<unsigned int>(model_->GetJointNum());

    if (joint_num == 0 || q_cmd.rows() != joint_num || q_actual.rows() != joint_num ||
        q_lower.rows() != joint_num || q_upper.rows() != joint_num ||
        q_vel_limit.rows() != joint_num) {
        return Result::UnmatchedJointsNumber;
    }

    const uint32_t dt_us = hardware_->GetDt();
    if (dt_us == 0) {
        return Result::IllegalParameter;
    }
    const double dt = static_cast<double>(dt_us) / 1'000'000.0;

    // 速度校验比较的是“指令步进”(q_cmd - q_last_cmd_)，而不是“指令 vs 反馈”
    // (q_cmd - q_actual)。反馈位置天然滞后于指令（跟随误差），用它除以 dt 会误报 SpeedLimit。
    const JntArray* previous =
        has_last_cmd_ && q_last_cmd_.rows() == joint_num ? &q_last_cmd_ : nullptr;
    if (!previous) {
        auto coupled_actual = q_actual;
        const auto rc = hardware_->ApplyPositionCoupling(coupled_actual);
        if (rc != Result::NoError) return rc;
        if (!coupled_actual.data.isApprox(q_actual.data, 1e-9)) previous = &q_actual;
    }

    for (unsigned int i = 0; i < joint_num; ++i) {
        if (!std::isfinite(q_cmd(i)) || !std::isfinite(q_actual(i)) ||
            !std::isfinite(q_lower(i)) || !std::isfinite(q_upper(i)) ||
            !std::isfinite(q_vel_limit(i))) {
            return Result::ParameterNanOrInf;
        }
        if (q_cmd(i) < q_lower(i) || q_cmd(i) > q_upper(i)) {
            return Result::PosLimit;
        }
        if (q_vel_limit(i) < 0.0) {
            return Result::IllegalParameter;
        }
        if (previous) {
            const double cmd_velocity = std::abs(q_cmd(i) - (*previous)(i)) / dt;
            if (cmd_velocity > q_vel_limit(i)) {
                return Result::SpeedLimit;
            }
        }
    }

    return Result::NoError;
}

// ==========================================================================
// 导纳参数设置
// ==========================================================================

Result JointAdmittanceController::SetInertia(const JntArray& M) {
    if (M.rows() == 0) {
        return Result::MoveInput;
    }
    for (unsigned int i = 0; i < M.rows(); ++i) {
        if (!std::isfinite(M(i)) || M(i) <= 0.0) {
            return Result::ParameterNanOrInf;
        }
    }
    M_ = M;
    return Result::NoError;
}

Result JointAdmittanceController::SetDamping(const JntArray& B) {
    if (B.rows() == 0) {
        return Result::MoveInput;
    }
    for (unsigned int i = 0; i < B.rows(); ++i) {
        if (!std::isfinite(B(i)) || B(i) < 0.0) {
            return Result::ParameterNanOrInf;
        }
    }
    B_ = B;
    return Result::NoError;
}

Result JointAdmittanceController::SetTorqueOffset(const JntArray& tau_offset) {
    if (tau_offset.rows() == 0) {
        return Result::MoveInput;
    }
    for (unsigned int i = 0; i < tau_offset.rows(); ++i) {
        if (!std::isfinite(tau_offset(i))) {
            return Result::ParameterNanOrInf;
        }
    }
    tau_offset_ = tau_offset;
    return Result::NoError;
}

Result JointAdmittanceController::SetDt(double dt) {
    if (dt <= 0.0 || !std::isfinite(dt)) {
        return Result::IllegalParameter;
    }
    dt_ = dt;
    return Result::NoError;
}

} // namespace rocos
