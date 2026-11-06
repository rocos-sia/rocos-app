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

#include "joint_impedance_controller.hpp"

#include <cmath>
#include <algorithm>
#include <variant>

namespace rocos {

JointImpedanceController::~JointImpedanceController() {
    try {
        if (hardware_ && !hardware_->SafetyStopRequested()) {
            hardware_->WaitForSignal();
            static_cast<void>(UpdateCmd(hardware_->GetPosition()));
        }
    } catch (...) {
        if (hardware_) hardware_->RequestSafetyStop();
    }
}

Result JointImpedanceController::SetReady() {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    hardware_->WaitForSignal();
    return UpdateCmd(hardware_->GetPosition());
}

// ==========================================================================
// 基础接口
// ==========================================================================

bool JointImpedanceController::Reset() {
    mode_set_ = false;
    has_last_torque_ = false;
    return true;
}

Result JointImpedanceController::SetHardware(HardwareInterface* hardware) {
    if (hardware == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_ = hardware;
    return Result::NoError;
}

Result JointImpedanceController::SetModel(ModelInterface* model) {
    if (model == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    model_ = model;
    return Result::NoError;
}

// ==========================================================================
// GenerateCmd — 将 Reference 转为期望关节角 q_des（与 PositionController 一致）
// ==========================================================================

Result JointImpedanceController::GenerateCmd(const Reference& ref_in,
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
// UpdateCmd — 关节空间阻抗控制（输出力矩 + 重力补偿）
//
//   τ_cmd = K_p·(q_des - q_act) - K_d·q̇_act + τ_grav
//
//   其中 τ_grav = InverseDynamics(q_act, 0, 0, ∅)
// ==========================================================================

Result JointImpedanceController::UpdateCmd(const JntArray& q_des) {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const unsigned int n = q_des.rows();
    if (n == 0) {
        return Result::MoveInput;
    }
    if (model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    // ---- 增益懒初始化 ----
    // 先检查已显式设置的增益维度是否与 q_des 一致
    if ((K_p_.rows() > 0 && K_p_.rows() != n) ||
        (K_d_.rows() > 0 && K_d_.rows() != n)) {
        return Result::UnmatchedJointsNumber;
    }
    // 对尚未设置的增益赋默认值
    if (K_p_.rows() == 0) {
        K_p_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            K_p_(i) = kDefaultStiffness;
        }
    }
    if (K_d_.rows() == 0) {
        K_d_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            K_d_(i) = kDefaultDamping;
        }
    }

    // ---- 读取实际关节状态 ----
    JntArray q_act = hardware_->GetPosition();
    JntArray q_dot_act = hardware_->GetVelocity();

    if (q_act.rows() != n) {
        return Result::JointStateError;
    }

    // ---- 重力补偿: τ_grav = InverseDynamics(q, 0, 0, ∅) ----
    JntArray tau_grav(n);
    JntArray zero(n);
    for (unsigned int i = 0; i < n; ++i) {
        zero(i) = 0.0;
    }
    Wrenches f_ext;

    Result id_res = model_->InverseDynamics(q_act, zero, zero, f_ext, tau_grav);
    if (id_res != Result::NoError) {
        return id_res;
    }

    // ---- 阻抗控制律: τ = K_p·(q_des - q_act) - K_d·q̇_act + τ_grav ----
    JntArray tau(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double pos_err = q_des(i) - q_act(i);
        const double vel_damp = (q_dot_act.rows() == n) ? K_d_(i) * q_dot_act(i) : 0.0;
        tau(i) = K_p_(i) * pos_err - vel_damp + tau_grav(i);
    }

    const Result valid = ValidateTorqueCommand(tau);
    if (valid != Result::NoError) {
        return valid;
    }

    const auto limits=hardware_->GetJointDerivativeLimits();
    const double period=hardware_->GetDt()*1e-6;
    if (!(period>0)) return Result::IllegalParameter;
    if (!has_last_torque_) {
        const auto previous=hardware_->GetOutputSnapshot();
        last_torque_=previous.mode==CST_MODE ? previous.torque : hardware_->GetTorque();
        if (last_torque_.rows()!=n) return Result::UnmatchedJointsNumber;
    }
    for (unsigned int i=0;i<n;++i) {
        const double rate=limits.size()==n ? limits[i].torque_rate : 200.0;
        if (!std::isfinite(last_torque_(i)) || !std::isfinite(rate) || rate<=0) return Result::IllegalParameter;
        tau(i)=std::clamp(tau(i),last_torque_(i)-rate*period,last_torque_(i)+rate*period);
    }
    const auto limited_valid=ValidateTorqueCommand(tau);
    if (limited_valid!=Result::NoError) return limited_valid;
    // ---- 下发力矩指令（CST 模式） ----
    if (!mode_set_) {
        hardware_->SetMode(CST_MODE);
        mode_set_ = true;
    }

    hardware_->SetTorque(tau);
    last_torque_=tau;
    has_last_torque_=true;
    return Result::NoError;
}

Result JointImpedanceController::ValidateTorqueCommand(const JntArray& tau_cmd) {
    if (hardware_ == nullptr || model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const JntArray q_actual = hardware_->GetPosition();
    const JntArray q_dot_actual = hardware_->GetVelocity();
    const auto& q_lower = model_->GetPosLowerLimit();
    const auto& q_upper = model_->GetPosUpperLimit();
    const auto& q_vel_limit = model_->GetVelocityLimit();
    const auto& q_effort_limit = model_->GetEffortLimit();
    const unsigned int joint_num = static_cast<unsigned int>(model_->GetJointNum());

    if (joint_num == 0 || tau_cmd.rows() != joint_num || q_actual.rows() != joint_num ||
        q_lower.rows() != joint_num || q_upper.rows() != joint_num ||
        q_vel_limit.rows() != joint_num || q_effort_limit.rows() != joint_num) {
        return Result::UnmatchedJointsNumber;
    }

    const bool has_velocity = (q_dot_actual.rows() == joint_num);
    if (q_dot_actual.rows() != 0 && !has_velocity) {
        return Result::UnmatchedJointsNumber;
    }

    for (unsigned int i = 0; i < joint_num; ++i) {
        if (!std::isfinite(tau_cmd(i)) || !std::isfinite(q_actual(i)) ||
            !std::isfinite(q_lower(i)) || !std::isfinite(q_upper(i)) ||
            !std::isfinite(q_vel_limit(i)) || !std::isfinite(q_effort_limit(i)) ||
            (has_velocity && !std::isfinite(q_dot_actual(i)))) {
            return Result::ParameterNanOrInf;
        }
        if (q_actual(i) < q_lower(i) || q_actual(i) > q_upper(i)) {
            return Result::PosLimit;
        }
        if (q_vel_limit(i) < 0.0 || q_effort_limit(i) < 0.0) {
            return Result::IllegalParameter;
        }
        if (has_velocity && std::abs(q_dot_actual(i)) > q_vel_limit(i)) {
            return Result::SpeedLimit;
        }
        if (std::abs(tau_cmd(i)) > q_effort_limit(i)) {
            return Result::ForceLimit;
        }
    }

    return Result::NoError;
}

// ==========================================================================
// 阻抗参数设置
// ==========================================================================

Result JointImpedanceController::SetStiffness(const JntArray& K) {
    if (K.rows() == 0) {
        return Result::MoveInput;
    }
    for (unsigned int i = 0; i < K.rows(); ++i) {
        if (!std::isfinite(K(i)) || K(i) < 0.0) {
            return Result::ParameterNanOrInf;
        }
    }
    K_p_ = K;
    return Result::NoError;
}

Result JointImpedanceController::SetDamping(const JntArray& D) {
    if (D.rows() == 0) {
        return Result::MoveInput;
    }
    for (unsigned int i = 0; i < D.rows(); ++i) {
        if (!std::isfinite(D(i)) || D(i) < 0.0) {
            return Result::ParameterNanOrInf;
        }
    }
    K_d_ = D;
    return Result::NoError;
}

Result JointImpedanceController::SetInertia(const JntArray& M) {
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

Result JointImpedanceController::SetTorqueOffset(const JntArray& tau_offset) {
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

Result JointImpedanceController::SetDt(double dt) {
    if (dt <= 0.0 || !std::isfinite(dt)) {
        return Result::IllegalParameter;
    }
    dt_ = dt;
    return Result::NoError;
}

} // namespace rocos
