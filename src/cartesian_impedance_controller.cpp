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

#include "cartesian_impedance_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <variant>

namespace rocos {

CartesianImpedanceController::~CartesianImpedanceController() {
    try {
        if (hardware_ != nullptr && !hardware_->SafetyStopRequested()) {
            hardware_->WaitForSignal();
            JntArray q_hold = hardware_->GetPosition();
            JntArray q_generated;
            x_des_valid_ = false;
            x_des_from_frame_ = false;
            tau_prev_valid_ = false;
            static_cast<void>(GenerateCmd(Reference{q_hold}, q_generated));
            static_cast<void>(UpdateCmd(q_hold));
        }

    } catch (...) {
        if (hardware_) hardware_->RequestSafetyStop();
    }
}

// ==========================================================================
// SetReady — 初始化硬件并发送重力补偿力矩
// ==========================================================================

Result CartesianImpedanceController::SetReady() {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    hardware_->WaitForSignal();
    JntArray q_hold = hardware_->GetPosition();
    JntArray q_generated;
    x_des_valid_ = false;
    x_des_from_frame_ = false;
    tau_prev_valid_ = false;
    const Result gen_res = GenerateCmd(Reference{q_hold}, q_generated);
    if (gen_res != Result::NoError) {
        return gen_res;
    }
    return UpdateCmd(q_hold);
}

// ==========================================================================
// 基础接口
// ==========================================================================

bool CartesianImpedanceController::Reset() {
    mode_set_ = false;
    x_des_valid_ = false;
    x_des_from_frame_ = false;
    tau_prev_valid_ = false;
    return true;
}

Result CartesianImpedanceController::SetHardware(HardwareInterface* hardware) {
    if (hardware == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_ = hardware;
    return Result::NoError;
}

Result CartesianImpedanceController::SetModel(ModelInterface* model) {
    if (model == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    model_ = model;
    return Result::NoError;
}

// ==========================================================================
// GenerateCmd — 将 Reference 转为期望笛卡尔位姿 x_des_
// ==========================================================================

Result CartesianImpedanceController::GenerateCmd(const Reference& ref_in,
                                                  JntArray& q_cmd) {
    // 分支1: 关节空间参考值 — 通过 FK 转为笛卡尔位姿
    if (auto* q_ref = std::get_if<JntArray>(&ref_in)) {
        if (q_ref->rows() == 0) {
            return Result::MoveInput;
        }

        for (unsigned int i = 0; i < q_ref->rows(); ++i) {
            if (!std::isfinite((*q_ref)(i))) {
                return Result::ParameterNanOrInf;
            }
        }

        if (model_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }

        Frame x_fk;
        Result fk_res = model_->ForwardKinematics(*q_ref, x_fk);
        if (fk_res != Result::NoError) {
            return fk_res;
        }

        // x_des_ 来自 Frame（笛卡尔运动如 MoveL）时，阻止 hold_position_ 的
        // JntArray 覆盖，避免运动结束后因柔顺跟踪滞后导致 x_des_ 跳变→放弃剩余误差。
        // 但 x_des_ 来自 JntArray（初始设置或 MoveJ）时，允许继续更新。
        if (!x_des_from_frame_) {
            x_des_ = x_fk;
            x_des_valid_ = true;
        }

        q_cmd = *q_ref;
        return Result::NoError;
    }

    // 分支2: 笛卡尔空间参考值 — 直接存储
    if (auto* frame_ref = std::get_if<Frame>(&ref_in)) {
        x_des_ = *frame_ref;
        x_des_valid_ = true;
        x_des_from_frame_ = true;  // 标记来源为 Frame，保护不被 hold_position_ 覆盖

        // 通过 IK 将期望位姿转为关节角
        if (hardware_ != nullptr && model_ != nullptr) {
            JntArray q_curr = hardware_->GetPosition();
            if (q_curr.rows() > 0) {
                JntArray q_out;
                q_out.resize(q_curr.rows());
                Result ik_res = model_->InverseKinematics(q_curr, *frame_ref, q_out);
                if (ik_res == Result::NoError) {
                    q_cmd = q_out;
                } else {
                    // IK 失败时保持当前关节角
                    q_cmd = q_curr;
                }
            }
        }
        return Result::NoError;
    }

    return Result::MoveUnknown;
}

// ==========================================================================
// UpdateCmd — 笛卡尔空间阻抗控制 + 力矩安全保护
//
//   核心公式：
//     Δx_base = diff(x_cur, x_des)          ← 6D 位姿误差（基坐标系，rocos::diff 定义）
//     Δx_tool = R_cur^T · Δx_base           ← 误差变换到工具系
//     v_base   = J · q̇_act                   ← 末端速度（基坐标系）
//     v_cur    = R_cur^T · v_base            ← 速度变换到工具系
//     F_cur    = K_p·Δx_tool - K_d·v_cur    ← 阻抗力/力矩（工具系）
//     F_base   = R_cur · F_cur              ← 转回基坐标系
//     τ_imp    = J^T · F_base                ← Jacobian 转置映射到关节力矩
//     τ_raw    = τ_imp + τ_grav
//     经过变化率限制和饱和后下发
// ==========================================================================

Result CartesianImpedanceController::UpdateCmd(const JntArray& q_cmd) {
    (void)q_cmd;  // 笛卡尔阻抗控制器使用内部 x_des_，不使用 q_cmd

    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (!x_des_valid_) {
        return Result::MoveInput;  // 尚未收到期望位姿
    }

    const unsigned int n = hardware_->GetPosition().rows();
    if (n == 0) {
        return Result::JointStateError;
    }

    // ---- 读取实际关节状态 ----
    JntArray q_act = hardware_->GetPosition();
    JntArray q_dot_act = hardware_->GetVelocity();

    if (q_act.rows() != n) {
        return Result::JointStateError;
    }

    const bool has_velocity = (q_dot_act.rows() == n);

    // ---- 1. FK → 当前笛卡尔位姿 x_cur ----
    Frame x_cur;
    Result fk_res = model_->ForwardKinematics(q_act, x_cur);
    if (fk_res != Result::NoError) {
        return fk_res;
    }

    // ---- 2. Jacobian J(q) (6×n) ----
    Jacobian J_out;
    Result jac_res = model_->GetJacobian(q_act, J_out);
    if (jac_res != Result::NoError) {
        return jac_res;
    }

    // 提取当前末端姿态矩阵，后续多处使用
    const rocos::Rotation& R_cur = x_cur.M;

    // ---- 3. 6D 位姿误差 Δx = diff(x_cur, x_des)（基坐标系下表达） ----
    // 注意: rocos::diff(F_a_b1, F_a_b2) 返回的 twist 在基坐标系 a 下表达
    Twist x_err_base = rocos::diff(x_cur, x_des_);

    // ---- 4. 末端速度 v_base = J · q̇（基坐标系） ----
    Twist v_base;
    if (has_velocity) {
        Eigen::VectorXd qd(n);
        for (unsigned int i = 0; i < n; ++i) {
            qd(i) = q_dot_act(i);
        }
        Eigen::Matrix<double, 6, 1> v_ee = J_out.data * qd;
        v_base.vel = Vector(v_ee(0), v_ee(1), v_ee(2));
        v_base.rot = Vector(v_ee(3), v_ee(4), v_ee(5));
    }

    // ---- 5. 位姿误差和速度变换到工具系 ----
    // Δx_tool = Ad_{g^{-1}} · Δx_base:  vel_tool = R_cur^T · vel_base
    Twist x_err_tool, v_cur;
    x_err_tool.vel = R_cur.Inverse() * x_err_base.vel;
    x_err_tool.rot = R_cur.Inverse() * x_err_base.rot;

    if (has_velocity) {
        v_cur.vel = R_cur.Inverse() * v_base.vel;
        v_cur.rot = R_cur.Inverse() * v_base.rot;
    }

    // ---- 6. 阻抗力/力矩（工具系下） ----
    Wrench F_cur;
    F_cur.force  = Vector(
        K_p_lin_ * x_err_tool.vel.x() - K_d_lin_ * v_cur.vel.x(),
        K_p_lin_ * x_err_tool.vel.y() - K_d_lin_ * v_cur.vel.y(),
        K_p_lin_ * x_err_tool.vel.z() - K_d_lin_ * v_cur.vel.z());
    F_cur.torque = Vector(
        K_p_ang_ * x_err_tool.rot.x() - K_d_ang_ * v_cur.rot.x(),
        K_p_ang_ * x_err_tool.rot.y() - K_d_ang_ * v_cur.rot.y(),
        K_p_ang_ * x_err_tool.rot.z() - K_d_ang_ * v_cur.rot.z());

    // ---- 7. 力/力矩变换到基坐标系：F_base = Ad_g · F_cur = R_cur · F_cur ----
    Wrench F_base;
    F_base.force  = R_cur * F_cur.force;
    F_base.torque = R_cur * F_cur.torque;

    // ---- 8. Jacobian 转置映射到关节力矩：τ_imp = J^T · F_base ----
    Eigen::Matrix<double, 6, 1> F_vec;
    F_vec(0) = F_base.force.x();  F_vec(1) = F_base.force.y();  F_vec(2) = F_base.force.z();
    F_vec(3) = F_base.torque.x(); F_vec(4) = F_base.torque.y(); F_vec(5) = F_base.torque.z();

    Eigen::VectorXd tau_imp_eig = J_out.data.transpose() * F_vec;

    // ---- 9. 重力补偿：τ_grav = InverseDynamics(q, 0, 0, ∅) ----
    JntArray tau_grav(n);
    {
        JntArray zero(n);
        for (unsigned int i = 0; i < n; ++i) {
            zero(i) = 0.0;
        }
        Wrenches f_ext;
        Result id_res = model_->InverseDynamics(q_act, zero, zero, f_ext, tau_grav);
        if (id_res != Result::NoError) {
            return id_res;
        }
    }

    // ---- 10. 合成原始力矩 τ_raw = τ_imp + τ_grav ----
    JntArray tau_raw(n);
    for (unsigned int i = 0; i < n; ++i) {
        tau_raw(i) = tau_imp_eig(i) + tau_grav(i);
    }

    // ---- 11. 力矩变化率限制 ----
    JntArray tau(n);
    if (tau_rate_limit_ > 0.0 && tau_prev_valid_ && tau_prev_.rows() == n) {
        const double max_delta = tau_rate_limit_ * dt_;
        for (unsigned int i = 0; i < n; ++i) {
            double delta = tau_raw(i) - tau_prev_(i);
            delta = std::max(-max_delta, std::min(max_delta, delta));
            tau(i) = tau_prev_(i) + delta;
        }
    } else {
        tau = tau_raw;
    }

    // ---- 12. 力矩饱和 ----
    if (tau_max_ > 0.0) {
        for (unsigned int i = 0; i < n; ++i) {
            tau(i) = std::max(-tau_max_, std::min(tau_max_, tau(i)));
        }
    }

    // ---- 13. URDF limit 保护 ----
    const Result valid = ValidateTorqueCommand(tau);
    if (valid != Result::NoError) {
        return valid;
    }

    // ---- 14. 保存当前力矩供下一周期变化率限制 ----
    tau_prev_ = tau;
    tau_prev_valid_ = true;

    // ---- 15. 下发力矩指令（CST 模式） ----
    if (!mode_set_) {
        hardware_->SetMode(CST_MODE);
        mode_set_ = true;
    }

    hardware_->SetTorque(tau);
    return Result::NoError;
}

Result CartesianImpedanceController::ValidateTorqueCommand(const JntArray& tau_cmd) {
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
// 笛卡尔阻抗参数
// ==========================================================================

Result CartesianImpedanceController::SetTranslationalStiffness(double K) {
    if (!std::isfinite(K) || K < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_p_lin_ = K;
    return Result::NoError;
}

Result CartesianImpedanceController::SetRotationalStiffness(double K) {
    if (!std::isfinite(K) || K < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_p_ang_ = K;
    return Result::NoError;
}

Result CartesianImpedanceController::SetTranslationalDamping(double D) {
    if (!std::isfinite(D) || D < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_d_lin_ = D;
    return Result::NoError;
}

Result CartesianImpedanceController::SetRotationalDamping(double D) {
    if (!std::isfinite(D) || D < 0.0) {
        return Result::ParameterNanOrInf;
    }
    K_d_ang_ = D;
    return Result::NoError;
}

// ==========================================================================
// 力矩安全参数
// ==========================================================================

Result CartesianImpedanceController::SetTorqueRateLimit(double limit) {
    if (!std::isfinite(limit) || limit < 0.0) {
        return Result::ParameterNanOrInf;
    }
    tau_rate_limit_ = limit;
    return Result::NoError;
}

Result CartesianImpedanceController::SetTorqueSaturation(double limit) {
    if (!std::isfinite(limit) || limit < 0.0) {
        return Result::ParameterNanOrInf;
    }
    tau_max_ = limit;
    return Result::NoError;
}

} // namespace rocos
