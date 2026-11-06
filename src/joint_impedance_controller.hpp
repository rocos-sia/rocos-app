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
#pragma once

#include "controller_interface.hpp"

namespace rocos {

/// @brief 关节空间阻抗控制器（输出力矩 + 重力补偿）
///
/// 控制律（每个周期执行）：
///   1. τ_grav = InverseDynamics(q_act, 0, 0, ∅)   ← 重力补偿
///   2. τ_imp  = K_p·(q_des - q_act) - K_d·q̇_act  ← PD 阻抗
///   3. τ_cmd  = τ_imp + τ_grav
///   4. SetTorque(τ_cmd)，CST 模式 (mode=10)
///
/// 最终下发的始终是关节力矩。
class JointImpedanceController : public ControllerInterface {
public:
    JointImpedanceController() = default;
    ~JointImpedanceController() override;

    bool Reset() override;
    Result SetReady() override;
    Result SetHardware(HardwareInterface* hardware) override;
    Result SetModel(ModelInterface* model) override;
    Result GenerateCmd(const Reference& ref_in, JntArray& q_cmd) override;
    Result UpdateCmd(const JntArray& q_cmd) override;

    /// @brief 设置关节刚度系数 K_p [Nm/rad]
    Result SetStiffness(const JntArray& K);
    /// @brief 设置关节阻尼系数 K_d [Nm·s/rad]
    Result SetDamping(const JntArray& D);

    /// @brief 获取关节刚度系数 K_p [Nm/rad]
    [[nodiscard]] const JntArray& GetStiffness() const { return K_p_; }
    /// @brief 获取关节阻尼系数 K_d [Nm·s/rad]
    [[nodiscard]] const JntArray& GetDamping() const { return K_d_; }

    /// @brief 设置导纳虚拟惯量 M [kg·m²]（导纳模式参数）
    Result SetInertia(const JntArray& M);
    /// @brief 设置关节力矩零飘 τ_offset [Nm]（导纳模式参数）
    Result SetTorqueOffset(const JntArray& tau_offset);
    /// @brief 设置控制周期 dt [s]（导纳模式参数）
    Result SetDt(double dt);

private:
    Result ValidateTorqueCommand(const JntArray& tau_cmd);

    HardwareInterface* hardware_{nullptr};
    ModelInterface* model_{nullptr};
    bool mode_set_{false};
    JntArray last_torque_;
    bool has_last_torque_{false};

    JntArray K_p_;           // 刚度系数 [Nm/rad]
    JntArray K_d_;           // 阻尼系数 [Nm·s/rad]

    // 导纳模式参数
    JntArray M_;             // 虚拟惯量 [kg·m²]
    JntArray tau_offset_;    // 关节力矩零飘 [Nm]
    double dt_{0.001};       // 控制周期 [s]

    static constexpr double kDefaultStiffness = 500.0;   // Nm/rad
    static constexpr double kDefaultDamping  = 20.0;     // Nm·s/rad
    static constexpr double kDefaultInertia  = 1.0;      // kg·m²
    static constexpr int8_t CST_MODE = 10;               // Cyclic Synchronous Torque
};

} // namespace rocos
