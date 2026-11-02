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

#include <Eigen/Dense>
#include <functional>
#include <vector>

namespace rocos {

/// @brief 零力拖动控制器：先辨识末端负载（质量+质心），再进入拖动模式
///
/// 工作流程：
///   阶段1 — 负载辨识：
///     1. 以 PD 控制驱动机械臂到预设的多组辨识位形
///     2. 每到位形后等待稳定，记录关节角 q_i 和关节力矩 τ_i
///     3. 利用重力补偿计算负载引起的关节力矩：τ_load = τ_measured - τ_robot_grav
///     4. 建立回归方程 τ_load = Y(q) · p，其中 p = [m, m·cx, m·cy, m·cz]^T
///     5. 最小二乘求解负载质量和质心
///   阶段2 — 零力拖动：
///     对每个控制周期：
///       1. 计算重力补偿力矩 τ_grav（机器人自重）
///       2. 计算负载重力补偿力矩 τ_load = J^T · W_load(p)
///       3. 下发 τ_cmd = τ_grav + τ_load - Kd · q̇
///     操作者施加微小外力即可拖动机器人
///
/// 辨识和拖动均在 CST 模式下运行。
class ZeroForceDragController : public ControllerInterface {
public:
    ZeroForceDragController() = default;
    ~ZeroForceDragController() override;

    // ---- ControllerInterface 接口 ----
    bool Reset() override;
    Result SetReady() override;
    Result SetHardware(HardwareInterface* hardware) override;
    Result SetModel(ModelInterface* model) override;
    Result GenerateCmd(const Reference& ref_in, JntArray& q_cmd) override;
    Result UpdateCmd(const JntArray& q_cmd) override;

    // ---- 辨识参数 ----
    /// @brief 设置辨识位形列表（关节角 [rad]）
    void SetIdentificationPoses(const std::vector<JntArray>& poses);
    /// @brief 设置每到位形后的稳定等待周期数，默认 500
    void SetSettleCycles(int cycles);
    /// @brief 设置每到位形后的测量采样周期数，默认 200
    void SetMeasureCycles(int cycles);

    // ---- 拖动参数 ----
    /// @brief 设置拖动阻尼系数 [Nm·s/rad]，默认 5.0
    void SetDamping(double Kd);

    // ---- 低速保持参数 ----
    /// @brief 设置低速保持阻尼 [Nm·s/rad]，默认 60.0（远大于拖动阻尼）
    void SetHoldDamping(double Kd);
    /// @brief 设置低速保持的速度阈值（进入/离开）[rad/s]，默认 0.02/0.05
    void SetHoldThreshold(double v_low, double v_high);

    // ---- 负载参数预置 ----
    /// @brief 设置已知负载参数（质量 + 质心），设置后跳过辨识直接进入拖动模式
    /// @param mass 负载质量 [kg]，必须 >= 0（0 表示无负载）
    /// @param com  负载质心（末端坐标系下）[m]
    /// @note 须在 SetReady() 之前调用；调用后 SetReady() 不再执行辨识流程
    void SetLoadParameters(double mass, const rocos::Vector& com);

    // ---- 辨识完成回调 ----
    /// @brief 注册辨识完成回调，辨识成功进入拖动模式时触发
    /// @param cb 回调 (mass, com)，供 Robot 层回写并落盘辨识结果
    void SetLoadParamsCallback(std::function<void(double, const rocos::Vector&)> cb);

    // ---- 辨识结果查询 ----
    /// @brief 获取辨识出的负载质量 [kg]，未辨识时返回 0
    double GetIdentifiedMass() const { return load_mass_; }
    /// @brief 获取辨识出的负载质心（末端坐标系下）[m]
    rocos::Vector GetIdentifiedCom() const { return load_com_; }
    /// @brief 辨识是否已完成
    bool IsIdentified() const { return identified_; }
    /// @brief 是否已进入拖动模式
    bool IsDragging() const { return state_ == State::DRAGGING; }

    // ---- 辨识状态查询（用于 HTTP API 上报进度） ----
    /// @brief 获取当前辨识阶段名称
    std::string GetStateString() const;
    /// @brief 获取辨识进度 [0.0, 1.0]，拖动模式下返回 1.0
    double GetProgress() const;

private:
    // ---- 内部状态机 ----
    enum class State {
        INIT,           // 初始化完成，等待开始辨识
        MOVE_TO_POSE,   // 正在移动到目标辨识位形
        SETTLE,         // 到位后等待稳定
        MEASURE,        // 采集力矩数据
        COMPUTE,        // 计算负载参数
        DRAGGING,       // 零力拖动模式
        IDENT_FAILED,   // 辨识失败
    };

    // ---- 内部辅助函数 ----
    /// @brief 计算 PD 控制力矩 τ = Kp·(q_des - q_act) - Kd·q̇
    JntArray computePDTorque(const JntArray& q_act, const JntArray& q_dot_act,
                             const JntArray& q_target, bool has_velocity);
    /// @brief 计算机器人自重重力补偿力矩 τ = InvDyn(q, 0, 0, ∅)
    Result computeGravityTorque(const JntArray& q_act, JntArray& tau_grav);
    /// @brief 对原始力矩施加变化率限制和饱和，然后下发到硬件
    Result applyTorqueWithLimits(const JntArray& tau_raw);
    /// @brief 建立回归矩阵并求解负载参数 p = [m, m·cx, m·cy, m·cz]^T
    Result computeLoadParams();
    /// @brief 计算包含负载的总重力补偿力矩 τ_total = τ_robot + J^T·W_load
    Result computeTotalGravityCompensation(const JntArray& q_act, JntArray& tau_total);
    /// @brief 返回基坐标系下的重力向量 [0, 0, -g]
    rocos::Vector gravityVector() const;

    // ---- 成员变量 ----
    HardwareInterface* hardware_{nullptr};
    ModelInterface* model_{nullptr};
    bool mode_set_{false};

    // 状态机
    State state_{State::INIT};
    int pose_index_{0};           // 当前辨识位形索引
    int settle_counter_{0};       // 稳定等待计数
    int measure_counter_{0};      // 测量采样计数
    int n_joints_{0};             // 关节数

    // 辨识位形
    std::vector<JntArray> id_poses_;  // 辨识位形列表
    int settle_cycles_{500};          // 稳定等待周期数
    int measure_cycles_{200};         // 测量采样周期数

    // 辨识数据缓冲区
    struct MeasurementData {
        JntArray q;         // 关节角
        JntArray tau_load;  // 负载引起的关节力矩
    };
    std::vector<MeasurementData> measurements_;

    // 力矩累加器（MEASURE 阶段使用）
    JntArray tau_accum_;

    // 辨识失败时保持的位置
    JntArray hold_position_;
    bool hold_position_valid_{false};

    // 辨识结果
    double load_mass_{0.0};              // 负载质量 [kg]
    rocos::Vector load_com_;               // 负载质心（末端坐标系下）[m]
    bool identified_{false};             // 辨识是否完成
    bool load_params_preset_{false};     // 是否已预置负载参数（预置则跳过辨识）

    // 辨识完成回调（Robot 层用于回写 + 落盘）
    std::function<void(double, const rocos::Vector&)> load_params_cb_;

    // 拖动参数
    double Kd_drag_{5.0};                // 拖动阻尼 [Nm·s/rad]

    // 低速保持参数（低速时增大阻尼阻止空载漂移；纯阻尼无回位力，不会回弹）
    bool holding_{false};                // 是否处于低速保持状态
    double v_hold_low_{0.02};            // 进入低速保持的速度阈值 [rad/s]
    double v_hold_high_{0.05};           // 离开低速保持的速度阈值 [rad/s]（滞回）
    double Kd_hold_{60.0};               // 低速保持阻尼 [Nm·s/rad]（远大于拖动阻尼）

    // PD 控制参数（用于驱动到辨识位形）
    double Kp_joint_{300.0};             // 关节比例增益 [Nm/rad]
    double Kd_joint_{15.0};              // 关节阻尼增益 [Nm·s/rad]

    // 力矩安全
    double tau_rate_limit_{300.0};       // 力矩变化率限制 [Nm/s]
    double tau_max_{0.0};                // 关节力矩饱和值 [Nm]，0=不限制
    JntArray tau_prev_;                  // 上一周期输出的力矩
    bool tau_prev_valid_{false};
    double dt_{0.001};                   // 控制周期 [s]

    // 重力加速度 [m/s²]
    double g_{9.81};

    static constexpr int8_t CST_MODE = 10;  // Cyclic Synchronous Torque
    static constexpr int8_t CSP_MODE = 8;   // Cyclic Synchronous Position
};

} // namespace rocos
