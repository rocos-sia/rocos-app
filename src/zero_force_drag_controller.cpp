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

#include "zero_force_drag_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <variant>

namespace rocos {

// ==========================================================================
// 析构 — 安全回退到 CSP 模式
// ==========================================================================

ZeroForceDragController::~ZeroForceDragController() {
    if (hardware_ != nullptr) {
        hardware_->WaitForSignal();
        hardware_->SetPosition(hardware_->GetPosition());
        hardware_->SetMode(CSP_MODE);
    }
}

// ==========================================================================
// Reset — 清除全部内部状态
// ==========================================================================

bool ZeroForceDragController::Reset() {
    mode_set_ = false;
    state_ = State::INIT;
    pose_index_ = 0;
    settle_counter_ = 0;
    measure_counter_ = 0;
    measurements_.clear();
    tau_prev_valid_ = false;
    hold_position_valid_ = false;
    holding_ = false;

    // 预置负载参数是"配置"而非运行时状态，Reset 应保留；
    // 仅当未预置时才清除辨识结果。
    if (!load_params_preset_) {
        identified_ = false;
        load_mass_ = 0.0;
        load_com_ = rocos::Vector::Zero();
    }
    return true;
}

// ==========================================================================
// SetHardware / SetModel — 存储裸指针
// ==========================================================================

Result ZeroForceDragController::SetHardware(HardwareInterface* hardware) {
    if (hardware == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_ = hardware;
    return Result::NoError;
}

Result ZeroForceDragController::SetModel(ModelInterface* model) {
    if (model == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    model_ = model;
    return Result::NoError;
}

// ==========================================================================
// SetReady — 初始化硬件，生成默认辨识位形，进入 CST 模式
// ==========================================================================

Result ZeroForceDragController::SetReady() {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    hardware_->WaitForSignal();

    const JntArray q_act = hardware_->GetPosition();
    n_joints_ = static_cast<int>(q_act.rows());
    if (n_joints_ == 0) {
        return Result::JointStateError;
    }

    // 控制周期（微秒 → 秒）
    dt_ = static_cast<double>(hardware_->GetDt()) / 1'000'000.0;

    // // 若未从外部设置辨识位形，则从当前位形自动生成默认位形
    // if (id_poses_.empty()) {
    //     // 默认: 从当前位置出发施加关节偏移以激发可辨识性
    //     // 偏移量针对通用 6 轴机械臂设计；关节数少于 6 时多余列被忽略
    //     std::vector<std::vector<double>> offsets = {
    //         { 0.0,   0.0,   0.0,   0.0,   0.0,   0.0 },   // 位形0: 当前位形（基线）
    //         { 0.3,  -0.2,   0.25,  0.5,  -0.35,  0.4 },   // 位形1: 肩+肘+腕变化
    //         {-0.25,  0.3,  -0.2,  -0.4,   0.5,  -0.35},   // 位形2: 不同方向
    //         { 0.15, -0.3,   0.1,   0.6,   0.0,   0.45},   // 位形3: 侧重腕部
    //         {-0.35,  0.1,  -0.3,  -0.5,   0.3,   0.0 },   // 位形4: 侧重肘部
    //     };

    //     for (const auto& offset : offsets) {
    //         JntArray pose(n_joints_);
    //         for (int j = 0; j < n_joints_; ++j) {
    //             double off = (j < static_cast<int>(offset.size())) ? offset[j] : 0.0;
    //             pose(j) = q_act(j) + off;
    //         }
    //         id_poses_.push_back(pose);
    //     }
    // }

    // 初始化 tau_accum_ 和 tau_prev_
    tau_accum_.resize(n_joints_);
    for (int i = 0; i < n_joints_; ++i) tau_accum_(i) = 0.0;

    tau_prev_.resize(n_joints_);

    // 计算初始重力补偿力矩
    JntArray tau_grav(n_joints_);
    Result res = computeGravityTorque(q_act, tau_grav);
    if (res != Result::NoError) return res;

    tau_prev_ = tau_grav;
    tau_prev_valid_ = true;

    // 下发重力补偿力矩，切换到 CST 模式
    hardware_->SetTorque(tau_grav);
    hardware_->SetMode(CST_MODE);
    mode_set_ = true;

    // 初始化状态机
    pose_index_ = 0;
    settle_counter_ = 0;
    measure_counter_ = 0;
    measurements_.clear();

    // 若已预置负载参数，跳过辨识直接进入拖动；否则走辨识流程
    if (load_params_preset_) {
        identified_ = true;
        state_ = State::DRAGGING;
    } else {
        identified_ = false;
        state_ = State::INIT;
    }

    return Result::NoError;
}

// ==========================================================================
// GenerateCmd — 辨识/拖动阶段由控制器自主决策，不使用外部参考
// ==========================================================================

Result ZeroForceDragController::GenerateCmd(const Reference& ref_in,
                                             JntArray& q_cmd) {
    if (auto* q_ref = std::get_if<JntArray>(&ref_in)) {
        if (q_ref->rows() == 0) return Result::MoveInput;
        q_cmd = *q_ref;
        return Result::NoError;
    }
    return Result::MoveUnknown;
}

// ==========================================================================
// UpdateCmd — 主状态机
//
//   流程: INIT → MOVE_TO_POSE ⇄ SETTLE → MEASURE → (重复)
//         全部位形采集完毕 → COMPUTE → DRAGGING
//         辨识失败 → IDENT_FAILED（保持位置+重力补偿）
// ==========================================================================

Result ZeroForceDragController::UpdateCmd(const JntArray& q_cmd) {
    (void)q_cmd;  // 辨识/拖动自主决策

    if (hardware_ == nullptr || model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const int n = n_joints_;
    JntArray q_act = hardware_->GetPosition();
    JntArray q_dot_act = hardware_->GetVelocity();
    if (q_act.rows() != static_cast<unsigned int>(n)) {
        return Result::JointStateError;
    }
    const bool has_vel = (q_dot_act.rows() == static_cast<unsigned int>(n));

    // 确保 CST 模式
    if (!mode_set_) {
        hardware_->SetMode(CST_MODE);
        mode_set_ = true;
    }

    // ---- 状态机 ----
    switch (state_) {

    // ======================================================================
    // INIT — 首个周期：校验位形列表 + 下发重力补偿力矩（不依赖 fall-through）
    // ======================================================================
    case State::INIT: {
        if (id_poses_.empty()) {
            state_ = State::IDENT_FAILED;
            return Result::LoadIdentInitFail;
        }

        // 下发纯重力补偿力矩，保证 CST 模式下每个周期都有力矩输出
        JntArray tau_grav(n);
        Result res = computeGravityTorque(q_act, tau_grav);
        if (res != Result::NoError) return res;

        res = applyTorqueWithLimits(tau_grav);
        if (res != Result::NoError) return res;

        pose_index_ = 0;
        state_ = State::MOVE_TO_POSE;
        break;  // 本周期仅做初始化，下周期进入 MOVE_TO_POSE
    }

    // ======================================================================
    // MOVE_TO_POSE — PD 控制到达辨识位形
    // ======================================================================
    case State::MOVE_TO_POSE: {
        if (pose_index_ >= static_cast<int>(id_poses_.size())) {
            state_ = State::COMPUTE;
            break;
        }

        const JntArray& q_target = id_poses_[pose_index_];

        // PD 控制力矩
        JntArray tau_pd = computePDTorque(q_act, q_dot_act, q_target, has_vel);

        // 重力补偿
        JntArray tau_grav(n);
        Result res = computeGravityTorque(q_act, tau_grav);
        if (res != Result::NoError) return res;

        // 合成 + 下发
        JntArray tau(n);
        for (int i = 0; i < n; ++i) tau(i) = tau_pd(i) + tau_grav(i);
        res = applyTorqueWithLimits(tau);
        if (res != Result::NoError) return res;

        // 到位判定：所有关节误差 < 0.03 rad (≈1.7°)
        double max_err = 0.0;
        for (int i = 0; i < n; ++i) {
            double err = std::abs(q_target(i) - q_act(i));
            if (err > max_err) max_err = err;
        }
        if (max_err < 0.03) {
            settle_counter_ = 0;
            state_ = State::SETTLE;
        }
        break;
    }

    // ======================================================================
    // SETTLE — 到位后保持，等待振动衰减
    // ======================================================================
    case State::SETTLE: {
        const JntArray& q_target = id_poses_[pose_index_];

        JntArray tau_pd = computePDTorque(q_act, q_dot_act, q_target, has_vel);
        JntArray tau_grav(n);
        Result res = computeGravityTorque(q_act, tau_grav);
        if (res != Result::NoError) return res;

        JntArray tau(n);
        for (int i = 0; i < n; ++i) tau(i) = tau_pd(i) + tau_grav(i);
        res = applyTorqueWithLimits(tau);
        if (res != Result::NoError) return res;

        ++settle_counter_;
        if (settle_counter_ >= settle_cycles_) {
            measure_counter_ = 0;
            for (int i = 0; i < n; ++i) tau_accum_(i) = 0.0;
            state_ = State::MEASURE;
        }
        break;
    }

    // ======================================================================
    // MEASURE — 多帧采样并取平均，得到 τ_load = τ_meas - τ_grav
    // ======================================================================
    case State::MEASURE: {
        const JntArray& q_target = id_poses_[pose_index_];

        // 保持 PD 控制
        JntArray tau_pd = computePDTorque(q_act, q_dot_act, q_target, has_vel);
        JntArray tau_grav(n);
        Result res = computeGravityTorque(q_act, tau_grav);
        if (res != Result::NoError) return res;

        JntArray tau_hold(n);
        for (int i = 0; i < n; ++i) tau_hold(i) = tau_pd(i) + tau_grav(i);
        res = applyTorqueWithLimits(tau_hold);
        if (res != Result::NoError) return res;

        // 读取实测力矩 → 减去重力补偿 → 累加
        JntArray tau_meas = hardware_->GetTorque();
        if (tau_meas.rows() != static_cast<unsigned int>(n)) {
            return Result::LoadIdentUfbFail;
        }
        for (int i = 0; i < n; ++i) {
            tau_accum_(i) += (tau_meas(i) - tau_grav(i));
        }

        ++measure_counter_;
        if (measure_counter_ >= measure_cycles_) {
            // 取平均保存
            MeasurementData data;
            data.q.resize(n);
            data.tau_load.resize(n);
            for (int i = 0; i < n; ++i) {
                data.q(i) = q_act(i);
                data.tau_load(i) = tau_accum_(i) / static_cast<double>(measure_cycles_);
            }
            measurements_.push_back(data);

            ++pose_index_;
            if (pose_index_ >= static_cast<int>(id_poses_.size())) {
                state_ = State::COMPUTE;
            } else {
                state_ = State::MOVE_TO_POSE;
            }
        }
        break;
    }

    // ======================================================================
    // COMPUTE — 最小二乘求解负载参数
    // ======================================================================
    case State::COMPUTE: {
        Result res = computeLoadParams();
        if (res != Result::NoError) {
            state_ = State::IDENT_FAILED;
            return res;
        }
        identified_ = true;
        state_ = State::DRAGGING;

        // 通知 Robot 层辨识结果（用于回写成员 + 落盘配置文件）
        if (load_params_cb_) {
            load_params_cb_(load_mass_, load_com_);
        }
        break;
    }

    // ======================================================================
    // DRAGGING — 零力拖动 + 低速保持
    //   拖动中:  τ_cmd = τ_grav_total - Kd_drag·q̇   （小阻尼，轻便）
    //   低速时:  τ_cmd = τ_grav_total - Kd_hold·q̇   （大阻尼，阻止漂移）
    //   低速时用大阻尼"粘"住关节，只抵抗速度、不产生回位力，因此松手
    //   停在原地不会回弹；发力拖动速度超阈值即切回小阻尼恢复正常拖动。
    // ======================================================================
    case State::DRAGGING: {
        JntArray tau_total(n);
        Result res = computeTotalGravityCompensation(q_act, tau_total);
        if (res != Result::NoError) return res;

        if (has_vel) {
            // 最大关节速度：判断拖动 vs 静止
            double max_speed = 0.0;
            for (int i = 0; i < n; ++i) {
                max_speed = std::max(max_speed, std::abs(q_dot_act(i)));
            }

            // 带滞回切换低速保持状态，避免阈值附近抖动
            if (!holding_) {
                if (max_speed < v_hold_low_) holding_ = true;   // 松手 → 进入低速
            } else {
                if (max_speed > v_hold_high_) holding_ = false; // 发力 → 离开低速
            }

            // 低速用大阻尼，拖动用小阻尼（纯阻尼，无回位力）
            const double Kd = holding_ ? Kd_hold_ : Kd_drag_;
            for (int i = 0; i < n; ++i) {
                tau_total(i) -= Kd * q_dot_act(i);
            }
        }

        res = applyTorqueWithLimits(tau_total);
        if (res != Result::NoError) return res;
        break;
    }

    // ======================================================================
    // IDENT_FAILED — 辨识失败时保持位置 + 机器人自重补偿
    // ======================================================================
    case State::IDENT_FAILED: {
        JntArray tau_grav(n);
        Result res = computeGravityTorque(q_act, tau_grav);
        if (res != Result::NoError) return res;

        // 记录初始位置用于保持
        if (!hold_position_valid_) {
            hold_position_.resize(n);
            hold_position_ = q_act;
            hold_position_valid_ = true;
        }

        // 添加柔顺位置保持（低刚度防止漂移）
        for (int i = 0; i < n; ++i) {
            tau_grav(i) += 80.0 * (hold_position_(i) - q_act(i));
        }

        res = applyTorqueWithLimits(tau_grav);
        if (res != Result::NoError) return res;
        break;
    }

    } // switch

    return Result::NoError;
}

// ==========================================================================
// 内部辅助函数
// ==========================================================================

JntArray ZeroForceDragController::computePDTorque(
    const JntArray& q_act, const JntArray& q_dot_act,
    const JntArray& q_target, bool has_velocity) {

    const int n = n_joints_;
    JntArray tau(n);
    for (int i = 0; i < n; ++i) {
        double perr = q_target(i) - q_act(i);
        tau(i) = Kp_joint_ * perr;
        if (has_velocity) {
            tau(i) -= Kd_joint_ * q_dot_act(i);
        }
    }
    return tau;
}

Result ZeroForceDragController::computeGravityTorque(
    const JntArray& q_act, JntArray& tau_grav) {

    const int n = n_joints_;
    tau_grav.resize(n);
    JntArray zero_vel(n);
    JntArray zero_acc(n);
    for (int i = 0; i < n; ++i) {
        zero_vel(i) = 0.0;
        zero_acc(i) = 0.0;
    }
    Wrenches f_ext;  // 空 → 模型补零外力旋量
    return model_->InverseDynamics(q_act, zero_vel, zero_acc, f_ext, tau_grav);
}

Result ZeroForceDragController::applyTorqueWithLimits(const JntArray& tau_raw) {
    const int n = n_joints_;
    JntArray tau = tau_raw;

    // 力矩变化率限制
    if (tau_rate_limit_ > 0.0 && tau_prev_valid_ &&
        tau_prev_.rows() == static_cast<unsigned int>(n)) {
        const double max_delta = tau_rate_limit_ * dt_;
        for (int i = 0; i < n; ++i) {
            double delta = tau(i) - tau_prev_(i);
            delta = std::max(-max_delta, std::min(max_delta, delta));
            tau(i) = tau_prev_(i) + delta;
        }
    }

    // 力矩饱和
    if (tau_max_ > 0.0) {
        for (int i = 0; i < n; ++i) {
            tau(i) = std::max(-tau_max_, std::min(tau_max_, tau(i)));
        }
    }

    tau_prev_ = tau;
    tau_prev_valid_ = true;

    hardware_->SetTorque(tau);
    return Result::NoError;
}

// ==========================================================================
// computeTotalGravityCompensation — 机器自重 + 负载重力
//
//   负载在基坐标系下的末端力旋量:
//     F_base = m_load · g_base
//     M_base = (R_ee · r_com) × (m_load · g_base)
//   关节力矩: τ_load = J^T · W_base
// ==========================================================================

Result ZeroForceDragController::computeTotalGravityCompensation(
    const JntArray& q_act, JntArray& tau_total) {

    const int n = n_joints_;

    // 1. 机器人自重重力补偿
    JntArray tau_robot(n);
    Result res = computeGravityTorque(q_act, tau_robot);
    if (res != Result::NoError) return res;

    // 2. 无负载参数时仅补偿机器人自重
    if (!identified_ || load_mass_ <= 0.0) {
        tau_total = tau_robot;
        return Result::NoError;
    }

    // 3. FK 获取末端姿态
    Frame x_ee;
    res = model_->ForwardKinematics(q_act, x_ee);
    if (res != Result::NoError) return res;

    const rocos::Rotation& R_ee = x_ee.M;

    // 4. Jacobian
    Jacobian J_out;
    res = model_->GetJacobian(q_act, J_out);
    if (res != Result::NoError) return res;

    // 5. 负载质心变换到基坐标系
    rocos::Vector g_vec = gravityVector();
    rocos::Vector r_com_base = R_ee * load_com_;

    // 6. 负载力旋量（基坐标系）
    rocos::Vector F_load = load_mass_ * g_vec;
    rocos::Vector M_load = r_com_base * F_load;  // KDL 叉积

    // 7. 构建 6×1 列向量
    Eigen::Matrix<double, 6, 1> W_vec;
    W_vec(0) = F_load.x();
    W_vec(1) = F_load.y();
    W_vec(2) = F_load.z();
    W_vec(3) = M_load.x();
    W_vec(4) = M_load.y();
    W_vec(5) = M_load.z();

    // 8. τ_load = J^T · W （负载重力对应的广义关节力矩）
    Eigen::VectorXd tau_load_eig = J_out.data.transpose() * W_vec;

    // 9. 合成补偿力矩
    //    KDL 约定: InverseDynamics(q,0,0,f_ext) = G(q) − J^T·f_ext
    //    负载重力是末端外力 f_ext，故补偿为 G(q) − J^T·W_load（减号）
    tau_total.resize(n);
    for (int i = 0; i < n; ++i) {
        tau_total(i) = tau_robot(i) - tau_load_eig(i);
    }

    return Result::NoError;
}

// ==========================================================================
// computeLoadParams — 最小二乘求解 p = [m, m·cx, m·cy, m·cz]^T
//
//   回归模型: τ_load_i = J_i^T · A_i · p
//   其中 A_i 将负载参数映射为基坐标系末端力旋量: W_base = A_i(q) · p
//
//   A_i 推导（重力 g_base = [gx, gy, gz], EE 旋转矩阵 R）:
//     力:  F = m · g
//     力矩: M = (R·r_com) × (m·g)
//   展开 M 分量得到 A_i 的非零结构。
// ==========================================================================

Result ZeroForceDragController::computeLoadParams() {
    const int n = n_joints_;
    const int M = static_cast<int>(measurements_.size());

    if (M < 2) {
        return Result::LoadIdentFitFail;  // 至少 2 组数据
    }

    rocos::Vector g_vec = gravityVector();
    const double gx = g_vec.x();
    const double gy = g_vec.y();
    const double gz = g_vec.z();

    // 回归矩阵 Y (M·n × 4), 观测向量 b (M·n)
    const int rows = M * n;
    Eigen::MatrixXd Y(rows, 4);
    Eigen::VectorXd b(rows);
    Y.setZero();

    for (int k = 0; k < M; ++k) {
        const JntArray& q_k = measurements_[k].q;
        const JntArray& tau_load_k = measurements_[k].tau_load;

        // FK → 末端姿态 R
        Frame x_ee;
        Result res = model_->ForwardKinematics(q_k, x_ee);
        if (res != Result::NoError) return res;
        const rocos::Rotation& R = x_ee.M;

        // Jacobian
        Jacobian J_out;
        res = model_->GetJacobian(q_k, J_out);
        if (res != Result::NoError) return res;

        // 构建 6×4 的 A 矩阵
        // A 将 p = [m, m·cx, m·cy, m·cz] 映射为 W_base = [F; M]
        //
        // F = m · g  →  F_x = m·gx, F_y = m·gy, F_z = m·gz
        //
        // M = (R·r) × (m·g), 令 R·r = [a,b,c]^T
        // M_x = b·m·gz - c·m·gy
        //     = (R10·cx+R11·cy+R12·cz)·m·gz - (R20·cx+R21·cy+R22·cz)·m·gy
        //     = p2·(R10·gz-R20·gy) + p3·(R11·gz-R21·gy) + p4·(R12·gz-R22·gy)
        // M_y = p2·(R20·gx-R00·gz) + p3·(R21·gx-R01·gz) + p4·(R22·gx-R02·gz)
        // M_z = p2·(R00·gy-R10·gx) + p3·(R01·gy-R11·gx) + p4·(R02·gy-R12·gx)

        Eigen::Matrix<double, 6, 4> A;
        A.setZero();

        // 力分量（仅质量贡献）
        A(0, 0) = gx;
        A(1, 0) = gy;
        A(2, 0) = gz;

        // 力矩分量（质量×质心贡献）
        const double R00 = R(0, 0), R01 = R(0, 1), R02 = R(0, 2);
        const double R10 = R(1, 0), R11 = R(1, 1), R12 = R(1, 2);
        const double R20 = R(2, 0), R21 = R(2, 1), R22 = R(2, 2);

        // M_x 行: p2·(R10·gz-R20·gy) + p3·(R11·gz-R21·gy) + p4·(R12·gz-R22·gy)
        A(3, 1) = R10 * gz - R20 * gy;
        A(3, 2) = R11 * gz - R21 * gy;
        A(3, 3) = R12 * gz - R22 * gy;

        // M_y 行: p2·(R20·gx-R00·gz) + p3·(R21·gx-R01·gz) + p4·(R22·gx-R02·gz)
        A(4, 1) = R20 * gx - R00 * gz;
        A(4, 2) = R21 * gx - R01 * gz;
        A(4, 3) = R22 * gx - R02 * gz;

        // M_z 行: p2·(R00·gy-R10·gx) + p3·(R01·gy-R11·gx) + p4·(R02·gy-R12·gx)
        A(5, 1) = R00 * gy - R10 * gx;
        A(5, 2) = R01 * gy - R11 * gx;
        A(5, 3) = R02 * gy - R12 * gx;

        // Y_k = J_k^T · A_k  (n × 4)
        Eigen::MatrixXd Jt = J_out.data.transpose();
        Eigen::MatrixXd Y_k = Jt * A;

        const int base_row = k * n;
        Y.block(base_row, 0, n, 4) = Y_k;

        // 观测 τ_load_measured = τ_measured − τ_robot_grav = −J^T·W_load = −J^T·A·p
        // 回归模型 Y·p = τ_load_measured 等价于 J^T·A·p = τ_load_measured
        // 故需对观测取负: b = −τ_load_measured = J^T·A·p
        for (int i = 0; i < n; ++i) {
            b(base_row + i) = -tau_load_k(i);
        }
    }

    // SVD 最小二乘求解 p = (Y^T·Y)^{-1}·Y^T·b
    Eigen::Vector4d p;
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            Y, Eigen::ComputeThinU | Eigen::ComputeThinV);

        const auto& S = svd.singularValues();
        if (S(0) < 1e-12) {
            return Result::LoadIdentFitFail;
        }
        double cond = S(0) / S(S.size() - 1);
        if (cond > 1e6) {
            return Result::LoadIdentFitFail;  // 病态矩阵，位形不足以激发全部参数
        }

        p = svd.solve(b);
    }

    // 提取参数
    const double p_mass = p(0);
    const double p_mcx  = p(1);
    const double p_mcy  = p(2);
    const double p_mcz  = p(3);

    if (p_mass <= 0.0) {
        return Result::LoadIdentNonLoaded;
    }

    load_mass_ = p_mass;
    load_com_ = rocos::Vector(p_mcx / p_mass, p_mcy / p_mass, p_mcz / p_mass);

    return Result::NoError;
}

// ==========================================================================
// gravityVector — 基坐标系下重力方向
// ==========================================================================

rocos::Vector ZeroForceDragController::gravityVector() const {
    // 默认 Z 向下（与 URDF 中 gravity [0, 0, -9.81] 约定一致）
    return rocos::Vector(0.0, 0.0, -g_);
}

// ==========================================================================
// 参数设置接口
// ==========================================================================

void ZeroForceDragController::SetIdentificationPoses(
    const std::vector<JntArray>& poses) {
    id_poses_ = poses;
}

void ZeroForceDragController::SetSettleCycles(int cycles) {
    if (cycles > 0) settle_cycles_ = cycles;
}

void ZeroForceDragController::SetMeasureCycles(int cycles) {
    if (cycles > 0) measure_cycles_ = cycles;
}

void ZeroForceDragController::SetDamping(double Kd) {
    if (std::isfinite(Kd) && Kd >= 0.0) Kd_drag_ = Kd;
}

void ZeroForceDragController::SetHoldDamping(double Kd) {
    if (std::isfinite(Kd) && Kd >= 0.0) Kd_hold_ = Kd;
}

void ZeroForceDragController::SetHoldThreshold(double v_low, double v_high) {
    if (!std::isfinite(v_low) || !std::isfinite(v_high)) return;
    if (v_low < 0.0 || v_high < v_low) return;
    v_hold_low_ = v_low;
    v_hold_high_ = v_high;
}

void ZeroForceDragController::SetLoadParameters(double mass,
                                                const rocos::Vector& com) {
    // 非法参数直接忽略，交由 SetReady 走辨识流程兜底；mass=0 表示无负载（合法）
    if (mass < 0.0 || !std::isfinite(mass) ||
        !std::isfinite(com.x()) || !std::isfinite(com.y()) ||
        !std::isfinite(com.z())) {
        return;
    }

    load_mass_ = mass;
    load_com_ = com;
    load_params_preset_ = true;
}

void ZeroForceDragController::SetLoadParamsCallback(
    std::function<void(double, const rocos::Vector&)> cb) {
    load_params_cb_ = std::move(cb);
}

// ==========================================================================
// 状态查询
// ==========================================================================

std::string ZeroForceDragController::GetStateString() const {
    switch (state_) {
    case State::INIT:          return "INIT";
    case State::MOVE_TO_POSE:  return "MOVE_TO_POSE";
    case State::SETTLE:        return "SETTLE";
    case State::MEASURE:       return "MEASURE";
    case State::COMPUTE:       return "COMPUTE";
    case State::DRAGGING:      return "DRAGGING";
    case State::IDENT_FAILED:  return "IDENT_FAILED";
    default:                   return "UNKNOWN";
    }
}

double ZeroForceDragController::GetProgress() const {
    if (state_ == State::DRAGGING) return 1.0;
    if (state_ == State::IDENT_FAILED) return 0.0;
    if (id_poses_.empty()) return 0.0;

    const int total = static_cast<int>(id_poses_.size());
    const double measure_weight = 0.8;

    if (state_ == State::COMPUTE) {
        return measure_weight + 0.1;  // 计算中
    }

    double pose_frac = static_cast<double>(pose_index_) / total;
    double stage_frac = 0.0;

    switch (state_) {
    case State::MOVE_TO_POSE:
        stage_frac = 0.0;
        break;
    case State::SETTLE:
        stage_frac = 0.2 / total;
        break;
    case State::MEASURE:
        stage_frac = (0.4 + 0.4 * measure_counter_ / measure_cycles_) / total;
        break;
    default:
        break;
    }

    return measure_weight * (pose_frac + stage_frac);
}

} // namespace rocos
