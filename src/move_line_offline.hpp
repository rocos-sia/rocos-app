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

#include "motion_interface.hpp"
#include "model_interface.hpp"
#include "unit_velocity_profile.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <vector>
#include <chrono>

namespace rocos {

/// @brief 笛卡尔直线运动 (MoveL) — 离线规划 + 全轨迹 IK 验证
///
/// 与 MoveLine（在线规划）的核心区别：
///   1. Reset() 中一次性跑完整条 S 曲线，生成全部笛卡尔路径点
///   2. 逐点逆运动学验证：任一点 IK 失败 → PlanError → 回到 Stopped
///   3. 关节空间轨迹存储，正常执行时按索引回放
///   4. 暂停/继续走 Jacobian 积分通道：
///        s_dot → 笛卡尔速度 (Twist) → J⁺ · Twist → 关节速度 → 积分
///      因为全轨迹 IK 已验证，Jacobian 沿路径非奇异，J⁺ 始终存在
///
/// 典型用法：
///   MoveLineOffline move(pose_start, pose_goal, model);
///   move.SetInitialJointPosition(q_current);  // IK 暖启动
///   Result r = move.Reset();                   // 离线规划 + 逐点 IK
///   if (r == Result::NoError) {
///       while (true) {
///           r = move.Update();
///           if (r == Result::PlanFinished) break;
///           if (r < Result::NoError) { /* error */ }
///           Reference ref;
///           move.GenerateRef(ref);             // JntArray
///       }
///   }
class MoveLineOffline : public MotionInterface {
public:
    /// @brief 构造并配置全部运动参数
    /// @param pose_start  起始笛卡尔位姿
    /// @param pose_goal   目标笛卡尔位姿
    /// @param model       机器人模型（必需，提供 IK / Jacobian）
    /// @param v_limit     速度限制（一维标量 m/s），默认 1.0
    /// @param a_limit     加速度限制（一维标量 m/s²），默认 2.0
    /// @param j_limit     jerk 限制（一维标量 m/s³），默认 10.0
    /// @param dt          控制周期 [秒]，默认 0.001
    explicit MoveLineOffline(const Frame& pose_start,
                             const Frame& pose_goal,
                             ModelInterface* model,
                             double v_limit = 1.0,
                             double a_limit = 2.0,
                             double j_limit = 10.0,
                             double dt = 0.001);
    ~MoveLineOffline() override;

    /// @brief 设置 IK 暖启动初始关节位置（Reset 前必须调用）
    void SetInitialJointPosition(const JntArray& q);

    // 仅预览任务设置预算；实机默认不改变既有规划行为。
    void SetPlanningBudget(size_t max_steps, std::chrono::steady_clock::time_point deadline) {
        max_planning_steps_ = max_steps;
        planning_deadline_ = deadline;
    }

    // ─── MotionInterface 接口 ───

    Result ValidateParameters() const override;
    Result Reset() override;
    Result GenerateRef(Reference& ref_out) override;

    bool CanPause()  const override { return true; }
    bool CanResume() const override { return true; }
    bool CanStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;

private:
    /// @brief 计算 path_length 和归一化限制
    bool computeNormalizedLimits();

    /// @brief s ∈ [0,1] 处插值笛卡尔位姿（平移线性 + 旋转 Slerp）
    Frame interpolatePose(double s) const;

    /// @brief s_dot → 6 维笛卡尔速度 Twist（平移速度 + 旋转角速度）
    Twist computeCartesianTwist(double s_dot) const;

    /// @brief J(q)⁺ · twist → q_dot，Eigen SVD 伪逆求解
    bool solveJointVelocity(const JntArray& q,
                            const Twist& twist,
                            JntArray& q_dot_out) const;

    /// @brief 从轨迹回放模式切换到 Jacobian 积分模式，初始化 profile 当前状态
    Result switchToIntegrateMode();

    // ─── 执行模式 ───
    enum class Mode { Normal, Integrate };

    size_t max_planning_steps_{0};
    std::chrono::steady_clock::time_point planning_deadline_{std::chrono::steady_clock::time_point::max()};
    double dt_;
    UnitVelocityProfile profile_;
    ModelInterface* model_;  // 必需，不拥有

    Frame pose_start_;
    Frame pose_goal_;
    Eigen::Quaterniond q_start_;  // 构造时缓存
    Eigen::Quaterniond q_end_;

    // 旋转角-轴缓存（构造时从 q_start→q_end 算出，pause 时算 ω）
    Eigen::Vector3d rot_axis_{0.0, 0.0, 1.0};
    double total_angle_{0.0};

    double path_length_{0.0};

    double v_limit_{0.0};
    double a_limit_{0.0};
    double j_limit_{0.0};

    double norm_v_{0.0};
    double norm_a_{0.0};
    double norm_j_{0.0};

    static constexpr double kEquivalentRadius = 0.1;
    static constexpr double kPathEpsilon      = 1e-7;

    // ─── 离线轨迹 ───
    struct TrajectoryPoint {
        double s;
        double s_dot;
        JntArray q;
    };
    std::vector<TrajectoryPoint> trajectory_;
    size_t index_{0};

    // ─── IK 暖启动 ───
    JntArray q_init_;

    // ─── 积分模式状态 ───
    Mode mode_{Mode::Normal};
    JntArray q_current_;  // 当前积分关节位置
    int n_joints_{0};

    bool has_motion_{false};
};

}  // namespace rocos
