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

#include "servo_settings.hpp"

#include <string>
#include <vector>

#include "types.hpp"
#include "result.hpp"

namespace rocos {

// 按 base→tip 的子连杆顺序（包括固定连杆）；局部力矩关于各连杆原点。
// 空数组表示无外力。
typedef std::vector<Wrench> Wrenches;


class ModelInterface {
 public:
  virtual ~ModelInterface() = default;

  virtual Result ForwardKinematics(const JntArray& q_in, Frame& p_out) = 0;
  virtual Result InverseKinematics(const JntArray& q_in, const Frame& p_in, JntArray& q_out) = 0;

  virtual Result ForwardDynamics(const JntArray &q, const JntArray &q_dot, const JntArray &torques, const Wrenches& f_ext,JntArray &q_dotdot) = 0;
  virtual Result InverseDynamics(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques) = 0;

  /// @brief tip 原点的几何 Jacobian (6×n)，base 系表达，线速度在前。
  virtual Result GetJacobian(const JntArray& q, Jacobian& J_out) = 0;

  /// @brief 获取 URDF 运动链 中可动关节的数量
  virtual int GetJointNum() const = 0;

  /// @brief 获取 URDF 运动链 中可动关节的名称列表，顺序与 ROCOS 关节向量 一致
  virtual std::vector<std::string> GetJointNames() const = 0;

  // 保留有驱动 mimic 的槽位，但位置由源关节唯一确定。
  virtual Result ApplyPositionCoupling(JntArray& /*q*/) const {
    return Result::NoError;
  }

  virtual const JntArray& GetPosLowerLimit() const {
    static const JntArray empty;
    return empty;
  }

  virtual const JntArray& GetPosUpperLimit() const {
    static const JntArray empty;
    return empty;
  }

  virtual const JntArray& GetVelocityLimit() const {
    static const JntArray empty;
    return empty;
  }

  // 位置/额定速度来自当前模型（URDF 或显式收紧后的限位）。
  // deceleration > 0，velocity_offset >= 0，单位与模型关节一致。
  // 纯计算，不修改模型；任何失败均保留输出。越界位置不能被静默钳位。
  Result GetPositionVelocityLimits(const JntArray& position,
                                   const JntArray& deceleration,
                                   const JntArray& velocity_offset,
                                   JntArray& lower_velocity,
                                   JntArray& upper_velocity) const;

  virtual const JntArray& GetEffortLimit() const {
    static const JntArray empty;
    return empty;
  }

  // 逐周期限速与制动可行性；只依赖模型限位和显式导数配置。
  Result LimitServoJointStep(unsigned int axis, double desired_velocity,
                            const ServoJointState& previous, const JointDerivativeLimits& limits,
                            double dt, ServoJointState& next) const;
  Result LimitServoPositionStep(unsigned int axis, double target,
                               const ServoJointState& previous, const JointDerivativeLimits& limits,
                               double dt, ServoJointState& next) const;
  // reference_next 必须由 reference 按同一离散积分生成；约束投影后仍需验证制动。
  Result LimitServoReferenceStep(unsigned int axis, const ServoJointState& reference,
                                const ServoJointState& reference_next,
                                const ServoJointState& previous, const JointDerivativeLimits& limits,
                                double dt, ServoJointState& next) const;
  Result MakeServoBrake(unsigned int axis, const ServoJointState& state,
                        const JointDerivativeLimits& limits, double dt, ServoBrake& brake) const;


};
}  // namespace rocos
