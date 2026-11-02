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

  virtual const JntArray& GetEffortLimit() const {
    static const JntArray empty;
    return empty;
  }

};
}  // namespace rocos
