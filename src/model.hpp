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

#include <memory>

#include "logger.hpp"
#include "mimic_joint.hpp"
#include "model_interface.hpp"

namespace rocos {

class Model : public ModelInterface {
 public:
  explicit Model(const std::string& urdf_file_path,
                 const std::string& base_link, const std::string& tip,
                 const std::vector<std::string>& driven_joint_names = {});

  ~Model() override;

  // 复制配置并创建独立求解器，不共享实时线程的可变计算状态。
  std::unique_ptr<Model> CloneForPreview() const;

  Result ForwardKinematics(const JntArray& q_in, Frame& p_out) override;
  Result InverseKinematics(const JntArray& q_in, const Frame& p_in,
                           JntArray& q_out) override;
  Result ForwardDynamics(const JntArray& q, const JntArray& q_dot,
                         const JntArray& torques, const Wrenches& f_ext,
                         JntArray& q_dotdot) override;
  Result InverseDynamics(const JntArray& q, const JntArray& q_dot,
                         const JntArray& q_dotdot, const Wrenches& f_ext,
                         JntArray& torques) override;

  Result GetJacobian(const JntArray& q, Jacobian& J_out) override;

  int GetJointNum() const override;
  std::vector<std::string> GetJointNames() const override;
  Result ApplyPositionCoupling(JntArray& q) const override;
  const std::vector<MimicJoint>& GetMimicJoints() const {
    return mimic_joints_;
  }

  void SetGravity(const Vector& gravity);

  // 默认取 URDF 标准限位；配置变更仅允许初始化或停止计算后调用。
  void SetPosLimits(const JntArray& q_min, const JntArray& q_max);

  void UpdateSolvers();

  // 不允许与求解或限位读取并发；失败时保留原模型。
  bool ParseUrdf(const std::string& urdf_string,
                 const std::string& base_link,
                 const std::string& tip,
                 const std::vector<std::string>& driven_joint_names = {});

  const JntArray& GetPosLowerLimit() const override { return q_min_; }
  const JntArray& GetPosUpperLimit() const override { return q_max_; }
  const JntArray& GetVelocityLimit() const override { return q_vel_; }
  const JntArray& GetEffortLimit() const override { return q_effort_; }

 private:
  Model();
  struct Impl;
  std::unique_ptr<Impl> impl_;

  std::vector<std::string> joint_names_;  // 模型链中可动关节名称，顺序与 JntArray 一致
  std::vector<MimicJoint> mimic_joints_;

  std::string urdf_string_;

  JntArray q_min_;
  JntArray q_max_;

  JntArray q_vel_;
  JntArray q_effort_;

  Vector gravity_{0.0, 0.0, -9.81};

  Logger::logger_ptr log_ptr_ = nullptr;
};

}  // namespace rocos
