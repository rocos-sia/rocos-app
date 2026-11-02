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

#include <cstdint>
#include <unordered_map>
#include <string>
#include <vector>

#include "result.hpp"

namespace rocos {

/// @brief 管理 URDF model joint ↔ hardware drive_id 的绑定关系
///
/// 支持两种模式：
/// - 默认顺序绑定：无 YAML 时，model joint 按顺序对应 hardware drive
/// - 显式绑定：通过 joint_binding.yaml 指定每个 model joint 对应的 drive_id
class JointBinding {
 public:
  JointBinding() = default;
  ~JointBinding() = default;

  /// @brief 配置绑定上下文：设置已知的 model joint 名称与 hardware drive id 列表
  /// @param model_joint_names   URDF 中可动关节名称（顺序与 ROCOS JntArray 一致）
  /// @param hardware_drive_ids  硬件中所有伺服驱动器的 id 列表
  /// @return NoError / IllegalParameter
  Result Configure(const std::vector<std::string>& model_joint_names,
                   const std::vector<int32_t>& hardware_drive_ids);

  /// @brief 从 YAML 文件加载显式绑定
  Result LoadFromYaml(const std::string& path);

  /// @brief 将当前绑定保存到 YAML 文件
  Result SaveToYaml(const std::string& path) const;

  /// @brief 绑定一个 model joint 到指定 drive_id
  Result Bind(const std::string& joint_name, int32_t drive_id);

  /// @brief 取消某个 model joint 的绑定
  Result UnbindJoint(const std::string& joint_name);

  /// @brief 清空所有绑定
  void Clear();

  /// @brief 校验绑定完整性（每个 model joint 都有绑定、无重复、drive_id 存在）
  Result Validate() const;

  /// @brief 生成 model_index → drive_id 的映射表，供 Hardware 使用
  std::vector<int32_t> GetModelIndexToDriveIds() const;

  /// @brief 按 model_index 查询对应的 drive_id，未绑定返回 -1
  int32_t DriveIdByModelIndex(size_t model_index) const;

  /// @brief 查询某个 joint name 是否已有绑定
  bool HasBindingForJoint(const std::string& joint_name) const;

  /// @brief 是否所有 model joint 都有绑定
  bool IsComplete() const;

 private:
  std::vector<std::string> model_joint_names_;
  std::vector<int32_t> hardware_drive_ids_;
  std::unordered_map<std::string, int32_t> joint_to_drive_id_;
};

}  // namespace rocos
