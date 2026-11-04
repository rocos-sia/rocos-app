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
#include <vector>

#include "drive_interface.hpp"
#include "ft_sensor_interface.hpp"
#include "io_inteface.hpp"
#include "result.hpp"

namespace rocos {

class HardwareInterface : public DriveInterface,
                          public FTSensorInterface,
                          public IOInteface {
 public:
  virtual ~HardwareInterface() = default;

  virtual bool Reset() = 0;

  /// @brief 清除硬件/驱动报警。实现层应清除 Fault/Alarm，但不直接进入运动使能态。
  virtual Result ClearFault() {
    return Result::FunctionNotSupported;
  }

  virtual void WaitForSignal() = 0;

  /// @brief Control cycle period from shared memory, unit: microseconds.
  ///        1 kHz -> 1000. Master (mujoco/ecm) writes EcatBus::dt; app reads it.
  virtual uint32_t GetDt() const {
    return 1000;
  };

  // 位置控制器在限位/速度检查前应用硬件联动约束。
  virtual Result ApplyPositionCoupling(JntArray& /*q*/) const {
    return Result::NoError;
  }

  /// @brief 设置 model index → drive id 映射表（建模层到硬件层的轴绑定）
  virtual Result SetJointBinding(const std::vector<int32_t>& /*model_index_to_drive_id*/) {
    return Result::FunctionNotSupported;
  }

  /// @brief 清除当前的轴绑定映射表
  virtual void ClearJointBinding() {}

  /// @brief 获取当前的 model index → drive id 映射表
  virtual std::vector<int32_t> GetJointBinding() const {
    return {};
  }

  /// @brief 获取真实的硬件伺服驱动器数量（不含绑定映射）
  virtual int GetDriveNum() const {
    return 0;
  }

  /// @brief 获取所有硬件伺服驱动器的 id 列表
  virtual std::vector<int32_t> GetDriveIds() const {
    return {};
  }

};
}  // namespace rocos
