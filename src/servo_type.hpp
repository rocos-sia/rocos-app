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

#include <array>
#include <cstddef>
#include <cstdint>

constexpr size_t MAX_DOF = 10;
constexpr uint32_t SERVO_PROTOCOL_VERSION = 2;

#pragma pack(push, 1)   // 强制 1 字节对齐，消除 padding

enum class MotionMode : uint8_t {
  kJointPosition,
  kCartesianPosition,
  kNone
};

struct RobotState {     // Robot → Client, ~800+ bytes
  uint32_t protocol_version{SERVO_PROTOCOL_VERSION};
  std::array<uint8_t, 16> session_id{};
  uint64_t message_id;              // 回包序号，非时间戳
  std::array<double, 16> flange_to_base;    // 法兰位姿（4x4 变换矩阵，row major）
  std::array<double, 16> tcp_to_base;    // TCP位姿

  std::array<double, MAX_DOF> tau;      // 关节扭矩
  std::array<double, MAX_DOF> load;     // 负载扭矩
  std::array<double, MAX_DOF> q;        // 关节位置
  std::array<double, MAX_DOF> dq;       // 关节速度

  // 0: 任意伺服故障；1: 首点；2: 通信；3: 运动约束；4..40 保留。
  std::array<bool, 41> errors;


  MotionMode mode;              // 机器人模式
  uint64_t cycle{0}, sample_time_ns{0}, source_time_us{0};
  uint64_t received_message_id{0}, applied_message_id{0};
  std::array<double, MAX_DOF> q_d{}, dq_d{}, ddq_d{}, q_cmd{}, tau_cmd{};
  int8_t output_mode{0};
  bool output_valid{false}; // 写入硬件接口成功，不代表驱动执行确认
  double control_command_success_rate; // 最近至多 100 个控制周期收到新指令的比例
};


struct MotionGeneratorCommand {   // 运动生成指令
  uint32_t protocol_version{SERVO_PROTOCOL_VERSION};
  std::array<uint8_t, 16> session_id{};
  uint64_t message_id;
  std::array<double, 16> tcp_c;    // 笛卡尔位姿指令
  std::array<double, MAX_DOF> q_c;          // 关节位置指令

};

#pragma pack(pop)