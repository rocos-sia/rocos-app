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
#include <atomic>
#include <mutex>
#include <thread>

#include <sockpp/udp_socket.h>

#include "hardware_interface.hpp"
#include "model_interface.hpp"
#include "motion_interface.hpp"
#include "result.hpp"
#include "servo_type.hpp"

namespace rocos {

/// @brief UDP 伺服运动模式 —— 接收外部 MotionGeneratorCommand 并生成 Reference
///
/// 通信模式参照 libfranka (docs/libfranka_udp.md)：
///   1. 后台线程持续监听 localhost:8081，两阶段接收：
///      Phase 1: 非阻塞排空缓冲区，丢弃旧包，保留 message_id 最大的指令
///      Phase 2: 无新包时短暂休眠避免忙等
///   2. 控制线程通过 Update() 读取硬件状态，构建 RobotState 并回复客户端
///   3. GenerateRef() 将最新收到的指令转为 Reference（JntArray 或 Frame）
///
/// 生命周期：SERVOING 状态进入 → EventStopReq 退出（STOPPING → STOPPED）
///
/// 线程安全：UDP 接收线程写 cmd_/client_addr_；控制线程读 cmd_/client_addr_，
///           mtx_ 保护共享数据。
class MoveServo : public MotionInterface {
 public:
  /// @param hw          硬件接口指针（生命周期由 Robot 管理）
  /// @param model       运动学模型指针
  /// @param listen_port UDP 监听端口
  explicit MoveServo(HardwareInterface* hw,
                     ModelInterface* model = nullptr,
                     uint16_t listen_port = 8081);

  ~MoveServo() override;

  // ─── MotionInterface 接口 ───
  Result ValidateParameters() const override;
  Result Reset() override;
  Result GenerateRef(Reference& ref_out) override;

  bool CanPause() const override { return false; }
  bool CanResume() const override { return false; }
  bool CanStop() const override { return true; }

  Result Pause() override;
  Result Resume() override;
  Result Stop() override;

  /// @brief 设置当前伺服模式（joint 或 cartesian）
  void SetMode(MotionMode mode) { current_mode_ = mode; }

 private:
  // ─── UDP 收发线程 ───
  void udpReceiveLoop();

  // ─── 从硬件读取状态并构建 RobotState（UDP 线程内调用） ───
  RobotState buildRobotState();

  /// @brief 无新指令时的兜底：按 current_mode_ 把当前实际位置填入 ref_out
  ///        kJointPosition/kNone → 当前关节角；kCartesianPosition → FK 得到的位姿
  Result fillCurrentReference(Reference& ref_out);

  // ─── 矩阵转换辅助：16 元素 row-major 4x4 <-> rocos::Frame ───
  static Frame matrixToFrame(const std::array<double, 16>& mat);
  static void  frameToMatrix(const Frame& f, std::array<double, 16>& mat);

  // ─── 通信质量统计 ───
  void   recordSuccess(bool ok);
  double computeSuccessRate() const;

  // ─── 非拥有指针（Robot 管理生命周期） ───
  HardwareInterface* hw_{nullptr};
  uint16_t           listen_port_{8081};

  // ─── UDP socket ───
  sockpp::udp_socket sock_;

  // ─── 共享数据（mtx_ 保护：UDP 线程写 / 控制线程读） ───
  mutable std::mutex     mtx_;
  MotionGeneratorCommand cmd_{};
  bool                   has_new_cmd_{false};

  // ─── 伺服模式（默认在 Reset 中设为 kJointPosition） ───
  MotionMode current_mode_{MotionMode::kNone};

  // ─── UDP 接收线程 ───
  std::thread thread_;

  // ─── 原子标志 ───
  std::atomic<bool> running_{false};
  std::atomic<bool> stopped_{false};

  // ─── UDP 线程独享状态（仅 udpReceiveLoop 访问，无需锁） ───
  uint64_t message_id_counter_{0};
  int      joint_count_{0};

  // ─── 滑动窗口：最近 100 次 send 操作的成功统计 ───
  static constexpr int                kSuccessWindowSize = 100;
  std::array<bool, kSuccessWindowSize> success_window_{};
  int success_index_{0};
  int success_count_{0};
};

}  // namespace rocos
