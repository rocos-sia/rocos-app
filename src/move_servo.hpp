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

#include <sockpp/udp_socket.h>
#include <json.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "hardware_interface.hpp"
#include "model_interface.hpp"
#include "motion_interface.hpp"
#include "result.hpp"
#include "servo_communication.hpp"
#include "servo_signal_processor.hpp"
#include "servo_type.hpp"
#include "servo_session.hpp"
#include "servo_timing_trace.hpp"

namespace rocos {

/// @brief UDP 伺服运动模式 —— 接收外部 MotionGeneratorCommand 并生成 Reference
///
/// UDP 线程接收指令并按控制周期推送状态；控制线程生成 Reference。
/// 首条运动指令单独保留并校验，后续使用序号严格递增的最新指令。
/// 生命周期：SERVOING → STOPPING → STOPPED；校验失败进入 ERROR_STATE。
/// mtx_ 保护指令、首点检查和模式切换；停止后才释放 socket。
class MoveServo : public MotionInterface {
 public:
  /// @param hw          硬件接口指针（生命周期由 Robot 管理）
  /// @param model       运动学模型指针
  /// @param listen_port UDP 监听端口
  explicit MoveServo(HardwareInterface* hw, ModelInterface* model = nullptr,
                     uint16_t listen_port = 8081,
                     const ServoSettings& settings = {},
                     const Frame& tool = Frame::Identity(),
                     std::shared_ptr<ServoSession> session = nullptr);

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
  Result SetMode(MotionMode mode);
  // 故障/销毁立即中止资源；普通 Stop 保留控制周期以受限减速。
  void Abort();
  struct Diagnostics {
    uint64_t cycle{0}, received_id{0}, control_cycles{0}, elapsed_periods{0};
    double success_rate{0};
    uint32_t missing_cycles{0};
    bool stopping{false};
    Result fault{Result::NoError};
    JntArray desired_position, desired_velocity, desired_acceleration;
  };
  Diagnostics GetDiagnostics() const;
  std::shared_ptr<ServoSession> GetSession() const { return session_; }
  void CompleteCycle(Result result);
  RobotState GetPublishedState() const;
  nlohmann::json GetStatus() const;


 private:
  // ─── UDP 收发线程 ───
  void udpReceiveLoop();

  // ─── 从硬件读取状态并构建 RobotState（UDP 线程内调用） ───
  RobotState buildRobotState();

  /// @brief 无指令时使用实际关节位置保持，避免控制器重复求 IK。
  Result fillCurrentReference(Reference& ref_out);

  Result commandReference(const MotionGeneratorCommand& cmd,
                          Reference& ref_out) const;
  Result validateStartReference(const Reference& ref);

  // ─── 矩阵转换辅助：16 元素 row-major 4x4 <-> rocos::Frame ───
  static Frame matrixToFrame(const std::array<double, 16>& mat);
  static void frameToMatrix(const Frame& f, std::array<double, 16>& mat);

  Result fail(Result result);
  void beginStop(Result reason);
  Result processStop(Reference& ref_out);

  // ─── 非拥有指针（Robot 管理生命周期） ───
  HardwareInterface* hw_{nullptr};
  uint16_t listen_port_{8081};

  // ─── UDP socket ───
  sockpp::udp_socket sock_;

  // ─── 共享数据（mtx_ 保护：UDP 线程写 / 控制线程读） ───
  mutable std::mutex mtx_;
  std::shared_ptr<ServoSession> session_;
  RobotState published_state_{};
  sockpp::inet_address peer_;
  bool peer_bound_{false};
  FeedbackSnapshot cycle_feedback_;
  uint64_t applied_id_{0}, generated_id_{0};
  MotionGeneratorCommand cmd_{};
  bool has_new_cmd_{false};
  MotionGeneratorCommand first_cmd_{};
  bool first_command_checked_{false};
  Result fault_{Result::NoError};

  // ─── 伺服模式（默认在 Reset 中设为 kJointPosition） ───
  std::atomic<MotionMode> current_mode_{MotionMode::kNone};

  // ─── UDP 接收线程 ───
  std::thread thread_;
  int cycle_event_fd_{-1}; // CompleteCycle 通知 UDP；关闭与写入由 mtx_ 串行化。

  // ─── 原子标志 ───
  std::atomic<bool> running_{false};
  std::atomic<bool> stopped_{false};

  // ─── UDP 线程独享状态（仅 udpReceiveLoop 访问，无需锁） ───
  uint64_t message_id_counter_{0};
  int joint_count_{0};

  ServoSettings settings_;
  Frame tool_;
  ServoSignalProcessor processor_;
  ServoCommunication communication_;
  std::unique_ptr<ServoTimingTrace> timing_trace_;
  ServoTimingTrace::ControlRecord timing_cycle_{};
  bool processor_ready_{false}, stopping_{false}, stop_complete_{false};
  Result stop_reason_{Result::NoError};
  uint64_t consumed_id_{0}, cycle_{0};
  uint32_t cycle_period_us_{0};
  uint32_t low_quality_count_{0};

  using Clock = std::chrono::steady_clock;
  Clock::time_point started_at_{}, last_received_at_{}, last_cycle_at_{};

  // ─── 兼容旧协议，成功率在控制线程计算 ───
};

}  // namespace rocos
