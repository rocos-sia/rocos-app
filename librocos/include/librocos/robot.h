#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

namespace librocos {

inline constexpr std::size_t kMaxJoints = 10;
using JointArray = std::array<double, kMaxJoints>;

class Duration {
   public:
    explicit Duration(std::chrono::nanoseconds value = {}) : value_(value) {}
    double toSec() const { return std::chrono::duration<double>(value_).count(); }
    std::int64_t toNSec() const { return value_.count(); }

   private:
    std::chrono::nanoseconds value_;
};

struct ServoSnapshot {
    std::uint64_t cycle{}, sample_time_ns{}, source_time_us{};
    std::uint64_t received_message_id{}, applied_message_id{};
    JointArray q{}, dq{}, q_d{}, dq_d{}, ddq_d{}, q_cmd{}, tau_cmd{};
    std::array<bool, 41> errors{};
    int fault_code{}, output_mode{};
    bool output_valid{};
    // 最近至多 100 个控制周期的新指令接收比例。
    double control_command_success_rate{};
    // 以下计数由 HTTP RCI 诊断提供，UDP v2 未携带，control 状态中为零。
    std::uint64_t control_cycles{}, elapsed_periods{};
};

struct RobotState : ServoSnapshot {
    enum class Source { Http, Udp };
    std::size_t dof{};
    JointArray tau{};
    Source source{Source::Http};
    // HTTP readOnce 只提供实际状态，周期参考由 control 的 UDP 快照提供。
    bool reference_valid{};
    bool enabled{};
    std::string robot_state;
    std::string work_mode;
    double timestamp{};  // HTTP 快照的 epoch seconds。
};

struct RciStatus {
    bool enabled{}, closing{}, control_active{};
    std::string robot_state;
    std::optional<ServoSnapshot> current_control, last_control;
};

class Exception : public std::runtime_error {
   public:
    explicit Exception(const std::string& message, int code = 0);
    int code() const noexcept { return code_; }

   private:
    int code_;
};
class NetworkException : public Exception {
   public:
    using Exception::Exception;
};
class ProtocolException : public Exception {
   public:
    using Exception::Exception;
};
class InvalidOperationException : public Exception {
   public:
    using Exception::Exception;
};
class ControlException : public Exception {
   public:
    using Exception::Exception;
    std::optional<RciStatus> status;
    std::exception_ptr cause;
};

struct JointPositions {
    JointArray q{};
    std::size_t dof{};
    bool motion_finished{};
    JointPositions(const JointArray& positions, std::size_t joint_count);
    JointPositions(std::initializer_list<double> positions);
    template <std::size_t N>
    explicit JointPositions(const std::array<double, N>& positions) : dof(N) {
        static_assert(N > 0 && N <= kMaxJoints, "Unsupported joint count");
        for (std::size_t i = 0; i < N; ++i) q[i] = positions[i];
    }
};

inline JointPositions MotionFinished(JointPositions command) {
    command.motion_finished = true;
    return command;
}

struct Options {
    std::uint16_t http_port{8080};
    std::uint16_t udp_port{8081};
    // 当前 HTTP 契约的硬件周期可为 null，由调用者匹配实际配置。
    std::chrono::microseconds control_period{1000};
    std::chrono::milliseconds receive_timeout{20};
    std::chrono::milliseconds startup_timeout{500};
    std::chrono::milliseconds stop_timeout{3000};
    std::chrono::milliseconds http_timeout{1000};
    std::chrono::milliseconds read_period{20};
};

class Robot {
   public:
    explicit Robot(const std::string& host, Options options = {});
    ~Robot();
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    RobotState readOnce() const;
    // 普通 HTTP 状态轮询，callback 返回 false 结束。
    void read(const std::function<bool(const RobotState&)>& callback);
    RciStatus rciStatus() const;
    void enableRci();
    void disableRci();
    // 阻塞调用。回调与 UDP 收发在调用线程执行，首周期 period=0。
    void control(const std::function<JointPositions(const RobotState&, Duration)>& callback);
    // 可从另一线程请求结束 read/control，control 返回前等待服务端停止。
    // 回调应及时返回；回调阻塞期间由服务端通信看门狗负责保护。
    void stop();

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace librocos
