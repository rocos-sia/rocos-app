#include <arpa/inet.h>
#include <httplib.h>
#include <librocos/robot.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <json.hpp>
#include <limits>
#include <mutex>
#include <thread>

#include "servo_type.hpp"

namespace librocos {
namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
static_assert(kMaxJoints == MAX_DOF, "Client/server joint capacity mismatch");
static_assert(sizeof(MotionGeneratorCommand) == 236, "UDP v2 command layout changed");

void require(bool condition, const std::string& message) {
    if (!condition) throw ProtocolException(message);
}

std::uint64_t unsignedField(const Json& json, const char* name) {
    require(json.at(name).is_number_unsigned(), std::string("Invalid unsigned field ") + name);
    return json.at(name).get<std::uint64_t>();
}

void jointArray(const Json& json, const char* name, JointArray& output, bool faulted) {
    const auto& values = json.at(name);
    require(values.is_array() && values.size() == output.size(),
            std::string("Invalid array ") + name);
    for (std::size_t i = 0; i < output.size(); ++i) {
        // nlohmann 将非有限浮点数编码为 null，故障诊断仍需保留停止状态。
        if (faulted && values[i].is_null()) {
            output[i] = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        output[i] = values[i].get<double>();
        require(std::isfinite(output[i]), std::string("Nonfinite array ") + name);
    }
}

ServoSnapshot snapshot(const Json& json) {
    ServoSnapshot result;
    result.cycle = unsignedField(json, "cycle");
    result.sample_time_ns = unsignedField(json, "sample_time_ns");
    result.source_time_us = unsignedField(json, "source_time_us");
    result.received_message_id = unsignedField(json, "received_message_id");
    result.applied_message_id = unsignedField(json, "applied_message_id");
    result.control_cycles = unsignedField(json, "control_cycles");
    result.elapsed_periods = unsignedField(json, "elapsed_periods");
    result.fault_code = json.at("fault_code").get<int>();
    result.output_mode = json.at("output_mode").get<int>();
    result.output_valid = json.at("output_valid").get<bool>();
    result.control_command_success_rate = json.at("control_command_success_rate").get<double>();
    require(std::isfinite(result.control_command_success_rate) &&
                result.control_command_success_rate >= 0 &&
                result.control_command_success_rate <= 1,
            "Invalid command success rate");
    const auto& errors = json.at("errors");
    require(errors.is_array() && errors.size() == result.errors.size(), "Invalid error flags");
    for (std::size_t i = 0; i < result.errors.size(); ++i) result.errors[i] = errors[i].get<bool>();
    jointArray(json, "q", result.q, result.fault_code != 0);
    jointArray(json, "dq", result.dq, result.fault_code != 0);
    jointArray(json, "q_d", result.q_d, result.fault_code != 0);
    jointArray(json, "dq_d", result.dq_d, result.fault_code != 0);
    jointArray(json, "ddq_d", result.ddq_d, result.fault_code != 0);
    jointArray(json, "q_cmd", result.q_cmd, result.fault_code != 0);
    jointArray(json, "tau_cmd", result.tau_cmd, result.fault_code != 0);
    return result;
}

RciStatus statusFromJson(const Json& json) {
    try {
        RciStatus status;
        status.enabled = json.at("enabled").get<bool>();
        status.closing = json.at("closing").get<bool>();
        status.control_active = json.at("control_active").get<bool>();
        status.robot_state = json.at("robot_state").get<std::string>();
        if (!json.at("last_control").is_null())
            status.last_control = snapshot(json.at("last_control"));
        if (json.contains("current_control"))
            status.current_control = snapshot(json.at("current_control"));
        return status;
    } catch (const Json::exception&) {
        throw ProtocolException("Invalid RCI status response");
    }
}

int faultCode(const RciStatus& status) {
    if (status.current_control) return status.current_control->fault_code;
    if (status.last_control) return status.last_control->fault_code;
    return 0;
}

JointArray wireJoints(const unsigned char* bytes, std::size_t offset) {
    JointArray result;
    // UDP v2 使用 packed 布局，先复制到对齐的数组再访问 double。
    std::memcpy(result.data(), bytes + offset, sizeof(result));
    return result;
}

std::array<std::uint8_t, 16> sessionId(const Json& data) {
    require(unsignedField(data, "protocol_version") == SERVO_PROTOCOL_VERSION,
            "Unsupported UDP protocol version");
    const auto value = data.at("session_id").get<std::string>();
    require(value.size() == 32, "Invalid RCI session ID length");
    std::array<std::uint8_t, 16> result{};
    auto digit = [](char ch) {
        if (ch >= '0' && ch <= '9') return ch - '0';
        if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
        throw ProtocolException("Invalid RCI session ID encoding");
    };
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] = static_cast<std::uint8_t>(digit(value[2 * i]) * 16 + digit(value[2 * i + 1]));
    return result;
}

class UdpSocket {
   public:
    UdpSocket(const std::string& host, std::uint16_t port) {
        const auto udp_host = host == "::1" ? "127.0.0.1" : host;
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo* addresses = nullptr;
        if (getaddrinfo(udp_host.c_str(), nullptr, &hints, &addresses) != 0)
            throw NetworkException("Unable to resolve UDP host");
        bool loopback = false;
        for (auto* item = addresses; item; item = item->ai_next) {
            const auto* address = reinterpret_cast<const sockaddr_in*>(item->ai_addr);
            loopback = loopback || address->sin_addr.s_addr == htonl(INADDR_LOOPBACK);
        }
        freeaddrinfo(addresses);
        if (!loopback)
            throw InvalidOperationException("RCI UDP v2 requires a local 127.0.0.1 client");
        fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) throw NetworkException("Unable to create UDP socket");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(port);
        if (::connect(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            ::close(fd_);
            throw NetworkException("Unable to connect UDP socket");
        }
    }
    ~UdpSocket() { ::close(fd_); }
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    void send(const MotionGeneratorCommand& command) {
        if (::send(fd_, &command, sizeof(command), 0) != sizeof(command))
            throw NetworkException("Unable to send UDP command");
    }

    // 排空有界队列，校验原始 bool 字节后才构造协议结构体。
    bool receive(const std::array<std::uint8_t, 16>& session, std::size_t dof,
                 std::uint64_t& reply_id, RobotState& state) {
        bool fresh = false;
        for (int attempt = 0; attempt < 64; ++attempt) {
            std::array<unsigned char, sizeof(::RobotState) + 1> bytes{};
            const auto count = ::recv(fd_, bytes.data(), bytes.size(), MSG_DONTWAIT);
            if (count < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                if (errno == EINTR) continue;
                throw NetworkException("Unable to receive UDP state");
            }
            if (count != sizeof(::RobotState)) continue;
            const auto error_begin = bytes.begin() + offsetof(::RobotState, errors);
            if (!std::all_of(error_begin, error_begin + 41, [](auto byte) { return byte <= 1; }) ||
                bytes[offsetof(::RobotState, output_valid)] > 1)
                continue;
            ::RobotState packet{};
            std::memcpy(&packet, bytes.data(), sizeof(packet));
            if (packet.protocol_version != SERVO_PROTOCOL_VERSION || packet.session_id != session ||
                packet.message_id <= reply_id || packet.mode != MotionMode::kJointPosition)
                continue;
            if (!std::isfinite(packet.control_command_success_rate) ||
                packet.control_command_success_rate < 0 || packet.control_command_success_rate > 1)
                continue;
            RobotState next;
            next.q = wireJoints(bytes.data(), offsetof(::RobotState, q));
            next.dq = wireJoints(bytes.data(), offsetof(::RobotState, dq));
            next.tau = wireJoints(bytes.data(), offsetof(::RobotState, tau));
            next.q_d = wireJoints(bytes.data(), offsetof(::RobotState, q_d));
            next.dq_d = wireJoints(bytes.data(), offsetof(::RobotState, dq_d));
            next.ddq_d = wireJoints(bytes.data(), offsetof(::RobotState, ddq_d));
            next.q_cmd = wireJoints(bytes.data(), offsetof(::RobotState, q_cmd));
            next.tau_cmd = wireJoints(bytes.data(), offsetof(::RobotState, tau_cmd));
            bool finite = true;
            for (std::size_t i = 0; i < dof; ++i) {
                finite = finite && std::isfinite(next.q[i]) && std::isfinite(next.dq[i]) &&
                         std::isfinite(next.tau[i]) && std::isfinite(next.q_d[i]) &&
                         std::isfinite(next.dq_d[i]) && std::isfinite(next.ddq_d[i]) &&
                         std::isfinite(next.q_cmd[i]) && std::isfinite(next.tau_cmd[i]);
            }
            if (!finite) throw ProtocolException("Nonfinite UDP state");
            reply_id = packet.message_id;
            if (std::any_of(packet.errors.begin(), packet.errors.end(),
                            [](bool value) { return value; }))
                throw ControlException("Controller reported a servo fault");
            if (packet.cycle <= state.cycle || packet.sample_time_ns <= state.sample_time_ns)
                continue;
            if (!packet.output_valid) throw ControlException("Controller output is invalid");
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    Clock::now().time_since_epoch())
                                    .count();
            if (packet.sample_time_ns > static_cast<std::uint64_t>(now_ns))
                throw ProtocolException("UDP sample timestamp is in the future");
            state = std::move(next);
            state.source = RobotState::Source::Udp;
            state.reference_valid = true;
            state.enabled = true;
            state.robot_state = "SERVOING";
            state.work_mode = "position";
            state.dof = dof;
            state.cycle = packet.cycle;
            state.sample_time_ns = packet.sample_time_ns;
            state.source_time_us = packet.source_time_us;
            state.received_message_id = packet.received_message_id;
            state.applied_message_id = packet.applied_message_id;
            state.errors = packet.errors;
            state.output_mode = packet.output_mode;
            state.output_valid = packet.output_valid;
            state.control_command_success_rate = packet.control_command_success_rate;
            fresh = true;
        }
        return fresh;
    }

   private:
    int fd_{-1};
};
}  // namespace

Exception::Exception(const std::string& message, int code)
    : std::runtime_error(code ? message + " (code=" + std::to_string(code) + ")" : message),
      code_(code) {}

JointPositions::JointPositions(const JointArray& positions, std::size_t joint_count)
    : q(positions), dof(joint_count) {
    if (dof == 0 || dof > kMaxJoints) throw std::invalid_argument("Invalid joint count");
}
JointPositions::JointPositions(std::initializer_list<double> positions) : dof(positions.size()) {
    if (dof == 0 || dof > kMaxJoints) throw std::invalid_argument("Invalid joint count");
    std::copy(positions.begin(), positions.end(), q.begin());
}

struct Robot::Impl {
    std::string host;
    Options options;
    std::mutex operation_mutex, wait_mutex;
    std::condition_variable wakeup;
    std::atomic<bool> stop_requested{false};
    bool session_owned{false};  // 受 operation_mutex 保护。
    httplib::Headers credentials;

    Impl(std::string address, Options settings) : host(std::move(address)), options(settings) {
        if (host.empty() || !options.http_port || !options.udp_port ||
            options.control_period.count() <= 0 ||
            options.receive_timeout <= options.control_period ||
            options.startup_timeout.count() <= 0 || options.stop_timeout.count() <= 0 ||
            options.http_timeout.count() <= 0 || options.read_period.count() <= 0)
            throw std::invalid_argument("Invalid librocos connection or timing options");
    }

    std::unique_lock<std::mutex> operation() {
        std::unique_lock<std::mutex> lock(operation_mutex, std::try_to_lock);
        if (!lock.owns_lock())
            throw InvalidOperationException("Another read/control/session operation is active");
        return lock;
    }

    Json http(const std::string& path, const Json* request = nullptr,
              const httplib::Headers& headers = {}) const {
        httplib::Client client(host, options.http_port);
        client.set_connection_timeout(options.http_timeout);
        client.set_read_timeout(options.http_timeout);
        client.set_write_timeout(options.http_timeout);
        const auto response = request
                                  ? client.Post(path, headers, request->dump(), "application/json")
                                  : client.Get(path, headers);
        if (!response)
            throw NetworkException("HTTP " + path + ": " + httplib::to_string(response.error()));
        if (response->status != 200)
            throw NetworkException("HTTP " + path + " status " + std::to_string(response->status));
        try {
            const auto body = Json::parse(response->body);
            const auto success = body.at("success").get<bool>();
            const auto code = body.at("code").get<int>();
            const auto message = body.at("message").get<std::string>();
            if (!success) {
                ControlException error(path + ": " + message, code);
                if (body.contains("data") && body["data"].is_object() &&
                    body["data"].contains("enabled"))
                    error.status = statusFromJson(body["data"]);
                throw error;
            }
            require(code == 0, "Inconsistent HTTP success/code at " + path);
            return body.value("data", Json());
        } catch (const Json::exception&) {
            throw ProtocolException("Invalid HTTP JSON at " + path);
        }
    }

    RciStatus status() const { return statusFromJson(http("/api/robot/rci/status")); }

    void release() {
        if (credentials.empty()) return;
        const Json body = Json::object();
        try {
            http("/api/control/release", &body, credentials);
        } catch (const ControlException& error) {
            if (error.code() != 3006 && error.code() != 3007) throw;
            // 已过期或撤销的 HTTP 租约可以清除本地缓存。
        }
        credentials.clear();
    }

    void setRci(bool enabled) {
        release();
        const auto current = status();
        if (current.enabled == enabled && !current.closing) return;
        static std::atomic<unsigned int> sequence{0};
        const auto id = "librocos-" + std::to_string(getpid()) + "-" + std::to_string(++sequence);
        const Json acquire = {{"client_id", id}, {"client_name", "librocos"}};
        const auto owner = http("/api/control/acquire", &acquire);
        try {
            credentials = {{"X-Rocos-Control-Token", owner.at("token").get<std::string>()},
                           {"X-Rocos-Client-Id", id}};
        } catch (const Json::exception&) {
            throw ProtocolException("Invalid HTTP control ownership response");
        }
        const Json body = Json::object();
        try {
            http(enabled ? "/api/robot/rci/enable" : "/api/robot/rci/disable", &body, credentials);
        } catch (...) {
            const auto original = std::current_exception();
            try {
                release();
            } catch (...) {
                ControlException error("RCI request failed and HTTP control release failed");
                error.cause = original;
                throw error;
            }
            std::rethrow_exception(original);
        }
        release();
        if (!enabled) {
            const auto deadline = Clock::now() + options.stop_timeout;
            for (;;) {
                const auto state = status();
                if (!state.enabled && !state.control_active && !state.closing) return;
                if (Clock::now() >= deadline)
                    throw ControlException("Timed out waiting for RCI disable");
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }

    void waitUntil(Clock::time_point deadline) {
        std::unique_lock<std::mutex> lock(wait_mutex);
        wakeup.wait_until(lock, deadline, [this] { return stop_requested.load(); });
    }

    RciStatus finishSession() {
        auto state = status();
        if (state.control_active) {
            const Json body = Json::object();
            http("/api/robot/rci/stop", &body);
        }
        const auto deadline = Clock::now() + options.stop_timeout;
        for (;;) {
            state = status();
            if (!state.control_active && state.robot_state != "STOPPING" && !state.closing) {
                session_owned = false;
                return state;
            }
            if (Clock::now() >= deadline) throw ControlException("Timed out waiting for RCI stop");
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
};

Robot::Robot(const std::string& host, Options options)
    : impl_(std::make_unique<Impl>(host, options)) {}
Robot::~Robot() {
    try {
        if (impl_->session_owned) impl_->finishSession();
    } catch (...) {
    }
    try {
        impl_->release();
    } catch (...) {
    }
}

RobotState Robot::readOnce() const {
    const auto json = impl_->http("/api/robot/state");
    try {
        RobotState state;
        const auto& joints = json.at("joint_states");
        state.dof = unsignedField(json.at("hw_state"), "joint_num");
        require(joints.is_array() && state.dof > 0 && state.dof <= kMaxJoints &&
                    joints.size() == state.dof,
                "Joint count disagrees with HTTP state");
        state.robot_state = json.at("robot_state").get<std::string>();
        state.work_mode = json.at("work_mode").get<std::string>();
        state.enabled = json.at("is_enabled").get<bool>();
        state.timestamp = json.at("timestamp").get<double>();
        require(std::isfinite(state.timestamp), "Invalid HTTP sample timestamp");
        for (std::size_t i = 0; i < state.dof; ++i) {
            state.q[i] = joints[i].at("position").get<double>();
            state.dq[i] = joints[i].at("velocity").get<double>();
            state.tau[i] = joints[i].at("torque").get<double>();
            require(std::isfinite(state.q[i]) && std::isfinite(state.dq[i]) &&
                        std::isfinite(state.tau[i]),
                    "Nonfinite HTTP joint state");
        }
        return state;
    } catch (const Json::exception&) {
        throw ProtocolException("Invalid robot state response");
    }
}

void Robot::read(const std::function<bool(const RobotState&)>& callback) {
    if (!callback) throw std::invalid_argument("Empty read callback");
    auto lock = impl_->operation();
    impl_->stop_requested = false;
    while (!impl_->stop_requested) {
        const auto deadline = Clock::now() + impl_->options.read_period;
        if (!callback(readOnce())) return;
        impl_->waitUntil(deadline);
    }
}

RciStatus Robot::rciStatus() const { return impl_->status(); }
void Robot::enableRci() {
    auto lock = impl_->operation();
    impl_->setRci(true);
}
void Robot::disableRci() {
    auto lock = impl_->operation();
    impl_->setRci(false);
}
void Robot::stop() {
    impl_->stop_requested = true;
    impl_->wakeup.notify_all();
    std::unique_lock<std::mutex> lock(impl_->operation_mutex, std::try_to_lock);
    if (lock.owns_lock() && impl_->session_owned) impl_->finishSession();
}

void Robot::control(const std::function<JointPositions(const RobotState&, Duration)>& callback) {
    if (!callback) throw std::invalid_argument("Empty control callback");
    auto lock = impl_->operation();
    if (impl_->session_owned)
        throw InvalidOperationException("Previous RCI stop is unconfirmed; call stop() first");
    impl_->stop_requested = false;
    const auto initial = readOnce();
    if (initial.work_mode != "position")
        throw InvalidOperationException("Joint position control requires the position work mode");
    UdpSocket socket(impl_->host, impl_->options.udp_port);
    const auto& options = impl_->options;
    const Json request = {{"port", options.udp_port}, {"mode", "joint"}};
    // 只有已确认启动成功才取得清理责任，拒绝接入时不会停止他人的会话。
    const auto data = impl_->http("/api/robot/rci/start", &request);
    impl_->session_owned = true;
    try {
        MotionGeneratorCommand command{};
        try {
            command.session_id = sessionId(data);
            require(unsignedField(data, "port") == options.udp_port, "RCI UDP port mismatch");
        } catch (const Json::exception&) {
            throw ProtocolException("Invalid RCI startup response");
        }
        RobotState state;
        std::uint64_t reply_id = 0;
        auto deadline = Clock::now() + options.startup_timeout;
        while (!impl_->stop_requested && !state.reference_valid) {
            socket.send(command);  // 序号 0 查询保持参考，绝不发送零位置首点。
            impl_->waitUntil(Clock::now() + options.control_period);
            socket.receive(command.session_id, initial.dof, reply_id, state);
            if (Clock::now() >= deadline)
                throw NetworkException("Timed out waiting for initial UDP state");
        }
        std::uint64_t callback_time = 0;
        std::uint64_t finished_id = 0;
        auto next_tick = Clock::now();
        auto last_fresh = Clock::now();
        auto finish_deadline = Clock::time_point::max();
        while (!impl_->stop_requested) {
            impl_->waitUntil(next_tick);
            if (impl_->stop_requested) break;
            if (socket.receive(command.session_id, initial.dof, reply_id, state))
                last_fresh = Clock::now();
            const auto sample_time =
                Clock::time_point(std::chrono::nanoseconds(state.sample_time_ns));
            if (Clock::now() - last_fresh >= options.receive_timeout ||
                Clock::now() - sample_time >= options.receive_timeout)
                throw NetworkException("UDP feedback is stale or missing");
            if (finished_id && state.applied_message_id >= finished_id) break;
            if (Clock::now() >= finish_deadline)
                throw NetworkException("Final command application was not confirmed");
            if (!finished_id && state.sample_time_ns > callback_time) {
                const Duration period(
                    callback_time ? std::chrono::nanoseconds(state.sample_time_ns - callback_time)
                                  : std::chrono::nanoseconds::zero());
                const auto generated = callback(state, period);
                if (impl_->stop_requested) break;
                if (Clock::now() - sample_time >= options.receive_timeout)
                    throw ControlException("Control callback exceeded feedback freshness deadline");
                if (generated.dof != initial.dof)
                    throw ControlException("Joint command dimension mismatch");
                for (std::size_t i = 0; i < initial.dof; ++i) {
                    if (!std::isfinite(generated.q[i]))
                        throw ControlException("Nonfinite joint position command");
                }
                std::memcpy(reinterpret_cast<unsigned char*>(&command) +
                                offsetof(MotionGeneratorCommand, q_c),
                            generated.q.data(), sizeof(generated.q));
                callback_time = state.sample_time_ns;
                if (generated.motion_finished) {
                    finished_id = command.message_id + 1;
                    finish_deadline = Clock::now() + options.receive_timeout;
                }
            }
            if (command.message_id == std::numeric_limits<std::uint64_t>::max())
                throw ControlException("UDP command sequence exhausted");
            ++command.message_id;
            socket.send(command);
            // 新采样到来前重发保持目标并推进序号，回调只处理新采样，超期不突发补发。
            next_tick += options.control_period;
            if (next_tick <= Clock::now()) next_tick = Clock::now() + options.control_period;
        }
        const auto status = impl_->finishSession();
        const auto code = faultCode(status);
        if (code || status.robot_state == "ERROR_STATE") {
            ControlException error("Controller stopped with a fault", code);
            error.status = status;
            throw error;
        }
    } catch (...) {
        const auto original = std::current_exception();
        if (!impl_->session_owned) std::rethrow_exception(original);
        RciStatus status;
        try {
            status = impl_->finishSession();
        } catch (const std::exception& cleanup) {
            ControlException error(std::string("RCI stop could not be confirmed: ") +
                                   cleanup.what());
            error.cause = original;
            throw error;
        }
        if (faultCode(status)) {
            ControlException error("Controller stopped with a servo fault", faultCode(status));
            error.status = status;
            error.cause = original;
            throw error;
        }
        std::rethrow_exception(original);
    }
}
}  // namespace librocos
