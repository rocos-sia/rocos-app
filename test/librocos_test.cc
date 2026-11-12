#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <arpa/inet.h>
#include <httplib.h>
#include <librocos/robot.h>
#include <poll.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <future>
#include <json.hpp>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>
#ifdef LIBROCOS_EXAMPLE_DIR
#include <yaml-cpp/yaml.h>
#endif

#include "src/servo_type.hpp"
#include "src/realtime_thread.hpp"
#include "test/doctest.h"
#include "librocos/examples/joint_sine_trajectory.hpp"

extern char** environ;

TEST_CASE("realtime scheduling reports effective policy and preserves higher priority") {
    // 在独立线程中测试，退出时由内核回收调度设置。
    rocos::RealtimeThreadStatus invalid, requested, preserved;
    int effective_policy = -1;
    sched_param effective{};
    int query_error = -1;
    std::thread worker([&] {
        invalid = rocos::ConfigureRealtimeThread(0);
        requested = rocos::ConfigureRealtimeThread(20);
        query_error = pthread_getschedparam(pthread_self(), &effective_policy, &effective);
        if (requested.ready()) preserved = rocos::ConfigureRealtimeThread(10);
    });
    worker.join();
    CHECK(invalid.error == EINVAL);
    CHECK_FALSE(invalid.ready());
    CHECK(query_error == 0);
    CHECK(requested.query_error == 0);
    CHECK(requested.policy == effective_policy);
    CHECK(requested.priority == effective.sched_priority);
    CHECK_FALSE(requested.description().empty());
    if (requested.error == 0) {
        CHECK(requested.ready());
        CHECK(preserved.ready());
        CHECK(preserved.policy == requested.policy);
        CHECK(preserved.priority == requested.priority);
    } else {
        CHECK(requested.error == EPERM);
        CHECK_FALSE(requested.ready());
    }
}

namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

// 独立 TCP/UDP 协议对端，不接触设备或现有 App。
class Peer {
   public:
    httplib::Server http;
    librocos::Options options;
    std::atomic<bool> running{true}, drop{false}, freeze{false}, wrong_session{false};
    std::atomic<bool> oversize{false}, nonfinite{false}, stuck_stop{false}, reject_start{false};
    std::atomic<bool> stale_ack{false};
    std::atomic<std::int64_t> clock_offset_ns{0};
    std::atomic<double> reported_position_lag{0};
    std::atomic<int> cycle_period_us{1000}, skip_cycle{0};
    bool peer_ready{false};
    sockaddr_in peer_address{};
    std::vector<uint64_t> command_cycles;
    std::atomic<int> fault{0}, stops{0}, starts{0}, acquisitions{0}, releases{0}, commands{0};
    bool rci_enabled{false}, active{false}, closing{false};
    std::string robot_state{"STOPPED"}, work_mode{"position"};
    std::string start_session_override;
    std::size_t dof{7};
    int protocol_version{2};
    std::array<double, MAX_DOF> first_q{};
    std::mutex mutex;
    ::RobotState state{};
    std::thread http_thread, udp_thread;
    int socket_fd{-1};
    Clock::time_point stopping_at{};
    bool lease{false};
    std::string client_id;

    Peer(std::string urdf = {}, Json joint_infos = Json::array(),
         const std::string& bind_host = "127.0.0.1") {
        options.http_timeout = std::chrono::milliseconds(300);
        options.startup_timeout = std::chrono::milliseconds(100);
        options.receive_timeout = std::chrono::milliseconds(40);
        options.stop_timeout = std::chrono::milliseconds(150);
        options.read_period = std::chrono::milliseconds(1);
        socket_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        REQUIRE(socket_fd >= 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        REQUIRE(inet_pton(AF_INET, bind_host.c_str(), &address.sin_addr) == 1);
        REQUIRE(::bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
        socklen_t length = sizeof(address);
        REQUIRE(::getsockname(socket_fd, reinterpret_cast<sockaddr*>(&address), &length) == 0);
        options.udp_port = ntohs(address.sin_port);
        state.mode = MotionMode::kJointPosition;
        state.output_valid = true;
        state.control_command_success_rate = 1;
        http.Get("/api/robot/urdf", [urdf](const auto&, auto& response) {
            response.set_content(urdf, "text/xml");
        });
        http.Get("/api/robot/info", [joint_infos](const auto&, auto& response) {
            reply(response, {{"joint_infos", joint_infos}});
        });
        http.Get("/api/robot/state", [&](const auto&, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            Json joints = Json::array();
            for (std::size_t i = 0; i < dof; ++i)
                joints.push_back({{"position", state.q[i]}, {"velocity", 0}, {"torque", 0}});
            reply(response, {{"joint_states", joints},
                             {"hw_state", {{"joint_num", dof}}},
                             {"robot_state", robot_state},
                             {"work_mode", work_mode},
                             {"is_enabled", true},
                             {"timestamp", 123.0}});
        });
        http.Get("/api/robot/rci/status", [&](const auto&, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            advanceStop();
            reply(response, status());
        });
        http.Post("/api/control/acquire", [&](const auto& request, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            if (lease) {
                reply(response, nullptr, false, 3008);
                return;
            }
            client_id = Json::parse(request.body).at("client_id");
            lease = true;
            ++acquisitions;
            reply(response, {{"token", "test-token"}, {"client_id", client_id}});
        });
        http.Post("/api/control/release", [&](const auto& request, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!authorized(request)) {
                reply(response, nullptr, false, 3007);
                return;
            }
            lease = false;
            ++releases;
            reply(response, Json::object());
        });
        http.Post("/api/robot/rci/enable", [&](const auto& request, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!authorized(request)) {
                reply(response, nullptr, false, 3006);
                return;
            }
            rci_enabled = true;
            robot_state = "RCI";
            reply(response, status());
        });
        http.Post("/api/robot/rci/disable", [&](const auto& request, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!authorized(request)) {
                reply(response, nullptr, false, 3006);
                return;
            }
            if (active) {
                closing = true;
                requestStop();
            } else {
                rci_enabled = false;
                robot_state = "STOPPED";
            }
            reply(response, status());
        });
        http.Post("/api/robot/rci/start", [&](const auto& request, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!rci_enabled) {
                reply(response, status(), false, -1018);
                return;
            }
            if (active || reject_start) {
                reply(response, status(), false, -2215);
                return;
            }
            const auto body = Json::parse(request.body);
            if (body.at("mode") != "joint" || body.at("port") != options.udp_port) {
                reply(response, nullptr, false, 1001);
                return;
            }
            ++starts;
            state.session_id.fill(static_cast<std::uint8_t>(starts.load()));
            state.message_id = state.received_message_id = state.applied_message_id = 0;
            state.cycle = state.sample_time_ns = 0;
            peer_ready = false;
            command_cycles.clear();
            state.q_d = state.q;
            active = true;
            robot_state = "SERVOING";
            std::string session;
            const char* hex = "0123456789abcdef";
            for (auto byte : state.session_id) {
                session += hex[byte >> 4];
                session += hex[byte & 15];
            }
            auto data = status();
            data["port"] = options.udp_port;
            data["protocol_version"] = protocol_version;
            data["session_id"] = start_session_override.empty() ? session : start_session_override;
            reply(response, data);
        });
        http.Post("/api/robot/rci/stop", [&](const auto&, auto& response) {
            std::lock_guard<std::mutex> lock(mutex);
            ++stops;
            requestStop();
            reply(response, status());
        });
        options.http_port = http.bind_to_any_port(bind_host);
        REQUIRE(options.http_port > 0);
        http_thread = std::thread([&] { http.listen_after_bind(); });
        http.wait_until_ready();
        udp_thread = std::thread([&] { udpLoop(); });
    }
    ~Peer() {
        running = false;
        udp_thread.join();
        ::close(socket_fd);
        http.stop();
        http_thread.join();
    }
    static void reply(httplib::Response& response, const Json& data, bool success = true,
                      int code = 0) {
        response.set_content(
            Json{{"success", success}, {"code", code}, {"message", "test response"}, {"data", data}}
                .dump(),
            "application/json");
    }
    bool authorized(const httplib::Request& request) const {
        return lease && request.get_header_value("X-Rocos-Control-Token") == "test-token" &&
               request.get_header_value("X-Rocos-Client-Id") == client_id;
    }
    Json diagnostic() const {
        return {{"cycle", state.cycle},
                {"sample_time_ns", state.sample_time_ns},
                {"source_time_us", state.source_time_us},
                {"received_message_id", state.received_message_id},
                {"applied_message_id", state.applied_message_id},
                {"control_cycles", state.cycle},
                {"elapsed_periods", state.cycle},
                {"fault_code", fault.load()},
                {"errors", state.errors},
                {"q", state.q},
                {"dq", state.dq},
                {"q_d", state.q_d},
                {"dq_d", state.dq_d},
                {"ddq_d", state.ddq_d},
                {"q_cmd", state.q_cmd},
                {"tau_cmd", state.tau_cmd},
                {"output_mode", 8},
                {"output_valid", true},
                {"control_command_success_rate", 1.0}};
    }
    Json status() const {
        Json data = {{"enabled", rci_enabled},
                     {"closing", closing},
                     {"control_active", active},
                     {"robot_state", robot_state},
                     {"last_control", starts ? diagnostic() : Json()}};
        if (active) data["current_control"] = diagnostic();
        return data;
    }
    void requestStop() {
        robot_state = "STOPPING";
        stopping_at = Clock::now();
    }
    void advanceStop() {
        if (robot_state == "STOPPING" && !stuck_stop &&
            Clock::now() - stopping_at >= std::chrono::milliseconds(3)) {
            active = false;
            robot_state = fault ? "ERROR_STATE" : closing ? "STOPPED" : "RCI";
            if (closing || fault) {
                rci_enabled = false;
                closing = false;
            }
        }
    }
    void udpLoop() {
        auto next_cycle = Clock::now();
        while (running) {
            // 独立控制时基：持续推送状态，不依赖收到下一条指令。
            const auto now = Clock::now();
            if (now < next_cycle) {
                pollfd descriptor{socket_fd, POLLIN, 0};
                const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(next_cycle-now).count();
                const timespec wait{ns / 1000000000, ns % 1000000000};
                ::ppoll(&descriptor, 1, &wait, nullptr);
            }
            std::lock_guard<std::mutex> lock(mutex);
            for (int attempt = 0; attempt < 64; ++attempt) {
                MotionGeneratorCommand command{};
                sockaddr_in source{};
                socklen_t length = sizeof(source);
                const auto size = ::recvfrom(socket_fd, &command, sizeof(command), 0,
                                             reinterpret_cast<sockaddr*>(&source), &length);
                if (size < 0) break;
                if (size != sizeof(command) || !active || command.session_id != state.session_id ||
                    command.protocol_version != SERVO_PROTOCOL_VERSION) continue;
                peer_address = source;
                peer_ready = true;
                if (command.message_id > state.received_message_id) {
                    if (!commands) first_q = command.q_c;
                    ++commands;
                    command_cycles.push_back(state.cycle);
                    state.received_message_id = command.message_id;
                    if (!stale_ack) state.applied_message_id = command.message_id;
                    state.q = state.q_d = state.q_cmd = command.q_c;
                }
            }
            if (Clock::now() < next_cycle) continue;
            next_cycle = Clock::now() + std::chrono::microseconds(cycle_period_us.load());
            if (!active) continue;
            if (!freeze) {
                ++state.cycle;
                state.sample_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           Clock::now().time_since_epoch()).count() + clock_offset_ns.load();
            }
            if (!peer_ready || drop || state.cycle == static_cast<uint64_t>(skip_cycle.load())) continue;
            auto packet = state;
            // 运动中报告位置滞后，起点及回到零位时保持精确反馈。
            if (packet.applied_message_id && packet.q_cmd[2] != 0)
                packet.q[2] -= reported_position_lag.load();
            packet.message_id = ++state.message_id;
            packet.errors[0] = fault != 0;
            if (wrong_session) packet.session_id[0] ^= 0x80;
            if (nonfinite) packet.q[0] = std::numeric_limits<double>::quiet_NaN();
            std::array<char, sizeof(packet) + 1> bytes{};
            std::memcpy(bytes.data(), &packet, sizeof(packet));
            ::sendto(socket_fd, bytes.data(), sizeof(packet) + (oversize ? 1 : 0), 0,
                     reinterpret_cast<sockaddr*>(&peer_address), sizeof(peer_address));
        }
    }
};

librocos::JointPositions hold(const librocos::RobotState& state) {
    return librocos::JointPositions(state.q_d, state.dof);
}
}  // namespace

TEST_CASE("librocos HTTP read preserves all-zero joint count and separates reference validity") {
    Peer peer;
    librocos::Robot robot("127.0.0.1", peer.options);
    const auto state = robot.readOnce();
    CHECK(state.dof == 7);
    CHECK(state.q == librocos::JointArray{});
    CHECK_FALSE(state.reference_valid);
    CHECK(state.source == librocos::RobotState::Source::Http);
    int samples = 0;
    robot.read([&](const auto&) { return ++samples < 3; });
    CHECK(samples == 3);
    CHECK(peer.starts == 0);
}

TEST_CASE("librocos program RCI enable releases HTTP lease and can disable") {
    Peer peer;
    librocos::Robot robot("127.0.0.1", peer.options);
    robot.enableRci();
    CHECK(robot.rciStatus().enabled);
    robot.enableRci();
    CHECK(peer.acquisitions == 1);
    CHECK(peer.releases == 1);
    robot.disableRci();
    CHECK_FALSE(robot.rciStatus().enabled);
    CHECK(peer.acquisitions == 2);
    CHECK(peer.releases == 2);
}

TEST_CASE(
    "librocos joint callback uses fresh samples and confirms final command and stop across "
    "sessions") {
    Peer peer;
    peer.rci_enabled = true;  // 上位机已经开启。
    librocos::Robot robot("localhost", peer.options);
    robot.enableRci();
    CHECK(peer.acquisitions == 0);
    for (int run = 0; run < 4; ++run) {
        std::uint64_t previous = 0;
        int samples = 0;
        robot.control([&](const auto& state, librocos::Duration period) {
            CHECK(state.dof == 7);
            CHECK(state.reference_valid);
            CHECK(state.sample_time_ns > previous);
            CHECK(period.toNSec() == (previous ? state.sample_time_ns - previous : 0));
            previous = state.sample_time_ns;
            auto command = hold(state);
            command.q[0] += 0.00001;
            return ++samples == 60 ? librocos::MotionFinished(command) : command;
        });
        const auto status = robot.rciStatus();
        CHECK_FALSE(status.control_active);
        CHECK(status.robot_state == "RCI");
        REQUIRE(status.last_control);
        CHECK(status.last_control->applied_message_id > 0);
        CHECK(samples == 60);
    }
    CHECK(peer.starts == 4);
    CHECK(peer.stops == 4);
}

TEST_CASE("librocos rejected session never stops existing owner") {
    Peer peer;
    peer.rci_enabled = true;
    peer.reject_start = true;
    {
        librocos::Robot robot("127.0.0.1", peer.options);
        CHECK_THROWS_AS(robot.control([](const auto& s, auto) { return hold(s); }),
                        librocos::ControlException);
    }
    CHECK(peer.stops == 0);
}

TEST_CASE("librocos IPv4 control tolerates independent host clocks and rejects stale application acknowledgements") {
    Peer peer({}, Json::array(), "127.0.0.2");
    bool freeze_after_start = false;
    SUBCASE("server monotonic clock ahead") { peer.clock_offset_ns = 30LL * 24 * 3600 * 1000000000; }
    SUBCASE("server monotonic clock behind") { peer.clock_offset_ns = -3600LL * 1000000000; }
    SUBCASE("fresh states with stuck applied command") { peer.stale_ack = true; }
    SUBCASE("application acknowledgement stops advancing") { freeze_after_start = true; }
    peer.rci_enabled = true;
    librocos::Robot robot("127.0.0.2", peer.options);
    int calls = 0;
    const auto control = [&] {
        robot.control([&](const auto& state, auto) {
            auto command = hold(state);
            if (freeze_after_start && calls == 5) peer.stale_ack = true;
            return ++calls == 80 ? librocos::MotionFinished(command) : command;
        });
    };
    if (peer.stale_ack || freeze_after_start) CHECK_THROWS_AS(control(), librocos::NetworkException);
    else { CHECK_NOTHROW(control()); CHECK(calls == 80); }
    CHECK_FALSE(peer.active);
    CHECK(peer.stops == 1);
}

TEST_CASE("librocos validates session before sending motion and cleans up failed startup") {
    Peer peer;
    peer.rci_enabled = true;
    SUBCASE("bad session encoding") { peer.start_session_override = std::string(32, 'g'); }
    SUBCASE("unsupported protocol") { peer.protocol_version = 3; }
    librocos::Robot robot("127.0.0.1", peer.options);
    CHECK_THROWS_AS(robot.control([](const auto& s, auto) { return hold(s); }),
                    librocos::ProtocolException);
    CHECK(peer.commands == 0);
    CHECK(peer.stops == 1);
    CHECK_FALSE(robot.rciStatus().control_active);
}

TEST_CASE("librocos malformed or missing feedback cannot keep a session alive") {
    Peer peer;
    peer.rci_enabled = true;
    SUBCASE("wrong session") { peer.wrong_session = true; }
    SUBCASE("oversized packet") { peer.oversize = true; }
    SUBCASE("no packets") { peer.drop = true; }
    librocos::Robot robot("127.0.0.1", peer.options);
    int calls = 0;
    CHECK_THROWS_AS(robot.control([&](const auto& s, auto) {
        ++calls;
        return hold(s);
    }),
                    librocos::NetworkException);
    CHECK(calls == 0);
    CHECK(peer.stops == 1);
}

TEST_CASE("librocos rejects frozen feedback even when replies continue") {
    Peer peer;
    peer.rci_enabled = true;
    librocos::Robot robot("127.0.0.1", peer.options);
    int calls = 0;
    CHECK_THROWS_AS(robot.control([&](const auto& s, auto) {
        if (++calls == 5) peer.freeze = true;
        return hold(s);
    }),
                    librocos::NetworkException);
    CHECK(calls < 8);
    CHECK(peer.commands == calls); // 冻结状态期间不能重发保持目标并生成新序号。
    CHECK(peer.stops == 1);
}

TEST_CASE("librocos callback and command failures stop before propagating") {
    Peer peer;
    peer.rci_enabled = true;
    librocos::Robot robot("127.0.0.1", peer.options);
    SUBCASE("callback exception") {
        CHECK_THROWS_WITH(robot.control([](const auto&, auto) -> librocos::JointPositions {
            throw std::runtime_error("callback failed");
        }),
                          "callback failed");
    }
    SUBCASE("dimension mismatch") {
        CHECK_THROWS_AS(robot.control([](const auto&, auto) {
            return librocos::JointPositions{0, 0};
        }),
                        librocos::ControlException);
    }
    SUBCASE("nonfinite command") {
        CHECK_THROWS_AS(robot.control([](const auto& s, auto) {
            auto q = hold(s);
            q.q[0] = std::numeric_limits<double>::infinity();
            return q;
        }),
                        librocos::ControlException);
    }
    SUBCASE("slow callback") {
        CHECK_THROWS_AS(robot.control([](const auto& s, auto) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return hold(s);
        }),
                        librocos::ControlException);
    }
    SUBCASE("nonfinite state") {
        peer.nonfinite = true;
        CHECK_THROWS_AS(robot.control([](const auto& s, auto) { return hold(s); }),
                        librocos::ProtocolException);
    }
    CHECK(peer.commands == 0);
    CHECK(peer.stops == 1);
    CHECK_FALSE(robot.rciStatus().control_active);
}

TEST_CASE("librocos exposes terminal controller fault and snapshot") {
    Peer peer;
    peer.rci_enabled = true;
    librocos::Robot robot("127.0.0.1", peer.options);
    try {
        robot.control([&](const auto& s, auto) {
            peer.fault = -2221;
            return hold(s);
        });
        FAIL("Expected controller fault");
    } catch (const librocos::ControlException& error) {
        CHECK(error.code() == -2221);
        REQUIRE(error.status);
        CHECK(error.status->robot_state == "ERROR_STATE");
    }
    CHECK(peer.stops == 1);
}

TEST_CASE("librocos concurrent stop interrupts control and rejects nested operations") {
    Peer peer;
    peer.rci_enabled = true;
    librocos::Robot robot("127.0.0.1", peer.options);
    std::promise<void> entered;
    auto ready = entered.get_future();
    std::exception_ptr error;
    std::thread control([&] {
        try {
            bool first = true;
            robot.control([&](const auto& s, auto) {
                if (first) {
                    first = false;
                    entered.set_value();
                }
                return hold(s);
            });
        } catch (...) {
            error = std::current_exception();
        }
    });
    CHECK(ready.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    CHECK_THROWS_AS(robot.enableRci(), librocos::InvalidOperationException);
    robot.stop();
    control.join();
    CHECK_FALSE(error);
    CHECK_FALSE(robot.rciStatus().control_active);
}

TEST_CASE("librocos stop timeout remains recoverable and preserves callback exception") {
    Peer peer;
    peer.rci_enabled = true;
    peer.stuck_stop = true;
    librocos::Robot robot("127.0.0.1", peer.options);
    try {
        robot.control([](const auto&, auto) -> librocos::JointPositions {
            throw std::runtime_error("cause");
        });
        FAIL("Expected stop timeout");
    } catch (const librocos::ControlException& error) {
        REQUIRE(error.cause);
        CHECK_THROWS_WITH(std::rethrow_exception(error.cause), "cause");
    }
    CHECK_THROWS_AS(robot.control([](const auto& s, auto) { return hold(s); }),
                    librocos::InvalidOperationException);
    peer.stuck_stop = false;
    robot.stop();
    CHECK_FALSE(robot.rciStatus().control_active);
}

TEST_CASE("librocos position-only mode rejects impedance before opening session") {
    Peer peer;
    peer.rci_enabled = true;
    peer.work_mode = "jnt_imp";
    librocos::Robot robot("127.0.0.1", peer.options);
    CHECK_THROWS_AS(robot.control([](const auto& s, auto) { return hold(s); }),
                    librocos::InvalidOperationException);
    CHECK(peer.starts == 0);
}

TEST_CASE("librocos failed control acquisition preserves the existing HTTP owner") {
    Peer peer;
    peer.lease = true;
    librocos::Robot robot("127.0.0.1", peer.options);
    CHECK_THROWS_AS(robot.enableRci(), librocos::ControlException);
    CHECK(peer.releases == 0);
    CHECK(peer.acquisitions == 0);
    CHECK(peer.stops == 0);
}

TEST_CASE("librocos fault snapshots retain nonfinite feedback without hiding the terminal state") {
    Peer peer;
    {
        std::lock_guard<std::mutex> lock(peer.mutex);
        peer.starts = 1;
        peer.fault = -2034;
        peer.robot_state = "ERROR_STATE";
        peer.state.q[0] = std::numeric_limits<double>::quiet_NaN();
    }
    librocos::Robot robot("127.0.0.1", peer.options);
    const auto status = robot.rciStatus();
    REQUIRE(status.last_control);
    CHECK(status.last_control->fault_code == -2034);
    CHECK(std::isnan(status.last_control->q[0]));
    CHECK(status.robot_state == "ERROR_STATE");
}

#ifdef LIBROCOS_EXAMPLE_DIR
TEST_CASE("J6 J7 sine is bounded and C3 at amplitude ramp connections") {
    joint_sine::Parameters p;
    const auto bound = joint_sine::bounds(p);
    CHECK_NOTHROW(joint_sine::checkTrajectory(p, {-2.11, 2.11, 3, 10, 10000, .06}));
    for (double frequency : {.2, .23}) {
        p.frequency = frequency;
        const auto peak = joint_sine::bounds(p);
        for (int k = 0; k <= 180000; ++k) {
            const auto value = joint_sine::evaluate(p, k * .001);
            REQUIRE(std::abs(value.q) <= peak.q + 1e-12);
            REQUIRE(std::abs(value.v) <= peak.v + 1e-12);
            REQUIRE(std::abs(value.a) <= peak.a + 1e-12);
            REQUIRE(std::abs(value.j) <= peak.j + 1e-12);
        }
        for (double t : {0., p.ramp, p.duration - p.ramp, p.duration}) {
            const auto left = joint_sine::evaluate(p, t - 1e-7);
            const auto right = joint_sine::evaluate(p, t + 1e-7);
            CHECK(std::abs(left.q - right.q) < 1e-5);
            CHECK(std::abs(left.v - right.v) < 1e-5);
            CHECK(std::abs(left.a - right.a) < 1e-5);
            CHECK(std::abs(left.j - right.j) < 1e-5);
        }
    }
    p.frequency = 10;
    CHECK_THROWS(joint_sine::checkTrajectory(p, {-2.11, 2.11, 3, 10, 10000, .06}));
    p.frequency = .2;
    CHECK_THROWS(joint_sine::checkTrajectory(p, {-.5, .5, 3, 10, 10000, .06}));
    p.frequency = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(p.validate());
    CHECK(bound.v < 3);
    CHECK(bound.a < 10);
    CHECK(bound.j < 10000);
}

TEST_CASE("J6 J7 sine example validates startup and records normal interrupted and failed sessions") {
    namespace fs = std::filesystem;
    const auto root = fs::path(LIBROCOS_SOURCE_ROOT);
    const auto robot_dir = root / "config/robots/talon";
    auto hardware = YAML::LoadFile((robot_dir / "hardware.yaml").string())["hardware"];
    Json info = Json::array();
    for (const auto& drive : hardware) {
        const auto transform = drive["transform"];
        info.push_back({{"id", drive["id"].as<int>()}, {"name", drive["joint_name"].as<std::string>()},
                        {"pos_per_unit", transform["pos_per_unit"].as<double>()},
                        {"ratio", transform["ratio"].as<double>()}, {"unit_name", "rad"},
                        {"zero_offset", transform["offset_pos_cnt"].as<int>()}});
    }
    std::ifstream model_file(robot_dir / "robot.urdf");
    const std::string model((std::istreambuf_iterator<char>(model_file)), {});
    Peer peer(model, info, "127.0.0.2");
    {
        std::lock_guard<std::mutex> lock(peer.mutex);
        for (std::size_t i = 0; i < 5; ++i) peer.state.q[i] = .1 * (i + 1);
        peer.state.q_cmd = peer.state.q;
        peer.state.output_mode = 8;
    }
    const auto directory = fs::temp_directory_path() / ("librocos-sine-" + std::to_string(getpid()));
    REQUIRE(fs::create_directory(directory));
    struct Cleanup { fs::path path; ~Cleanup() { fs::remove_all(path); } } cleanup{directory};
    YAML::Node config;
    config["software"]["http_port"] = peer.options.http_port;
    auto robot = config["robot"];
    robot["name"] = "talon"; robot["base_link"] = "base_link"; robot["tip_link"] = "link_7";
    robot["urdf"] = (robot_dir / "robot.urdf").string();
    robot["hardware"] = hardware;
    robot["master"]["cycle"] = 1000;
    robot["servo"]["communication_guard_enabled"] = true;
    robot["servo"]["rate_limit_enabled"] = true;
    std::ofstream(directory / "effective.yaml") << config;

    auto run = [&](const std::string& name, bool interrupt, bool drop, bool check,
                   bool realtime = false, bool all_joints = false) {
        const auto out = directory / name;
        const std::string executable = std::string(LIBROCOS_EXAMPLE_DIR) + "/librocos_joint_6_7_sine";
        std::vector<std::string> arguments{executable, (directory / "effective.yaml").string(),
            out.string(), "--ip", "127.0.0.2", "--duration", "10", "--udp-port", std::to_string(peer.options.udp_port)};
        if (check) arguments.insert(arguments.end(), {"--check", "--frequency", "10"});
        if (realtime) arguments.push_back("--realtime");
        if (all_joints) arguments.push_back("--all-joints");
        std::vector<char*> argv;
        for (auto& argument : arguments) argv.push_back(argument.data());
        argv.push_back(nullptr);
        int input[2]; REQUIRE(pipe(input) == 0);
        REQUIRE(::write(input[1], "\n", 1) == 1); ::close(input[1]);
        posix_spawn_file_actions_t actions;
        REQUIRE(posix_spawn_file_actions_init(&actions) == 0);
        REQUIRE(posix_spawn_file_actions_adddup2(&actions, input[0], STDIN_FILENO) == 0);
        REQUIRE(posix_spawn_file_actions_addclose(&actions, input[0]) == 0);
        pid_t child{};
        const auto spawned = posix_spawn(&child, executable.c_str(), &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions); ::close(input[0]);
        REQUIRE(spawned == 0);
        const auto started = Clock::now();
        bool injected = false, finished = false;
        int status = 0;
        while (Clock::now() - started < std::chrono::seconds(20)) {
            if (waitpid(child, &status, WNOHANG) == child) { finished = true; break; }
            if (!injected && Clock::now() - started > std::chrono::milliseconds(500)) {
                if (interrupt) ::kill(child, SIGINT);
                if (drop) peer.drop = true;
                injected = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        peer.drop = false;
        if (!finished) { ::kill(child, SIGKILL); waitpid(child, &status, 0); }
        REQUIRE(finished);
        REQUIRE(WIFEXITED(status));
        return std::make_pair(WEXITSTATUS(status), out);
    };
    const auto invalid = run("invalid", false, false, true);
    CHECK(invalid.first != 0);
    CHECK(peer.starts == 0);
    {
        std::lock_guard<std::mutex> lock(peer.mutex);
        peer.state.q[5] = .1;
    }
    CHECK(run("off_zero", false, false, false).first != 0);
    CHECK(peer.starts == 0);
    {
        std::lock_guard<std::mutex> lock(peer.mutex);
        peer.state.q[5] = 0;
    }
    rocos::RealtimeThreadStatus scheduling;
    std::thread probe([&] { scheduling = rocos::ConfigureRealtimeThread(30); });
    probe.join();
    const auto realtime = run("realtime", true, false, false, true);
    CHECK(realtime.first == 1);
    std::ifstream realtime_file(realtime.second / "outcome.json");
    REQUIRE(realtime_file.good());
    const auto realtime_outcome = Json::parse(realtime_file);
    CHECK(realtime_outcome.contains("client_scheduling"));
    if (scheduling.ready()) {
        CHECK(peer.starts == 1);
        CHECK(realtime_outcome.at("interrupted").get<bool>());
        CHECK(realtime_outcome.at("samples").get<int>() > 10);
    } else {
        CHECK(peer.starts == 0);
        CHECK(realtime_outcome.at("samples").get<int>() == 0);
        CHECK(realtime_outcome.at("error").get<std::string>().find("Realtime scheduling unavailable") != std::string::npos);
    }
    {
        std::lock_guard<std::mutex> lock(peer.mutex);
        peer.state.q[5] = peer.state.q[6] = 0;
        peer.state.q_cmd = peer.state.q;
    }
    for (const auto& name : {"normal", "interrupt", "drop", "all_joints"}) {
        const bool all_joints = std::string(name) == "all_joints";
        if (all_joints) {
            {
                std::lock_guard<std::mutex> lock(peer.mutex);
                peer.state.q.fill(0);
                peer.state.q[3] = .1;
            }
            const auto starts_before = peer.starts.load();
            CHECK(run("all_joints_off_zero", false, false, false, false, true).first != 0);
            CHECK(peer.starts == starts_before);
            std::lock_guard<std::mutex> lock(peer.mutex);
            peer.state.q.fill(0);
            peer.state.q_cmd = peer.state.q;
        }
        peer.reported_position_lag = all_joints ? .1 : 0;
        const bool normal = std::string(name) == "normal" || all_joints;
        const auto result = run(name, std::string(name) == "interrupt", std::string(name) == "drop", false, false, all_joints);
        INFO(name);
        CHECK(result.first == (normal ? 0 : 1));
        std::ifstream outcome_file(result.second / "outcome.json");
        REQUIRE(outcome_file.good());
        const auto outcome = Json::parse(outcome_file);
        CHECK(outcome.at("samples").get<int>() > 10);
        CHECK_FALSE(outcome.at("writer_failed").get<bool>());
        CHECK(outcome.at("robot_state") == "RCI");
        CHECK_FALSE(outcome.at("control_active").get<bool>());
        CHECK(outcome.at("completed").get<bool>() == normal);
        std::ifstream metadata_file(result.second / "metadata.json");
        const auto metadata = Json::parse(metadata_file);
        CHECK(metadata.at("controller_ip") == "127.0.0.2");
        CHECK_FALSE(metadata.at("client_position_following_protection_enabled").get<bool>());
        CHECK(metadata.at("trajectory_time_basis") == "control_cycle");
        CHECK(fs::file_size(result.second / "samples.csv") > 1000);
        std::ifstream csv(result.second / "samples.csv");
        std::string line;
        std::getline(csv, line);
        std::size_t rows = 0;
        double first_cycle = 0;
        double largest_clock_difference = 0;
        double first_sample_ns = 0;
        double peak_position_error = 0;
        joint_sine::Parameters trajectory;
        trajectory.duration = 10;
        while (std::getline(csv, line)) {
            std::istringstream stream(line);
            std::vector<double> values;
            std::string cell;
            while (std::getline(stream, cell, ',')) values.push_back(std::stod(cell));
            REQUIRE(values.size() == 65);
            peak_position_error = std::max(peak_position_error, std::abs(values[18] - values[32]));
            if (rows == 0) { first_cycle = values[2]; first_sample_ns = values[3]; }
            CHECK(std::abs(values[0] - (values[2] - first_cycle) * .001) < 1e-12);
            largest_clock_difference = std::max(largest_clock_difference,
                std::abs(values[0] - (values[3] - first_sample_ns) * 1e-9));
            if (all_joints) {
                const double shape = joint_sine::evaluate(trajectory, values[0]).q / trajectory.amplitude;
                for (std::size_t i = 0; i < 7; ++i) {
                    const double degrees = (i == 3 || i >= 5) ? 50 : 10;
                    CHECK(std::abs(values[9 + i] - degrees * joint_sine::kPi / 180 * shape) < 1e-12);
                    CHECK(metadata.at("joint_amplitudes_rad").at(i).get<double>() == degrees * joint_sine::kPi / 180);
                }
            } else {
                for (std::size_t i = 0; i < 5; ++i)
                    CHECK(values[9 + i] == .1 * (i + 1));
            }
            CHECK(values[14] == values[15]);
            CHECK(std::abs(values[14]) <= joint_sine::kPi / 3 + 1e-12);
            ++rows;
        }
        CHECK(rows == outcome.at("samples").get<std::size_t>());
        CHECK(largest_clock_difference > 1e-6);
        if (all_joints) CHECK(peak_position_error > .06);
        peer.reported_position_lag = 0;
        std::lock_guard<std::mutex> lock(peer.mutex);
        if (all_joints) {
            for (std::size_t i = 0; i < 7; ++i) CHECK(std::abs(peer.state.q[i]) < 1e-12);
        } else {
            for (std::size_t i = 0; i < 5; ++i) CHECK(peer.state.q[i] == .1 * (i + 1));
        }
        peer.state.q[5] = peer.state.q[6] = 0;
        peer.state.q_cmd = peer.state.q;
    }
}

TEST_CASE("librocos example executables complete against a local protocol peer") {
    Peer peer;
    for (const auto* name :
         {"echo_robot_state", "generate_joint_position_motion", "joint_point_to_point_motion",
          "generate_consecutive_motions", "communication_test"}) {
        int input[2];
        REQUIRE(pipe(input) == 0);
        REQUIRE(::write(input[1], "\n", 1) == 1);
        ::close(input[1]);
        const std::string program = std::string(LIBROCOS_EXAMPLE_DIR) + "/librocos_" + name;
        std::vector<std::string> arguments{program,
                                           "127.0.0.1",
                                           std::to_string(peer.options.http_port),
                                           std::to_string(peer.options.udp_port),
                                           "0",
                                           "0.001",
                                           "0.05"};
        std::vector<char*> argv;
        for (auto& argument : arguments) argv.push_back(argument.data());
        argv.push_back(nullptr);
        posix_spawn_file_actions_t actions;
        REQUIRE(posix_spawn_file_actions_init(&actions) == 0);
        REQUIRE(posix_spawn_file_actions_adddup2(&actions, input[0], STDIN_FILENO) == 0);
        REQUIRE(posix_spawn_file_actions_addclose(&actions, input[0]) == 0);
        pid_t child{};
        const auto spawned =
            posix_spawn(&child, program.c_str(), &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);
        ::close(input[0]);
        REQUIRE(spawned == 0);
        if (std::string(name) == "echo_robot_state") {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ::kill(child, SIGINT);
        }
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        int status = 0;
        bool finished = false;
        while (Clock::now() < deadline) {
            if (waitpid(child, &status, WNOHANG) == child) {
                finished = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (!finished) {
            ::kill(child, SIGKILL);
            waitpid(child, &status, 0);
        }
        INFO("example=" << name);
        CHECK(finished);
        CHECK(WIFEXITED(status));
        CHECK(WEXITSTATUS(status) == 0);
    }
}
#endif

TEST_CASE("librocos follows server cycles despite a different nominal local period") {
    Peer peer;
    peer.rci_enabled = true;
    peer.cycle_period_us = 5000;
    auto options = peer.options;
    options.control_period = std::chrono::microseconds(500);
    peer.skip_cycle = 5; // 丢一个状态后，下周期仍自主推送，不能形成请求/回复死锁。
    librocos::Robot robot("127.0.0.1", options);
    std::vector<uint64_t> cycles;
    robot.control([&](const auto& state, auto) {
        cycles.push_back(state.cycle);
        auto q = hold(state);
        return cycles.size() == 12 ? librocos::MotionFinished(q) : q;
    });
    REQUIRE(cycles.size() == 12);
    CHECK(std::find(cycles.begin(), cycles.end(), 5) == cycles.end());
    for (size_t i = 1; i < cycles.size(); ++i) CHECK(cycles[i] > cycles[i-1]);
    CHECK(peer.commands == 12);
    CHECK(peer.stops == 1);
}
