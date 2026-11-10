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
#include <thread>
#include <vector>

#include "src/servo_type.hpp"
#include "test/doctest.h"

extern char** environ;

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

    Peer() {
        options.http_timeout = std::chrono::milliseconds(300);
        options.startup_timeout = std::chrono::milliseconds(100);
        options.receive_timeout = std::chrono::milliseconds(40);
        options.stop_timeout = std::chrono::milliseconds(150);
        options.read_period = std::chrono::milliseconds(1);
        socket_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        REQUIRE(socket_fd >= 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        REQUIRE(::bind(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
        socklen_t length = sizeof(address);
        REQUIRE(::getsockname(socket_fd, reinterpret_cast<sockaddr*>(&address), &length) == 0);
        options.udp_port = ntohs(address.sin_port);
        state.mode = MotionMode::kJointPosition;
        state.output_valid = true;
        state.control_command_success_rate = 1;
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
            } else
                rci_enabled = false;
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
        options.http_port = http.bind_to_any_port("127.0.0.1");
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
            robot_state = fault ? "ERROR_STATE" : "STOPPED";
            if (closing) {
                rci_enabled = false;
                closing = false;
            }
        }
    }
    void udpLoop() {
        while (running) {
            pollfd descriptor{socket_fd, POLLIN, 0};
            ::poll(&descriptor, 1, 1);
            std::lock_guard<std::mutex> lock(mutex);
            if (!freeze) {
                ++state.cycle;
                state.sample_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           Clock::now().time_since_epoch())
                                           .count();
            }
            MotionGeneratorCommand command{};
            sockaddr_in source{};
            socklen_t length = sizeof(source);
            const auto size = ::recvfrom(socket_fd, &command, sizeof(command), 0,
                                         reinterpret_cast<sockaddr*>(&source), &length);
            if (size != sizeof(command) || !active || command.session_id != state.session_id ||
                command.protocol_version != SERVO_PROTOCOL_VERSION)
                continue;
            if (command.message_id > state.received_message_id) {
                if (!commands) first_q = command.q_c;
                ++commands;
                state.received_message_id = state.applied_message_id = command.message_id;
                state.q = state.q_d = state.q_cmd = command.q_c;
            }
            if (drop) continue;
            auto packet = state;
            packet.message_id = ++state.message_id;
            packet.errors[0] = fault != 0;
            if (wrong_session) packet.session_id[0] ^= 0x80;
            if (nonfinite) packet.q[0] = std::numeric_limits<double>::quiet_NaN();
            std::array<char, sizeof(packet) + 1> bytes{};
            std::memcpy(bytes.data(), &packet, sizeof(packet));
            ::sendto(socket_fd, bytes.data(), sizeof(packet) + (oversize ? 1 : 0), 0,
                     reinterpret_cast<sockaddr*>(&source), length);
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
        CHECK(status.robot_state == "STOPPED");
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
