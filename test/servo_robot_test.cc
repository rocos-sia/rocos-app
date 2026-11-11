#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <sockpp/udp_socket.h>
#include <librocos/robot.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <thread>

#include "src/robot_http_server.hpp"
#include "src/lua_script_engine.hpp"
#include "src/servo_type.hpp"
#include "src/move_servo.hpp"
#include "test/doctest.h"

namespace rocos {
struct RobotHttpServerTestAccess {
    static nlohmann::json startFrom(RobotHttpServer& server, const std::string& address, int port) {
        httplib::Request request;
        request.remote_addr = address;
        request.body = nlohmann::json{{"port", port}}.dump();
        httplib::Response response;
        server.handleRciStart(request, response);
        return nlohmann::json::parse(response.body);
    }
    static void expire(RobotHttpServer& server) {
        std::lock_guard<std::mutex> lock(server.control_mutex_);
        server.last_seen_at_ = std::chrono::steady_clock::now() - std::chrono::hours(1);
    }
    static int start(RobotHttpServer& server, const std::string& host = "127.0.0.1") {
        const int port = server.server_->bind_to_any_port(host);
        REQUIRE(port > 0);
        server.thread_ = std::make_unique<std::thread>(
            [&server] { server.server_->listen_after_bind(); });
        server.server_->wait_until_ready();
        return port;
    }
};
}  // namespace rocos

namespace {
using rocos::Result;
using Json = nlohmann::json;

// 独立共享内存主站，仅模拟测试模型的两个驱动；不连接真实 EtherCAT。
struct OfflineMaster {
    static int nextId() {
        static int id = 800000 + getpid() * 10;
        return id++;
    }
    const int id{nextId()};
    rocos::SharedMemoryConfig master{id};
    std::atomic<bool> running{true};
    std::atomic<bool> follow_targets{false};
    std::thread thread;
    OfflineMaster() {
        REQUIRE(master.createSharedMemory());
        REQUIRE(master.createPdDataMemoryProvider(32, 32));
        auto& bus = *master.ecatBus;
        bus.slave_num = 2;
        bus.dt = 1000;
        bus.current_state = 8;
        for (int i = 0; i < 2; ++i) {
            auto& slave = bus.slaves[i];
            slave.id = i;
            slave.input_var_num = slave.output_var_num = 4;
            descriptor(slave.input_vars[0], "status", i * 16, 2);
            descriptor(slave.input_vars[1], "position", i * 16 + 2, 4);
            descriptor(slave.input_vars[2], "velocity", i * 16 + 6, 4);
            descriptor(slave.input_vars[3], "torque", i * 16 + 10, 2);
            descriptor(slave.output_vars[0], "control", i * 16, 2);
            descriptor(slave.output_vars[1], "mode", i * 16 + 2, 1);
            descriptor(slave.output_vars[2], "target", i * 16 + 3, 4);
            descriptor(slave.output_vars[3], "target_torque", i * 16 + 7, 2);
            master.setSlaveInputVarValue<uint16_t>(i, 0, 0x40);
            master.setSlaveInputVarValue<int32_t>(i, 1, i ? -500 : 1000);
            master.setSlaveInputVarValue<int32_t>(i, 2, 0);
            master.setSlaveInputVarValue<int16_t>(i, 3, 0);
        }
        REQUIRE(master.publishReady());
        thread = std::thread([this] {
            while (running.load()) {
                for (int i = 0; i < 2; ++i) {
                    const auto control =
                        master.getSlaveOutputVarValue<uint16_t>(i, 0);
                    uint16_t status = 0x40;
                    if ((control & 0x8f) == 0x06) status = 0x21;
                    if ((control & 0x8f) == 0x07) status = 0x23;
                    if ((control & 0x8f) == 0x0f) status = 0x27;
                    master.setSlaveInputVarValue<uint16_t>(i, 0, status);
                    if (follow_targets && status == 0x27) {
                        const auto previous = master.getSlaveInputVarValue<int32_t>(i, 1);
                        const auto target = master.getSlaveOutputVarValue<int32_t>(i, 2);
                        master.setSlaveInputVarValue<int32_t>(i, 1, target);
                        master.setSlaveInputVarValue<int32_t>(i, 2, (target - previous) * 1000);
                    }
                }
                busTimestamp();
                master.notifyClients();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }
    void busTimestamp() {
        rocos::atomicStore(master.ecatBus->timestamp, static_cast<long>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()));
    }
    ~OfflineMaster() {
        running.store(false);
        if (thread.joinable()) thread.join();
    }
    static void descriptor(rocos::PdVar& v, const char* name, int offset,
                           int size) {
        std::strcpy(v.name, name);
        v.offset = offset;
        v.size = size;
    }
    rocos::Robot::Config config() const {
        rocos::Robot::Config config;
        config.hardware_config = SERVO_TEST_DATA "/hardware.yaml";
        config.urdf = SERVO_TEST_DATA "/robot.urdf";
        config.base_link = "base";
        config.tip_link = "two";
        config.joint_binding.clear();
        config.ecat_id = id;
        config.composed_configuration = true;
        config.frames = "servo-test-frames.yaml";
        config.tool_library = "servo-test-tools";
        return config;
    }
};
struct Fixture {
    OfflineMaster offline;
    rocos::Robot robot{offline.config()};
    rocos::RobotHttpServer server{&robot};
    httplib::Client client{"127.0.0.1",
                           rocos::RobotHttpServerTestAccess::start(server)};
    httplib::Headers credentials;
    Fixture() {
        client.set_read_timeout(3);
        const auto owner =
            post("/api/control/acquire", {{"client_id", "servo-test"}})["data"];
        credentials = {
            {"X-Rocos-Control-Token", owner["token"].get<std::string>()},
            {"X-Rocos-Client-Id", "servo-test"}};
        REQUIRE(robot.SetEnabled() == Result::NoError);
        REQUIRE(robot.GetStateString() == "STOPPED");
    }
    ~Fixture() { server.stop(); }
    Json post(const std::string& path, const Json& body) {
        const auto response =
            client.Post(path, credentials, body.dump(), "application/json");
        REQUIRE(response);
        CHECK(response->status == 200);
        return Json::parse(response->body);
    }
    bool awaitState(const std::string& state) {
        const auto end =
            std::chrono::steady_clock::now() + std::chrono::seconds(3);
        do {
            if (robot.GetStateString() == state) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (std::chrono::steady_clock::now() < end);
        return false;
    }
};
}  // namespace

TEST_CASE("Hardware position counts round without bias and reject partial batches") {
    OfflineMaster offline;
    auto config = rocos::Hardware::LoadConfigFromYAML(offline.config().hardware_config);
    for (auto& drive : config.drives) {
        drive.transform.cnt_per_unit = 156455.678;
        drive.transform.ratio = drive.id ? -0.5 : 1.0;
        drive.transform.offset_pos_cnt = 123;
    }
    rocos::Hardware hardware(config, offline.id);
    for (const auto& drive : config.drives) {
        const double scale = drive.transform.cnt_per_unit * drive.transform.ratio;
        for (int count : {-100000, -1, 0, 1, 100000}) {
            for (double direction : {-INFINITY, INFINITY}) {
                hardware.SetJointPosition(drive.id, std::nextafter(count / scale, direction));
                CHECK(offline.master.getSlaveOutputVarValue<int32_t>(drive.id, 2) == count + 123);
            }
        }
        CHECK_THROWS_AS(hardware.SetJointPosition(drive.id, INFINITY), std::out_of_range);
        CHECK_THROWS_AS(hardware.SetJointPosition(drive.id, drive.limit.upper + 1), std::out_of_range);
    }
    const auto before0 = offline.master.getSlaveOutputVarValue<int32_t>(0, 2);
    const auto before1 = offline.master.getSlaveOutputVarValue<int32_t>(1, 2);
    rocos::JntArray invalid(2);
    invalid(0) = .1; invalid(1) = INFINITY;
    CHECK_THROWS_AS(hardware.SetPosition(invalid), std::out_of_range);
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(0, 2) == before0);
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(1, 2) == before1);
}

TEST_CASE("librocos actual HTTP and UDP joint servo with offline master") {
    const char* configured_host = std::getenv("ROCOS_TEST_IPV4");
    const std::string host = configured_host ? configured_host : "127.0.0.1";
    OfflineMaster offline;
    auto config = offline.config();
    config.hardware = rocos::Hardware::LoadConfigFromYAML(config.hardware_config);
    for (auto& drive : config.hardware->drives) drive.limit.jerk = 10000;
    rocos::Robot controller(config);
    rocos::RobotHttpServer server(&controller);
    librocos::Options options;
    options.http_port = rocos::RobotHttpServerTestAccess::start(server, host);
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address(host, 0)));
    options.udp_port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(controller.SetEnabled() == Result::NoError);
    offline.follow_targets = true;
    REQUIRE(controller.hardware->GetJointDerivativeLimits()[0].jerk == 10000);
    librocos::Robot client(host, options);
    REQUIRE(client.readOnce().dof == 2);
    client.enableRci();
    double largest_tracking_error = 0;
    double largest_actual_position = 0;
    double minimum_reception = 1;
    std::size_t samples = 0;
    for (int motion = 0; motion < 3; ++motion) {
        librocos::JointArray initial{};
        double elapsed = 0;
        client.control([&](const librocos::RobotState& state, librocos::Duration period) {
            if (period.toNSec() == 0) initial = state.q_d;
            elapsed += period.toSec();
            ++samples;
            largest_tracking_error = std::max(largest_tracking_error, std::abs(state.q[0] - state.q_d[0]));
            largest_actual_position = std::max(largest_actual_position, std::abs(state.q[0]));
            if (state.applied_message_id > 100)
                minimum_reception = std::min(minimum_reception, state.control_command_success_rate);
            const double phase = 2 * std::acos(-1.0) * std::min(elapsed / 4.0, 1.0);
            const double wave = (1 - std::cos(phase)) / 2;
            librocos::JointPositions command(initial, state.dof);
            command.q[0] += 0.001 * wave * wave;
            if (elapsed >= 4.25) return librocos::MotionFinished(command);
            return command;
        });
        const auto status = client.rciStatus();
        REQUIRE(status.robot_state == "STOPPED");
        REQUIRE_FALSE(status.control_active);
        REQUIRE(status.last_control);
        CHECK(status.last_control->fault_code == 0);
    }
    CHECK(samples > 1000);
    CHECK(largest_actual_position > 0.0009);
    CHECK(largest_tracking_error < 0.0001);
    std::cout << "librocos offline master: samples=" << samples
              << ", max |q-q_d|=" << largest_tracking_error
              << ", min reception ratio=" << minimum_reception << '\n';
    client.disableRci();
    CHECK_FALSE(controller.IsRciEnabled());
    server.stop();
}

TEST_CASE_FIXTURE(Fixture, "librocos actual controller reports invalid first point with retained fault") {
    REQUIRE(post("/api/robot/rci/enable", Json::object())["success"] == true);
    librocos::Options options;
    // client 的端口由现有 httplib 对象公开查询。
    options.http_port = client.port();
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    options.udp_port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    librocos::Robot sdk("127.0.0.1", options);
    try {
        sdk.control([](const auto& state, auto) {
            librocos::JointPositions target(state.q_d, state.dof);
            target.q[0] += 0.5;
            return target;
        });
        FAIL("Expected invalid first point");
    } catch (const librocos::ControlException& error) {
        CHECK(error.code() == static_cast<int>(Result::HomePositionError));
        REQUIRE(error.status);
        CHECK(error.status->robot_state == "ERROR_STATE");
    }
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(0, 2) == 1000);
}

TEST_CASE_FIXTURE(Fixture,
                  "Servo HTTP propagates occupied UDP port failure instead of "
                  "reporting success") {
    sockpp::udp_socket occupied;
    REQUIRE(occupied.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(occupied.address()).port();
    const auto response = post("/api/robot/servo/start", {{"port", port}});
    CHECK(response["success"] == false);
    CHECK(response["code"] == static_cast<int>(Result::ServoBindFail));
    CHECK(response["data"]["robot_state"] == "ERROR_STATE");
    CHECK(response["data"]["control_active"] == false);
}

TEST_CASE_FIXTURE(Fixture,
                  "Invalid servo first point enters error without writing it "
                  "to hardware and releases UDP port") {
    sockpp::udp_socket udp;
    REQUIRE(udp.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(udp.address()).port();
    udp.close();
    const auto log_cursor = rocos::LogManager::instance()
                                .query("", 1)["next_id"]
                                .get<std::string>();
    const auto response = post("/api/robot/servo/start", {{"port", port}});
    REQUIRE(response["success"] == true);
    MotionGeneratorCommand cmd{};
    cmd.message_id = 1;
    cmd.session_id=dynamic_cast<rocos::MoveServo*>(robot.motion.get())->GetSession()->id;
    cmd.q_c[0] = 0.1;  // 模型速度 3 rad/s × 1 ms = 0.003 rad。
    sockpp::udp_socket sender;
    REQUIRE(sender.send_to(&cmd, sizeof(cmd),
                           sockpp::inet_address("127.0.0.1", port)) ==
            sizeof(cmd));
    REQUIRE(awaitState("ERROR_STATE"));
    CHECK_FALSE(robot.IsControlActive());
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(0, 2) == 1000);
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(1, 2) == -500);
    rocos::Reference ref;
    CHECK(robot.motion->GenerateRef(ref) == Result::HomePositionError);
    bool fault_logged = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    do {
        const auto logs = rocos::LogManager::instance().query(log_cursor, 100);
        for (const auto& event : logs["events"]) {
            if (event["code"] == static_cast<int>(Result::HomePositionError) &&
                event["message"].get<std::string>().find(
                    "start_pose_invalid") != std::string::npos)
                fault_logged = true;
        }
        if (fault_logged) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    CHECK(fault_logged);
    sockpp::udp_socket probe;
    CHECK(probe.bind(sockpp::inet_address("127.0.0.1", port)));
}

TEST_CASE_FIXTURE(Fixture,
                  "Servo HTTP stop closes receiver and returns to stopped") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(post("/api/robot/servo/start", {{"port", port}})["success"] ==
            true);
    CHECK(post("/api/robot/servo/stop", Json::object())["success"] == true);
    REQUIRE(awaitState("STOPPED"));
    sockpp::udp_socket probe;
    CHECK(probe.bind(sockpp::inet_address("127.0.0.1", port)));
}

TEST_CASE_FIXTURE(Fixture, "Servo receive timeout reaches error and releases resources without new motion") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(post("/api/robot/servo/start", {{"port", port}})["success"] == true);
    MotionGeneratorCommand command{};
    command.message_id = 1;
    command.session_id=dynamic_cast<rocos::MoveServo*>(robot.motion.get())->GetSession()->id;
    sockpp::udp_socket sender;
    REQUIRE(sender.send_to(&command, sizeof(command), sockpp::inet_address("127.0.0.1", port)) == sizeof(command));
    REQUIRE(awaitState("ERROR_STATE"));
    rocos::Reference ref;
    CHECK(robot.motion->GenerateRef(ref) == Result::LostHeartbeat);
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(0, 2) == 1000);
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(1, 2) == -500);
    sockpp::udp_socket probe;
    CHECK(probe.bind(sockpp::inet_address("127.0.0.1", port)));
}

TEST_CASE("Servo derivative limits follow model binding rather than hardware YAML order") {
    OfflineMaster offline;
    auto config = offline.config();
    config.hardware = rocos::Hardware::LoadConfigFromYAML(config.hardware_config);
    for (auto& drive : config.hardware->drives) {
        drive.limit.acc = 10 + drive.id;
        drive.limit.jerk = 100 + drive.id;
        drive.limit.deceleration = .5 + .1 * drive.id;
        drive.limit.velocity_offset = .01 * drive.id;
    }
    rocos::Robot robot(config);
    REQUIRE(robot.hardware->GetJointBinding() == std::vector<int32_t>{0, 1});
    const auto limits = robot.hardware->GetJointDerivativeLimits();
    REQUIRE(limits.size() == 2);
    CHECK(limits[0].acceleration == 10);
    CHECK(limits[1].acceleration == 11);
    CHECK(limits[0].jerk == 100);
    CHECK(limits[1].jerk == 101);
    CHECK(limits[0].deceleration == doctest::Approx(.5));
    CHECK(limits[1].deceleration == doctest::Approx(.6));
    CHECK(limits[0].velocity_offset == 0);
    CHECK(limits[1].velocity_offset == doctest::Approx(.01));
}

TEST_CASE_FIXTURE(Fixture,"Servo master stall triggers error before notifications resume") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1",0)));
    const auto port=sockpp::inet_address(reservation.address()).port(); reservation.close();
    const auto started=post("/api/robot/servo/start",{{"port",port}});
    REQUIRE(started["success"]==true);
    CHECK(started["data"]["protocol_version"]==SERVO_PROTOCOL_VERSION);
    CHECK(started["data"]["session_id"].get<std::string>().size()==32);
    offline.running=false; offline.thread.join();
    const bool error=awaitState("ERROR_STATE");
    const auto control=offline.master.getSlaveOutputVarValue<uint16_t>(0,0);
    // 恢复离线主站仅用于析构，不参与超时判定。
    offline.running=true;
    offline.thread=std::thread([this]{while(offline.running) {
        offline.busTimestamp(); offline.master.notifyClients();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }});
    CHECK(error); CHECK((control & 0x0004)==0); CHECK(robot.hardware->SafetyStopRequested());
}
TEST_CASE_FIXTURE(Fixture,"Releasing HTTP control revokes the active UDP session") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1",0)));
    const auto port=sockpp::inet_address(reservation.address()).port(); reservation.close();
    REQUIRE(post("/api/robot/servo/start",{{"port",port}})["success"]==true);
    auto session=dynamic_cast<rocos::MoveServo*>(robot.motion.get())->GetSession();
    REQUIRE(post("/api/control/release",Json::object())["success"]==true);
    CHECK_FALSE(session->Active()); CHECK(awaitState("ERROR_STATE"));
}

TEST_CASE_FIXTURE(Fixture,"Taking HTTP control revokes the previous UDP lease") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1",0)));
    const auto port=sockpp::inet_address(reservation.address()).port(); reservation.close();
    REQUIRE(post("/api/robot/servo/start",{{"port",port}})["success"]==true);
    auto session=dynamic_cast<rocos::MoveServo*>(robot.motion.get())->GetSession();
    REQUIRE(post("/api/control/takeover",{{"client_id","next-owner"}})["success"]==true);
    CHECK_FALSE(session->Active()); CHECK(awaitState("ERROR_STATE"));
}

TEST_CASE_FIXTURE(
    Fixture,
    "RCI requires explicit enable and rejects competing entry points") {
    const auto owner = credentials;
    credentials.clear();
    CHECK(post("/api/robot/rci/start", Json::object())["success"] == false);
    CHECK(post("/api/robot/rci/enable", Json::object())["code"] == 3006);
    credentials = {{"X-Rocos-Control-Token", "debug-token"},
                   {"X-Rocos-Client-Id", "debug-client"}};
    CHECK(post("/api/robot/rci/enable", Json::object())["success"] == false);
    credentials = owner;
    REQUIRE(post("/api/robot/rci/enable", Json::object())["success"] == true);
    CHECK(robot.GetStateString() == "STOPPED");
    CHECK(robot.GetRciStatus()["control_active"] == false);
    CHECK(robot.MoveJ(robot.hardware->GetPosition()) ==
          Result::ConflictTaskRunning);
    CHECK(robot.MoveL(rocos::Frame::Identity()) == Result::ConflictTaskRunning);
    CHECK(robot.MoveServoing() == Result::ConflictTaskRunning);
    CHECK(robot.SetWorkMode("position") == Result::ConflictTaskRunning);
    CHECK(robot.SetServoMode("cartesian") == Result::ConflictTaskRunning);
    CHECK(robot.SetToolFrame("rci-test", rocos::Frame::Identity()) ==
          Result::CallingConflictError);
    const auto target = offline.master.getSlaveOutputVarValue<int32_t>(0, 2);
    robot.setJointPosition(0, 1.0);
    CHECK(offline.master.getSlaveOutputVarValue<int32_t>(0, 2) == target);
    CHECK(post("/api/robot/movej", {{"q", {0.0, 0.0}}})["success"] == false);
    CHECK(post("/api/script/run", Json::object())["success"] == false);
    CHECK(post("/api/control/takeover", Json::object())["success"] == false);
    CHECK(post("/api/robot/rci/start", {{"port", 0}})["code"] == 1001);
    CHECK(post("/api/robot/rci/start", {{"mode", "torque"}})["code"] == 1001);
    CHECK(post("/api/robot/rci/disable", Json::object())["success"] == true);
    CHECK_FALSE(robot.IsRciEnabled());
    CHECK(robot.SetWorkMode("position") == Result::NoError);
}

TEST_CASE_FIXTURE(Fixture,
                  "RCI accepts another program without credentials and "
                  "survives HTTP release or expiry") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(post("/api/robot/rci/enable", Json::object())["success"] == true);
    const auto owner = credentials;
    credentials.clear();
    const auto started =
        post("/api/robot/rci/start", {{"port", port}, {"mode", "joint"}});
    REQUIRE(started["success"] == true);
    CHECK(started["data"]["session_id"].get<std::string>().size() == 32);
    CHECK(started["data"]["protocol_version"] == SERVO_PROTOCOL_VERSION);
    const auto session =
        dynamic_cast<rocos::MoveServo*>(robot.motion.get())->GetSession();
    CHECK(post("/api/robot/rci/start", {{"port", port}})["success"] == false);
    const auto status = client.Get("/api/robot/rci/status");
    REQUIRE(status);
    CHECK_FALSE(Json::parse(status->body)["data"].contains("session_id"));
    credentials = owner;
    SUBCASE("owner releases HTTP control") {
        REQUIRE(post("/api/control/release", Json::object())["success"] ==
                true);
    }
    SUBCASE("owner HTTP lease expires") {
        rocos::RobotHttpServerTestAccess::expire(server);
        REQUIRE(post("/api/control/heartbeat", Json::object())["code"] == 3007);
    }
    CHECK(session->Active());
    CHECK(robot.GetStateString() == "SERVOING");
    credentials.clear();
    REQUIRE(post("/api/robot/rci/stop", Json::object())["success"] == true);
    REQUIRE(awaitState("STOPPED"));
    CHECK_FALSE(session->Active());
    CHECK(robot.IsRciEnabled());
    const auto restarted = post("/api/robot/rci/start", {{"port", port}});
    REQUIRE(restarted["success"] == true);
    CHECK(restarted["data"]["session_id"] != started["data"]["session_id"]);
    REQUIRE(post("/api/robot/rci/stop", Json::object())["success"] == true);
    REQUIRE(awaitState("STOPPED"));
}

TEST_CASE_FIXTURE(Fixture, "RCI remote IPv4 session restricts UDP origin and endpoint") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.2", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(robot.EnableRci() == Result::NoError);
    const auto started = rocos::RobotHttpServerTestAccess::startFrom(server, "127.0.0.2", port);
    REQUIRE(started["success"] == true);
    auto* motion = dynamic_cast<rocos::MoveServo*>(robot.motion.get());
    REQUIRE(motion);
    const auto session = motion->GetSession();
    CHECK(session->client_ipv4 == 0x7f000002u);
    sockpp::udp_socket authorized, wrong_ip, wrong_port;
    REQUIRE(authorized.bind(sockpp::inet_address("127.0.0.2", 0)));
    REQUIRE(wrong_ip.bind(sockpp::inet_address("127.0.0.1", 0)));
    REQUIRE(wrong_port.bind(sockpp::inet_address("127.0.0.2", 0)));
    for (auto* socket : {&authorized, &wrong_ip, &wrong_port})
        REQUIRE(socket->set_non_blocking(true));
    MotionGeneratorCommand command{};
    command.session_id = session->id;
    command.message_id = 1;
    const auto target = sockpp::inet_address("127.0.0.2", port);
    // 错误源 IP 即使持有会话 ID，也不能预先绑定会话或查询状态。
    REQUIRE(wrong_ip.send_to(&command, sizeof(command), target) == sizeof(command));
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    ::RobotState state{};
    CHECK(wrong_ip.recv(&state, sizeof(state)) < 0);
    REQUIRE(authorized.send_to(&command, sizeof(command), target) == sizeof(command));
    bool replied = false;
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (authorized.recv(&state, sizeof(state)) == sizeof(state)) { replied = true; break; }
    }
    REQUIRE(replied);
    command.message_id = 100;
    REQUIRE(wrong_port.send_to(&command, sizeof(command), target) == sizeof(command));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    CHECK(wrong_port.recv(&state, sizeof(state)) < 0);
    command.message_id = 0;
    REQUIRE(authorized.send_to(&command, sizeof(command), target) == sizeof(command));
    for (int i = 0; i < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (authorized.recv(&state, sizeof(state)) == sizeof(state)) break;
    }
    CHECK(state.received_message_id == 1);
    REQUIRE(robot.StopRciControl() == Result::NoError);
    REQUIRE(awaitState("STOPPED"));
    CHECK_FALSE(session->Active());
    const auto unsupported = rocos::RobotHttpServerTestAccess::startFrom(server, "2001:db8::1", port);
    CHECK(unsupported["code"] == 1001);
}

TEST_CASE_FIXTURE(
    Fixture, "RCI disable stops active control before releasing exclusivity") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(post("/api/robot/rci/enable", Json::object())["success"] == true);
    REQUIRE(post("/api/robot/rci/start", {{"port", port}})["success"] == true);
    const auto session =
        dynamic_cast<rocos::MoveServo*>(robot.motion.get())->GetSession();
    REQUIRE(post("/api/robot/rci/disable", Json::object())["success"] == true);
    REQUIRE(awaitState("STOPPED"));
    CHECK_FALSE(robot.IsRciEnabled());
    CHECK_FALSE(session->Active());
    CHECK(post("/api/robot/rci/start", {{"port", port}})["success"] == false);
    sockpp::udp_socket probe;
    CHECK(probe.bind(sockpp::inet_address("127.0.0.1", port)));
}

TEST_CASE_FIXTURE(
    Fixture,
    "RCI startup failure revokes session and requires fault recovery") {
    sockpp::udp_socket occupied;
    REQUIRE(occupied.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(occupied.address()).port();
    REQUIRE(robot.EnableRci() == Result::NoError);
    std::shared_ptr<rocos::ServoSession> session;
    CHECK(robot.StartRciControl(session, port) == Result::ServoBindFail);
    CHECK_FALSE(session);
    CHECK(robot.GetStateString() == "ERROR_STATE");
    CHECK(robot.IsRciEnabled());
    CHECK(robot.GetRciStatus()["control_active"] == false);
    CHECK(robot.StartRciControl(session, port) == Result::ConflictTaskRunning);
    CHECK(robot.DisableRci() == Result::NoError);
}

TEST_CASE_FIXTURE(Fixture, "Concurrent RCI programs receive only one session") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(robot.EnableRci() == Result::NoError);
    std::shared_ptr<rocos::ServoSession> a, b;
    Result ra, rb;
    std::thread first([&] { ra = robot.StartRciControl(a, port); });
    std::thread second([&] { rb = robot.StartRciControl(b, port); });
    first.join();
    second.join();
    CHECK((ra == Result::NoError) != (rb == Result::NoError));
    CHECK((ra == Result::ConflictTaskRunning) !=
          (rb == Result::ConflictTaskRunning));
    CHECK(bool(a) != bool(b));
    REQUIRE(robot.StopRciControl() == Result::NoError);
    REQUIRE(awaitState("STOPPED"));
}

TEST_CASE_FIXTURE(
    Fixture,
    "RCI and Lua execution exclude each other including single stepping") {
    rocos::LuaScriptEngine script(robot, SERVO_TEST_DATA);
    REQUIRE(script.LoadSource("local x = 1\nx = x + 1\n", "rci.lua") ==
            Result::NoError);
    REQUIRE(robot.EnableRci() == Result::NoError);
    CHECK(script.Run() == Result::ConflictTaskRunning);
    CHECK(script.Step() == Result::ConflictTaskRunning);
    REQUIRE(robot.DisableRci() == Result::NoError);
    REQUIRE(script.AddBreakpoint("rci.lua", 1) == Result::NoError);
    REQUIRE(script.Run() == Result::NoError);
    CHECK(robot.EnableRci() == Result::ConflictTaskRunning);
    REQUIRE(script.Stop() == Result::NoError);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    Result enabled = Result::ConflictTaskRunning;
    do {
        enabled = robot.EnableRci();
        if (enabled == Result::NoError) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    CHECK(enabled == Result::NoError);
}

TEST_CASE_FIXTURE(
    Fixture,
    "RCI disconnect revokes control and cannot bypass fault recovery") {
    sockpp::udp_socket reservation;
    REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1", 0)));
    const auto port = sockpp::inet_address(reservation.address()).port();
    reservation.close();
    REQUIRE(robot.EnableRci() == Result::NoError);
    std::shared_ptr<rocos::ServoSession> session;
    REQUIRE(robot.StartRciControl(session, port) == Result::NoError);
    MotionGeneratorCommand command{};
    command.message_id = 1;
    command.session_id = session->id;
    const auto actual = robot.hardware->GetPosition();
    for (unsigned int i = 0; i < actual.rows(); ++i) command.q_c[i] = actual(i);
    sockpp::udp_socket sender;
    REQUIRE(sender.send_to(&command, sizeof(command),
                           sockpp::inet_address("127.0.0.1", port)) ==
            sizeof(command));
    REQUIRE(awaitState("ERROR_STATE"));
    CHECK_FALSE(session->Active());
    CHECK(robot.IsRciEnabled());
    CHECK(robot.GetRciStatus()["control_active"] == false);
    CHECK(robot.hardware->SafetyStopRequested());
    std::shared_ptr<rocos::ServoSession> next;
    CHECK(robot.StartRciControl(next, port) == Result::ConflictTaskRunning);
    CHECK_FALSE(next);
    sockpp::udp_socket probe;
    CHECK(probe.bind(sockpp::inet_address("127.0.0.1", port)));
}

TEST_CASE_FIXTURE(Fixture,"RCI rapid start stop preserves the control thread and terminal status") {
    REQUIRE(robot.EnableRci()==Result::NoError);
    for(int i=0;i<300;++i) {
        sockpp::udp_socket reservation;
        REQUIRE(reservation.bind(sockpp::inet_address("127.0.0.1",0)));
        const auto port=sockpp::inet_address(reservation.address()).port();reservation.close();
        std::shared_ptr<rocos::ServoSession> session;
        REQUIRE(robot.StartRciControl(session,port)==Result::NoError);
        std::this_thread::sleep_for(std::chrono::microseconds(100+(i%10)*500));
        REQUIRE(robot.StopRciControl()==Result::NoError);
        REQUIRE(awaitState("STOPPED"));
        CHECK(robot.IsControlActive());
        CHECK_FALSE(session->Active());
        const auto status=robot.GetRciStatus();
        REQUIRE(status["last_control"].is_object());
        CHECK(status["last_control"]["fault_code"]==0);
    }
}
