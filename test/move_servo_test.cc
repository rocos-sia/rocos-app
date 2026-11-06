// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "src/move_servo.hpp"

#include <test/doctest.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <string>
#include <variant>
#include <vector>

namespace {

class FakeHardware : public rocos::HardwareInterface {
 public:
    explicit FakeHardware(const rocos::JntArray& position)
        : fake_position_(position) {}

    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override {
        std::unique_lock<std::mutex> lock(gate_mutex);
        feedback_entered = true;
        gate_cv.notify_all();
        gate_cv.wait(lock, [this] { return !feedback_paused; });
        return rocos::JntArray(fake_position_.rows());
    }
    uint32_t GetDt() const override { return dt_us; }
    uint32_t dt_us{1000};
    std::vector<rocos::JointDerivativeLimits> GetJointDerivativeLimits() const override {
        return std::vector<rocos::JointDerivativeLimits>(fake_position_.rows(), {10, 150, 10, 0});
    }
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool feedback_paused{false}, feedback_entered{false};
    void pauseFeedback() {
        std::lock_guard<std::mutex> lock(gate_mutex);
        feedback_paused = true;
        feedback_entered = false;
    }
    bool awaitFeedback() {
        std::unique_lock<std::mutex> lock(gate_mutex);
        return gate_cv.wait_for(lock, std::chrono::seconds(2),
                                [this] { return feedback_entered; });
    }
    void releaseFeedback() {
        std::lock_guard<std::mutex> lock(gate_mutex);
        feedback_paused = false;
        gate_cv.notify_all();
    }
    rocos::JntArray GetTorque() override { return rocos::JntArray(fake_position_.rows()); }
    rocos::JntArray GetLoadTorque() override { return rocos::JntArray(fake_position_.rows()); }
    void SetPosition(const rocos::JntArray&) override {}
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray&) override {}
    void SetMode(int8_t) override {}
    void SetEnabled() override {}
    void SetDisabled() override {}
    rocos::JntState GetState() override { return rocos::JntState::ENABLED; }
    double GetJointPosition(int32_t) override { return 0.0; }
    double GetJointVelocity(int32_t) override { return 0.0; }
    double GetJointTorque(int32_t) override { return 0.0; }
    double GetJointLoadTorque(int32_t) override { return 0.0; }
    void SetJointPosition(int32_t, double) override {}
    void SetJointVelocity(int32_t, double) override {}
    void SetJointTorque(int32_t, double) override {}
    void SetJointMode(int32_t, int8_t) override {}
    void SetJointEnabled(int32_t) override {}
    void SetJointDisabled(int32_t) override {}
    rocos::JntState GetJointState(int32_t) override { return rocos::JntState::ENABLED; }
    std::string getJointName(int32_t) override { return ""; }
    rocos::Wrench GetWrench() override { return rocos::Wrench::Zero(); }
    bool GetDigitalInput(int, int) override { return false; }
    void SetDigitalOutput(int, int, bool) override {}
    double GetAnalogInput(int, int) override { return 0.0; }
    void SetAnalogOutput(int, int, double) override {}
    bool Reset() override { return true; }
    void WaitForSignal() override {}
    int GetDriveNum() const override { return static_cast<int>(fake_position_.rows()); }

 private:
    rocos::JntArray fake_position_;
};

class FakeModel : public rocos::ModelInterface {
 public:
  FakeModel() : lower(3), upper(3), velocity(3) {
      for (unsigned int i = 0; i < 3; ++i) {
          lower(i) = -10;
          upper(i) = 10;
          velocity(i) = i + 1;
      }
  }
  rocos::JntArray lower, upper, velocity;
  rocos::Result ik_result{rocos::Result::NoError};
  const rocos::JntArray& GetPosLowerLimit() const override { return lower; }
  const rocos::JntArray& GetPosUpperLimit() const override { return upper; }
  const rocos::JntArray& GetVelocityLimit() const override { return velocity; }
  rocos::Result ForwardKinematics(const rocos::JntArray& q_in,
                                  rocos::Frame& p_out) override {
      p_out = rocos::Frame(rocos::Vector(q_in(0), q_in(1), q_in(2)));
      return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray&,
                                    const rocos::Frame& pose,
                                    rocos::JntArray& q) override {
        q.resize(3);
        for (unsigned int i = 0; i < 3; ++i) q(i) = pose.p(i);
        return ik_result;
    }

    rocos::Result ForwardDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    rocos::Result GetJacobian(const rocos::JntArray&,
                              rocos::Jacobian&) override {
        return rocos::Result::NoError;
    }

    int GetJointNum() const override { return 3; }
    std::vector<std::string> GetJointNames() const override { return {}; }
};

rocos::JntArray makeJntArray(double q0, double q1, double q2) {
    rocos::JntArray q(3);
    q(0) = q0;
    q(1) = q1;
    q(2) = q2;
    return q;
}

}  // namespace

TEST_CASE("MoveServo GenerateRef fills current joint position when no command is available") {
    const auto current_position = makeJntArray(1.0, 2.0, 3.0);
    FakeHardware hw(current_position);
    rocos::MoveServo move(&hw, nullptr, 0);
    move.SetMode(MotionMode::kJointPosition);

    rocos::Reference ref = rocos::Frame::Identity();

    CHECK(move.GenerateRef(ref) == rocos::Result::NoError);
    REQUIRE(std::holds_alternative<rocos::JntArray>(ref));

    const auto& q_ref = std::get<rocos::JntArray>(ref);
    REQUIRE(q_ref.rows() == current_position.rows());
    CHECK(q_ref(0) == doctest::Approx(1.0));
    CHECK(q_ref(1) == doctest::Approx(2.0));
    CHECK(q_ref(2) == doctest::Approx(3.0));
}

TEST_CASE("MoveServo holds current joints while waiting for a cartesian first point") {
    FakeHardware hw(makeJntArray(4.0, 5.0, 6.0));
    FakeModel model;
    rocos::MoveServo move(&hw, &model, 0);
    move.SetMode(MotionMode::kCartesianPosition);

    rocos::Reference ref = rocos::JntArray(3);

    CHECK(move.GenerateRef(ref) == rocos::Result::NoError);
    REQUIRE(std::holds_alternative<rocos::JntArray>(ref));
    const auto& q_ref = std::get<rocos::JntArray>(ref);
    CHECK(q_ref(0) == doctest::Approx(4.0));
    CHECK(q_ref(1) == doctest::Approx(5.0));
    CHECK(q_ref(2) == doctest::Approx(6.0));
}

namespace {
using rocos::Result;
struct UdpClient {
    sockpp::udp_socket socket;
    std::shared_ptr<rocos::ServoSession> session;
    explicit UdpClient() {
        REQUIRE(socket.bind(sockpp::inet_address("127.0.0.1", 0)));
        const timeval timeout{0, 200000};
        REQUIRE(socket.set_option(SOL_SOCKET, SO_RCVTIMEO, timeout));
    }
    uint16_t port() const {
        return sockpp::inet_address(socket.address()).port();
    }
    void send(uint16_t port, MotionGeneratorCommand cmd) {
        if (session) cmd.session_id=session->id;
        REQUIRE(socket.send_to(&cmd, sizeof(cmd),
                               sockpp::inet_address("127.0.0.1", port)) ==
                sizeof(cmd));
    }
    bool receive() {
        RobotState state{};
        while (socket.recv_from(&state, sizeof(state)) == sizeof(state)) {
            if (!session || state.session_id == session->id) return true;
        }
        return false;
    }
};
uint16_t freePort() {
    UdpClient reservation;
    return reservation.port();
}
MotionGeneratorCommand jointCommand(uint64_t id, double q0 = 0, double q1 = 0,
                                    double q2 = 0) {
    MotionGeneratorCommand cmd{};
    cmd.message_id = id;
    cmd.q_c[0] = q0;
    cmd.q_c[1] = q1;
    cmd.q_c[2] = q2;
    cmd.tcp_c[0] = cmd.tcp_c[5] = cmd.tcp_c[10] = cmd.tcp_c[15] = 1;
    cmd.tcp_c[3] = q0;
    cmd.tcp_c[7] = q1;
    cmd.tcp_c[11] = q2;
    return cmd;
}
struct ServoFixture {
    FakeHardware hw{makeJntArray(0, 0, 0)};
    FakeModel model;
    uint16_t port{freePort()};
    rocos::MoveServo move{&hw, &model, port};
    UdpClient client;
    rocos::Reference ref;
    ServoFixture() { REQUIRE(move.Reset() == Result::NoError); client.session=move.GetSession(); }
    ~ServoFixture() {
        hw.releaseFeedback();
        move.Stop();
    }
    void exchange(const MotionGeneratorCommand& cmd) {
        client.send(port, cmd);
        REQUIRE(client.receive());
    }
    void barrier() { exchange(jointCommand(0)); }
};
}  // namespace

TEST_CASE(
    "MoveServo reports bind failure and can retry after the port is released") {
    UdpClient reservation;
    const auto port = reservation.port();
    FakeHardware hw(makeJntArray(0, 0, 0));
    FakeModel model;
    rocos::MoveServo move(&hw, &model, port);
    CHECK(move.Reset() == Result::ServoBindFail);
    reservation.socket.close();
    REQUIRE(move.Reset() == Result::NoError);
    CHECK(move.Stop() == Result::NoError);
    sockpp::udp_socket probe;
    CHECK(probe.bind(sockpp::inet_address("127.0.0.1", port)));
}

TEST_CASE_FIXTURE(ServoFixture,
                  "MoveServo rejects stale and duplicate packets across "
                  "batches without resetting on probes") {
    exchange(jointCommand(10));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    exchange(jointCommand(12, 0.0005));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    client.send(port, jointCommand(11, 0.8));
    client.send(port, jointCommand(12, 0.9));
    barrier();
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    CHECK(move.GetDiagnostics().received_id == 12);
    CHECK(std::get<rocos::JntArray>(ref)(0) > 0);
    CHECK(std::get<rocos::JntArray>(ref)(0) < 0.0005);
    exchange(jointCommand(13, 0.0008));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    CHECK(move.GetDiagnostics().received_id == 13);
    CHECK(std::get<rocos::JntArray>(ref)(0) < 0.0008);
}

TEST_CASE_FIXTURE(ServoFixture, "MoveServo binds the authorized sender endpoint") {
    exchange(jointCommand(20));
    REQUIRE(move.GenerateRef(ref)==Result::NoError);
    UdpClient other; other.session=move.GetSession();
    other.send(port,jointCommand(100,.1));
    CHECK_FALSE(other.receive());
    CHECK(move.GetDiagnostics().received_id==20);
    exchange(jointCommand(21));
}

TEST_CASE_FIXTURE(ServoFixture,
                  "MoveServo preserves the first point until the control "
                  "thread validates it") {
    exchange(jointCommand(1, 0.1));
    exchange(jointCommand(2));
    ref = rocos::Frame::Identity();
    CHECK(move.GenerateRef(ref) == Result::HomePositionError);
    CHECK(std::holds_alternative<rocos::Frame>(ref));
    CHECK(move.GenerateRef(ref) == Result::HomePositionError);
    CHECK(move.Stop() == Result::NoError);
    CHECK(move.GenerateRef(ref) == Result::HomePositionError);
}

TEST_CASE_FIXTURE(ServoFixture,
                  "MoveServo first point tolerance follows each model axis and "
                  "hardware period") {
    move.Stop();
    hw.dt_us = 2000;
    REQUIRE(move.Reset() == Result::NoError);
    SUBCASE("boundary accepted") {
        exchange(jointCommand(1, 0.002, 0.004, 0.006));
        CHECK(move.GenerateRef(ref) == Result::NoError);
    }
    SUBCASE("axis-specific limit exceeded") {
        exchange(jointCommand(1, 0, 0.0041));
        CHECK(move.GenerateRef(ref) == Result::HomePositionError);
    }
    SUBCASE("NaN rejected") {
        exchange(jointCommand(1, std::numeric_limits<double>::quiet_NaN()));
        CHECK(move.GenerateRef(ref) == Result::ParameterNanOrInf);
    }
    SUBCASE("position limit rejected") {
        exchange(jointCommand(1, 11));
        CHECK(move.GenerateRef(ref) == Result::PosLimit);
    }
}

TEST_CASE_FIXTURE(ServoFixture,
                  "MoveServo probe is not a first point and mode changes "
                  "require a fresh first point") {
    barrier();
    CHECK(move.GenerateRef(ref) == Result::NoError);
    exchange(jointCommand(1));
    CHECK(move.GenerateRef(ref) == Result::NoError);
    move.SetMode(MotionMode::kCartesianPosition);
    CHECK(move.GenerateRef(ref) == Result::NoError);
    REQUIRE(std::holds_alternative<rocos::JntArray>(ref));
    exchange(jointCommand(2, 0.1));
    CHECK(move.GenerateRef(ref) == Result::HomePositionError);
}

TEST_CASE_FIXTURE(
    ServoFixture,
    "MoveServo validates cartesian first points and does not hide IK failure") {
    move.SetMode(MotionMode::kCartesianPosition);
    auto cmd = jointCommand(1, 0.0005);
    auto expected = Result::NoError;
    SUBCASE("reachable near pose") {}
    SUBCASE("unreachable pose") {
        model.ik_result = Result::IkCalcFail;
        expected = Result::IkCalcFail;
    }
    SUBCASE("invalid rotation") {
        cmd.tcp_c[0] = 2;
        expected = Result::IllegalParameter;
    }
    exchange(cmd);
    CHECK(move.GenerateRef(ref) == expected);
}

TEST_CASE_FIXTURE(
    ServoFixture,
    "MoveServo restart resets sequence and first point validation") {
    exchange(jointCommand(100));
    CHECK(move.GenerateRef(ref) == Result::NoError);
    REQUIRE(move.Reset() == Result::NoError);
    exchange(jointCommand(1, 0.1));
    CHECK(move.GenerateRef(ref) == Result::HomePositionError);
}

TEST_CASE_FIXTURE(
    ServoFixture,
    "MoveServo ignores oversized datagrams instead of accepting their prefix") {
    const auto cmd = jointCommand(100, 0.1);
    std::array<char, sizeof(cmd) + 1> oversized{};
    std::memcpy(oversized.data(), &cmd, sizeof(cmd));
    REQUIRE(client.socket.send_to(oversized.data(), oversized.size(),
                                  sockpp::inet_address("127.0.0.1", port)) ==
            oversized.size());
    barrier();
    exchange(jointCommand(1));
    CHECK(move.GenerateRef(ref) == Result::NoError);
}

TEST_CASE("MoveServo rejects unusable startup configuration") {
    FakeHardware hw(makeJntArray(0, 0, 0));
    FakeModel model;
    const auto port = freePort();
    SUBCASE("zero cycle") { hw.dt_us = 0; }
    SUBCASE("invalid model velocity") { model.velocity(0) = -1; }
    rocos::MoveServo move(&hw, &model, port);
    CHECK(move.Reset() == Result::IllegalParameter);
}

TEST_CASE("MoveServo compares the first target to nonzero actual feedback") {
    FakeHardware hw(makeJntArray(1, 2, 3));
    FakeModel model;
    const auto port = freePort();
    rocos::MoveServo move(&hw, &model, port);
    REQUIRE(move.Reset() == Result::NoError);
    UdpClient client;
    client.session=move.GetSession();
    auto cmd = jointCommand(1, 1.0005, 2, 3);
    auto expected = Result::NoError;
    SUBCASE("near actual pose") {}
    SUBCASE("near zero but far from actual pose") {
        cmd = jointCommand(1);
        expected = Result::HomePositionError;
    }
    client.send(port, cmd);
    REQUIRE(client.receive());
    rocos::Reference ref;
    CHECK(move.GenerateRef(ref) == expected);
}

TEST_CASE("Servo communication counts received control cycles not reply sends") {
    rocos::ServoCommunication monitor;
    CHECK(monitor.SuccessRate() == 0);
    for (int i = 0; i < 100; ++i) monitor.Record(i % 10 != 0);
    CHECK(monitor.FullWindow());
    CHECK(monitor.SuccessRate() == doctest::Approx(0.9));
    monitor.Record(false, 20);
    CHECK(monitor.MissingCycles() == 20);
    CHECK(monitor.SuccessRate() == doctest::Approx(0.72));
    monitor.Record(true, 101);
    CHECK(monitor.MissingCycles() == 0);
    CHECK(monitor.SuccessRate() == doctest::Approx(0.01));
    monitor.Reset();
    CHECK_FALSE(monitor.FullWindow());
    CHECK(monitor.SuccessRate() == 0);
}

TEST_CASE("Servo default filtering and derivative bounds hold across periods and reversals") {
    FakeModel model;
    for (const double dt : {0.0005, 0.001, 0.002}) {
        rocos::ServoSignalProcessor processor;
        const std::vector<rocos::JointDerivativeLimits> limits(3, {2, 20, 2, 0});
        REQUIRE(processor.Reset(&model, limits, makeJntArray(0, 0, 0), dt, {}) == Result::NoError);
        auto previous = makeJntArray(0, 0, 0);
        auto previous_v = previous, previous_a = previous;
        for (int k = 0; k < 2500; ++k) {
            rocos::JntArray output;
            const auto target = makeJntArray(k < 1000 ? 0.5 : -0.5, 0.1, -0.2);
            CAPTURE(dt);
            CAPTURE(k);
            CAPTURE(processor.Position()(0));
            CAPTURE(processor.Velocity()(0));
            REQUIRE(processor.Step(target, false, output) == Result::NoError);
            for (unsigned int i = 0; i < 3; ++i) {
                const auto v = (output(i) - previous(i)) / dt;
                const auto a = (v - previous_v(i)) / dt;
                const auto jerk = (a - previous_a(i)) / dt;
                CHECK(std::abs(v) <= model.velocity(i) + 1e-8);
                CHECK(std::abs(a) <= 2 + 1e-7);
                CHECK(std::abs(jerk) <= 20 + 1e-4);
                previous_a(i) = a;
                previous_v(i) = v;
            }
            previous = output;
        }
        bool stopped = false;
        for (int k = 0; k < 5000; ++k) {
            rocos::JntArray output;
            REQUIRE(processor.Step(previous, true, output) == Result::NoError);
            CHECK(std::abs(processor.Acceleration()(0)) <= 2 + 1e-9);
            if (processor.AtRest()) { stopped = true; break; }
        }
        CHECK(stopped);
        CHECK(std::abs(processor.Velocity()(0)) < 1e-10);
        CHECK(std::abs(processor.Acceleration()(0)) < 1e-10);
    }
}

TEST_CASE("Servo lowpass uses configured cutoff and necessary checks survive disabling limiter") {
    FakeModel model;
    rocos::ServoSettings settings;
    settings.rate_limit_enabled = false;
    rocos::ServoSignalProcessor processor;
    const std::vector<rocos::JointDerivativeLimits> limits(3, {1000000, 1e12, 10, 0});
    REQUIRE(processor.Reset(&model, limits, makeJntArray(0, 0, 0), .001, settings) == Result::NoError);
    rocos::JntArray output;
    REQUIRE(processor.Step(makeJntArray(.001, 0, 0), false, output) == Result::NoError);
    const double alpha = .001 / (.001 + 1 / (2 * std::acos(-1.0) * 100));
    CHECK(output(0) == doctest::Approx(.001 * alpha));
    CHECK(processor.Step(makeJntArray(11, 0, 0), false, output) == Result::PosLimit);
    CHECK(processor.Step(makeJntArray(1, 0, 0), false, output) == Result::SpeedLimit);
}

TEST_CASE_FIXTURE(ServoFixture, "Servo watchdog stops on missing cycles and never resumes from fresh packets") {
    exchange(jointCommand(1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    exchange(jointCommand(2, 0.1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    for (int i = 0; i < 20; ++i) REQUIRE(move.GenerateRef(ref) == Result::NoError);
    CHECK(move.GetDiagnostics().stopping);
    CHECK(move.GetDiagnostics().fault == Result::LostHeartbeat);
    client.send(port, jointCommand(3, 0.5));
    barrier();
    CHECK(move.GetDiagnostics().received_id == 2);
    Result result = Result::NoError;
    for (int i = 0; i < 5000 && result == Result::NoError; ++i)
        result = move.GenerateRef(ref);
    CHECK(result == Result::LostHeartbeat);
    CHECK(move.GenerateRef(ref) == Result::LostHeartbeat);
}

TEST_CASE_FIXTURE(ServoFixture, "Servo moving mode change is rejected and ordinary stop decelerates") {
    exchange(jointCommand(1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    exchange(jointCommand(2, .1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    CHECK(move.SetMode(MotionMode::kCartesianPosition) == Result::ConflictTaskRunning);
    REQUIRE(move.Stop() == Result::NoError);
    Result result = Result::NoError;
    for (int i = 0; i < 5000 && result == Result::NoError; ++i)
        result = move.GenerateRef(ref);
    CHECK(result == Result::PlanFinished);
    CHECK(move.GetDiagnostics().desired_velocity(0) == doctest::Approx(0));
    CHECK(move.GetDiagnostics().desired_acceleration(0) == doctest::Approx(0));
}

TEST_CASE("Servo startup watchdog does not let probes keep the session alive") {
    FakeHardware hw(makeJntArray(0, 0, 0));
    FakeModel model;
    rocos::ServoSettings settings;
    settings.first_command_timeout_ms = 5;
    const auto port = freePort();
    rocos::MoveServo move(&hw, &model, port, settings);
    REQUIRE(move.Reset() == Result::NoError);
    UdpClient client;
    client.session=move.GetSession();
    client.send(port, jointCommand(0));
    REQUIRE(client.receive());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    rocos::Reference ref;
    CHECK(move.GenerateRef(ref) == Result::LostHeartbeat);
}

TEST_CASE_FIXTURE(ServoFixture, "UDP reports receive quality and probes cannot improve it") {
    client.send(port, jointCommand(0));
    RobotState state{};
    REQUIRE(client.socket.recv_from(&state, sizeof(state)) == sizeof(state));
    CHECK(state.control_command_success_rate == 0);
    exchange(jointCommand(1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    move.CompleteCycle(Result::NoError);
    for (int i = 0; i < 3; ++i) {
        client.send(port, jointCommand(0));
        REQUIRE(client.socket.recv_from(&state, sizeof(state)) == sizeof(state));
        CHECK(state.control_command_success_rate == doctest::Approx(0.5));
    }
}

TEST_CASE("Servo sustained low quality stops even without a long loss burst") {
    FakeHardware hw(makeJntArray(0, 0, 0));
    FakeModel model;
    rocos::ServoSettings settings;
    settings.low_quality_cycles = 3;
    settings.max_missing_cycles = 100000;
    const auto port = freePort();
    rocos::MoveServo move(&hw, &model, port, settings);
    REQUIRE(move.Reset() == Result::NoError);
    UdpClient client;
    client.session=move.GetSession();
    rocos::Reference ref;
    uint64_t id = 0;
    for (int i = 0; i < 104 && !move.GetDiagnostics().stopping; ++i) {
        if (i % 2 == 0) {
            client.send(port, jointCommand(++id));
            REQUIRE(client.receive());
        }
        REQUIRE(move.GenerateRef(ref) == Result::NoError);
    }
    CHECK(move.GetDiagnostics().stopping);
    CHECK(move.GetDiagnostics().fault == Result::LostHeartbeat);
    CHECK(move.GetDiagnostics().missing_cycles < 20);
}

TEST_CASE("Servo cartesian commands include the active tool and reject raw IK failures") {
    FakeHardware hw(makeJntArray(0, 0, 0));
    FakeModel model;
    const auto port = freePort();
    rocos::MoveServo move(&hw, &model, port, {}, rocos::Frame(rocos::Vector(.2, 0, 0)));
    REQUIRE(move.SetMode(MotionMode::kCartesianPosition) == Result::NoError);
    REQUIRE(move.Reset() == Result::NoError);
    UdpClient client;
    client.session=move.GetSession();
    client.send(port, jointCommand(1, .2));
    RobotState state{};
    REQUIRE(client.socket.recv_from(&state, sizeof(state)) == sizeof(state));
    CHECK(state.flange_to_base[3] == 0);
    CHECK_FALSE(state.output_valid);
    rocos::Reference ref;
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    REQUIRE(std::holds_alternative<rocos::JntArray>(ref));
    CHECK(std::get<rocos::JntArray>(ref)(0) == doctest::Approx(0));
}

TEST_CASE("Servo approaches position limits within the model braking envelope") {
    FakeModel model;
    model.lower.data.setConstant(-1);
    model.upper.data.setConstant(1);
    for (const auto limits : {rocos::JointDerivativeLimits{2, 20, 2, 0},
                              rocos::JointDerivativeLimits{2, 20, 1, .05}}) {
        rocos::ServoSignalProcessor processor;
        REQUIRE(processor.Reset(&model, std::vector<rocos::JointDerivativeLimits>(3, limits),
                                makeJntArray(0, 0, 0), .001, {}) == Result::NoError);
        // 阻尼跟踪渐近收敛：5 秒目标轨迹后留出 7 秒稳定时间。
        for (int k = 0; k < 12000; ++k) {
            CAPTURE(k);
            CAPTURE(limits.deceleration);
            CAPTURE(limits.velocity_offset);
            CAPTURE(processor.Position()(0));
            CAPTURE(processor.Velocity()(0));
            rocos::JntArray output;
            // 逐周期限速接收连续轨迹；不再要求规划器替阶跃目标生成到点运动。
            const double t=std::min(1.0,k*.001/5.0);
            const double target=.95*t*t*t*(10+t*(-15+6*t));
            REQUIRE(processor.Step(makeJntArray(target, -target, 0), false, output) == Result::NoError);
        }
        CHECK(processor.Position()(0) == doctest::Approx(.95).epsilon(1e-6));
        CHECK(processor.Position()(1) == doctest::Approx(-.95).epsilon(1e-6));
        CHECK(processor.AtRest());
    }
}

TEST_CASE("Servo first point respects position dependent directional tolerance") {
    FakeHardware hw(makeJntArray(9.9999, 0, 0));
    FakeModel model;
    const auto port = freePort();
    rocos::MoveServo move(&hw, &model, port);
    REQUIRE(move.Reset() == Result::NoError);
    UdpClient client;
    client.session=move.GetSession();
    client.send(port, jointCommand(1, 9.99998));
    REQUIRE(client.receive());
    rocos::Reference ref = rocos::Frame::Identity();
    CHECK(move.GenerateRef(ref) == Result::HomePositionError);
    CHECK(std::holds_alternative<rocos::Frame>(ref));
}

TEST_CASE_FIXTURE(ServoFixture, "Servo rejects a runtime cycle change rather than using stale derivative state") {
    exchange(jointCommand(1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    hw.dt_us = 2000;
    CHECK(move.GenerateRef(ref) == Result::IllegalParameter);
}

TEST_CASE_FIXTURE(ServoFixture, "A fresh packet after a missed control deadline cannot conceal the gap") {
    exchange(jointCommand(1));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    exchange(jointCommand(2));
    REQUIRE(move.GenerateRef(ref) == Result::NoError);
    CHECK(move.GetDiagnostics().stopping);
    CHECK(move.GetDiagnostics().fault == Result::LostHeartbeat);
    CHECK(move.GetDiagnostics().success_rate == 1);
    CHECK(move.GetDiagnostics().elapsed_periods >= 25);
    CHECK(move.GetDiagnostics().control_cycles == 2);
    CHECK(move.GenerateRef(ref) == Result::LostHeartbeat);
}

TEST_CASE("Servo per-cycle limiter preserves braking feasibility in both directions") {
    FakeModel model; model.lower.data.setConstant(-1); model.upper.data.setConstant(1);
    for (double dec:{.2,1.,2.,10.}) for (double jerk:{1.,20.,150.})
    for (double direction:{-1.,1.}) for (double target:{.7,.95,.999})
    for (int stop_cycle:{50,200,500,900}) {
        rocos::ServoSignalProcessor p;
        REQUIRE(p.Reset(&model,std::vector<rocos::JointDerivativeLimits>(3,{2,jerk,dec,0}),
                        makeJntArray(0,0,0),.001,{})==Result::NoError);
        rocos::JntArray out;
        for (int k=0;k<stop_cycle;++k) {
            const auto old_q=p.Position(), old_v=p.Velocity(), old_a=p.Acceleration();
            REQUIRE(p.Step(makeJntArray(direction*target,0,0),false,out)==Result::NoError);
            CHECK(std::abs(p.Acceleration()(0))<=2+1e-9);
            CHECK(std::abs(p.Acceleration()(0)-old_a(0))<=jerk*.001+1e-8);
            CHECK((out(0)-old_q(0))/.001==doctest::Approx(p.Velocity()(0)).epsilon(1e-7));
        }
        int steps=0;
        while(!p.AtRest() && ++steps<12000)
            REQUIRE(p.Step(p.Position(),true,out)==Result::NoError);
        CHECK(p.AtRest());
    }
}
TEST_CASE_FIXTURE(ServoFixture,"Servo rejects stale session packets and revoked leases") {
    auto old=move.GetSession()->id;
    REQUIRE(move.Reset()==Result::NoError);
    MotionGeneratorCommand stale=jointCommand(999,.1); stale.session_id=old;
    REQUIRE(client.socket.send_to(&stale,sizeof(stale),sockpp::inet_address("127.0.0.1",port))==sizeof(stale));
    CHECK_FALSE(client.receive());
    exchange(jointCommand(1)); REQUIRE(move.GenerateRef(ref)==Result::NoError);
    move.CompleteCycle(Result::NoError);
    client.send(port,jointCommand(0));
    RobotState state{}; REQUIRE(client.socket.recv_from(&state,sizeof(state))==sizeof(state));
    CHECK(state.applied_message_id==1); CHECK(state.sample_time_ns>0); CHECK(state.cycle>0);
    CHECK(state.session_id==move.GetSession()->id);
    move.GetSession()->valid.store(false);
    CHECK(move.GenerateRef(ref)==Result::NoError);
    CHECK(move.GetDiagnostics().stopping);
    CHECK(move.GenerateRef(ref)==Result::LostHeartbeat);
}

TEST_CASE("Servo regression small targets converge without self-excited motion") {
    FakeModel model;
    model.velocity.data.setConstant(3);
    for (int update_cycles : {1, 2, 3, 10}) {
        rocos::ServoSignalProcessor processor;
        REQUIRE(processor.Reset(&model, std::vector<rocos::JointDerivativeLimits>(3, {10,150,10,0}),
                                makeJntArray(0,0,0), .001, {}) == Result::NoError);
        rocos::JntArray output;
        double peak = 0;
        for (int k=0; k<20000; ++k) {
            double t=(k/update_cycles)*update_cycles*.001;
            const double target=.01*std::sin(2*std::acos(-1.)*.05*t)*
                std::pow(std::sin(std::min(t/5.,1.)*std::acos(-1.)/2),2);
            REQUIRE(processor.Step(makeJntArray(target,.00297,0),false,output)==Result::NoError);
            peak=std::max(peak,std::abs(output(0)));
            CHECK(std::abs(output(1))<=.003+1e-8);
        }
        CAPTURE(update_cycles);
        CHECK(peak<=.0101);
        CHECK(output(1)==doctest::Approx(.00297).epsilon(1e-5));
        CHECK(std::abs(processor.Velocity()(1))<1e-7);
    }
}

TEST_CASE("Servo regression waiting reference equals nonzero actual position") {
    FakeHardware hw(makeJntArray(1,2,3)); FakeModel model;
    const auto port=freePort(); rocos::MoveServo move(&hw,&model,port);
    REQUIRE(move.Reset()==Result::NoError);
    rocos::Reference ref; REQUIRE(move.GenerateRef(ref)==Result::NoError);
    move.CompleteCycle(Result::NoError);
    UdpClient client;client.session=move.GetSession();client.send(port,jointCommand(0));
    RobotState state{};REQUIRE(client.socket.recv_from(&state,sizeof(state))==sizeof(state));
    CHECK(state.q_d[0]==1);CHECK(state.q_d[1]==2);CHECK(state.q_d[2]==3);
}

TEST_CASE_FIXTURE(ServoFixture,"Servo regression emits terminal fault before closing UDP") {
    exchange(jointCommand(1,.1));
    const auto rc=move.GenerateRef(ref); REQUIRE(rc==Result::HomePositionError);
    move.CompleteCycle(rc);move.Abort();
    RobotState state{}; REQUIRE(client.socket.recv_from(&state,sizeof(state))==sizeof(state));
    CHECK(state.errors[0]);CHECK(state.errors[1]);CHECK_FALSE(state.output_valid);
    CHECK(state.applied_message_id==0);
}

TEST_CASE("Servo communication separates scheduling lateness from missing commands") {
    rocos::ServoCommunication c;
    for(int k=0;k<1000;++k) c.RecordCycle(true,1060000,1000000);
    CHECK(c.SuccessRate()==1);
    CHECK(c.ControlCycles()==1000);
    CHECK(c.ElapsedPeriods()==1060);
    c.Reset();
    for(int k=0;k<1000;++k) c.RecordCycle(true,400000,1000000);
    CHECK(c.ElapsedPeriods()==400);
    CHECK(c.SuccessRate()==1);
    for(int k=0;k<20;++k) c.RecordCycle(false,1000000,1000000);
    CHECK(c.MissingCycles()==20);
    CHECK(c.SuccessRate()==doctest::Approx(.8));
}
