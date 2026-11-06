#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <yaml-cpp/yaml.h>

#include "examples/dynamics_id/excitation_trajectory.hpp"
#include "examples/dynamics_id/identified_joint_impedance.hpp"
#include "test/doctest.h"

using namespace rocos;
using namespace rocos::experiment;
namespace {
std::string fixture(const std::string& name) {
    return std::string(IDENTIFICATION_TEST_DATA) + "/" + name;
}
Eigen::VectorXd vector(const YAML::Node& node) {
    Eigen::VectorXd result(node.size());
    for (size_t i = 0; i < node.size(); ++i) result[i] = node[i].as<double>();
    return result;
}
}  // namespace

TEST_CASE("Excitation matches Studio analytic reference and endpoints") {
    const auto hardware = Hardware::LoadConfigFromYAML(fixture("hardware.yaml"));
    ExcitationTrajectory trajectory(fixture("trajectory.json"), hardware);
    trajectory.validate(1, .001);
    REQUIRE(trajectory.drives[0].id == 0);
    REQUIRE(hardware.drives[0].id == 1);
    const auto cases = YAML::LoadFile(fixture("trajectory_reference.json"));
    for (const auto& sample : cases) {
        for (size_t j = 0; j < trajectory.names.size(); ++j) {
            const auto actual = evaluateFourier(trajectory.a[j], trajectory.b[j], 0,
                                                sample["time"].as<double>(), trajectory.duration);
            CHECK(actual[0] == doctest::Approx(sample["q"][j].as<double>()).epsilon(1e-10));
            CHECK(actual[1] == doctest::Approx(sample["dq"][j].as<double>()).epsilon(1e-10));
            CHECK(actual[2] == doctest::Approx(sample["ddq"][j].as<double>()).epsilon(1e-10));
        }
    }
    CHECK_THROWS(trajectory.validate(0, .001));
    trajectory.a[0][0] = 100;
    CHECK_THROWS(trajectory.validate(1, .001));
}

TEST_CASE("Hardware bindings reject ambiguous names and unrelated drives") {
    auto hardware = Hardware::LoadConfigFromYAML(fixture("hardware.yaml"));
    CHECK_THROWS(orderedDrives(hardware, {"joint_1"}));
    CHECK_THROWS(orderedDrives(hardware, {"joint_1", "joint_1"}));
    hardware.drives[0].id = hardware.drives[1].id;
    CHECK_THROWS(orderedDrives(hardware, {"joint_1", "joint_2"}));
}

TEST_CASE("Acquisition checks position steps against hardware period and stricter velocity") {
    const auto hardware = Hardware::LoadConfigFromYAML(fixture("hardware.yaml"));
    ExcitationTrajectory trajectory(fixture("trajectory.json"), hardware);
    JntArray previous(2), target(2);
    previous.data.setZero();
    target.data.setZero();
    auto check = [&](double dt = .001) {
        validatePositionCommand(target, previous, trajectory.drives, trajectory.velocity, dt);
    };
    CHECK_NOTHROW(check()); // 首次预置使用实测位置，无首帧豁免。
    for (double sign : {-1., 1.}) {
        target(0) = sign * .002;
        CHECK_NOTHROW(check()); // 恰好等于 Studio 的 2 rad/s 上限。
        target(0) = sign * .0021;
        CHECK_THROWS_AS(check(), std::runtime_error);
    }
    // 即使外部速度上限更大，也不能越过硬件的 3 rad/s。
    trajectory.velocity[0] = 10;
    target(0) = .0031;
    CHECK_THROWS_AS(check(), std::runtime_error);
    target.data.setZero();
    for (double dt : {0., -1., std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()}) CHECK_THROWS(check(dt));
    for (double value : {std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        target(1) = value;
        CHECK_THROWS_WITH(check(), "Nonfinite position command: joint_2");
    }
    target.data.setZero();
    previous(1) = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS(check());
    previous.data.setZero();
    target(1) = 2.1;
    CHECK_THROWS_WITH(check(), "Command position limit exceeded: joint_2");
    // 过渡/保持允许位于激励范围外，但必须满足硬件限位。
    previous(1) = target(1) = 1.5;
    CHECK_NOTHROW(check());
    target.resize(1);
    CHECK_THROWS_WITH(check(), "Position command dimension mismatch");
}

TEST_CASE("Identified rigid and friction torque matches Studio, not nominal URDF") {
    IdentifiedJointImpedanceController controller(fixture("controller.yaml"));
    const auto cases = YAML::LoadFile(fixture("reference.json"));
    for (const auto& sample : cases) {
        auto q = vector(sample["q"]), dq = vector(sample["dq"]), target = vector(sample["target"]);
        const Eigen::VectorXd actual = controller.computeTorque(q, dq, target);
        CHECK((actual - vector(sample["torque"])).norm() < 1e-9);
        CHECK((controller.rigidTorque() - vector(sample["rigid"])).norm() < 1e-9);
        const Eigen::VectorXd friction = controller.frictionTorque();
        const Eigen::VectorXd without = controller.computeTorque(q, dq, target, {}, false);
        CHECK((actual - without - friction).norm() < 1e-9);
        CHECK(controller.frictionTorque().norm() == 0);
    }
    const auto zero = Eigen::VectorXd::Zero(2).eval();
    auto speed = zero;
    for (double sign : {-1., 1.}) {
        speed[0] = sign * controller.config().velocityLimit[0];
        CHECK_NOTHROW(controller.computeTorque(zero, speed, zero));
        speed[0] *= 1.001;
        CHECK_THROWS(controller.computeTorque(zero, speed, zero));
    }
    CHECK_THROWS(controller.computeTorque(Eigen::VectorXd::Zero(1), zero, zero));
    auto target = zero;
    target[0] = 3;
    CHECK_THROWS(controller.computeTorque(zero, zero, target));
}

TEST_CASE("Torque slew limit uses elapsed seconds and preserves magnitude limit") {
    Eigen::VectorXd requested(2), magnitude(2), rate(2), sent(2);
    requested << 100, -100;
    magnitude << 3, 4;
    rate << 10, 20;
    sent << 0, 0;
    limitTorque(requested, magnitude, rate, .02, sent);
    CHECK(sent[0] == doctest::Approx(.2));
    CHECK(sent[1] == doctest::Approx(-.4));
    limitTorque(requested, magnitude, rate, 1, sent);
    CHECK(sent[0] == 3);
    CHECK(sent[1] == -4);
}
