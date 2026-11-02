#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <fstream>

#include "doctest.h"
#include "json.hpp"
#include "src/model.hpp"

TEST_CASE("Model matches recorded KDL FK Jacobian and dynamics") {
    std::ifstream input(ROCOS_MODEL_REFERENCE);
    REQUIRE(input.good());
    const auto reference = nlohmann::json::parse(input);
    rocos::Model model("config/robots/talon/robot.urdf", "base_link", "link_7");
    CHECK(model.GetJointNames() ==
          reference["joint_names"].get<std::vector<std::string>>());
    for (const auto& sample : reference["samples"]) {
        rocos::JntArray q(7), v(7), a(7), tau, gravity, fd;
        for (int i = 0; i < 7; ++i) {
            q(i) = sample["q"][i];
            v(i) = sample["v"][i];
            a(i) = sample["a"][i];
        }
        rocos::Wrenches external(7);
        for (int i = 0; i < 6; ++i) external[6](i) = sample["tip_force"][i];
        rocos::Frame pose;
        rocos::Jacobian jac;
        REQUIRE(model.ForwardKinematics(q, pose) == rocos::Result::NoError);
        REQUIRE(model.GetJacobian(q, jac) == rocos::Result::NoError);
        REQUIRE(model.InverseDynamics(q, v, a, external, tau) ==
                rocos::Result::NoError);
        REQUIRE(model.InverseDynamics(q, rocos::JntArray(7), rocos::JntArray(7),
                                      {}, gravity) == rocos::Result::NoError);
        // 使用记录的输入力矩验证 FD，避免仅检验新后端自己的往返一致性。
        rocos::JntArray input_torque(7);
        for (int i = 0; i < 7; ++i) input_torque(i) = sample["tau"][i];
        REQUIRE(model.ForwardDynamics(q, v, input_torque, external, fd) ==
                rocos::Result::NoError);
        for (int r = 0; r < 3; ++r) {
            CHECK(pose.p(r) ==
                  doctest::Approx(sample["position"][r].get<double>())
                      .epsilon(1e-10));
            for (int c = 0; c < 3; ++c)
                CHECK(
                    pose.M(r, c) ==
                    doctest::Approx(sample["rotation"][r * 3 + c].get<double>())
                        .epsilon(1e-10));
        }
        for (int i = 0; i < 7; ++i) {
            CHECK(
                tau(i) ==
                doctest::Approx(sample["tau"][i].get<double>()).epsilon(1e-9));
            CHECK(gravity(i) ==
                  doctest::Approx(sample["gravity"][i].get<double>())
                      .epsilon(1e-9));
            CHECK(fd(i) ==
                  doctest::Approx(sample["fd"][i].get<double>()).epsilon(1e-8));
            for (int r = 0; r < 6; ++r)
                CHECK(
                    jac(r, i) ==
                    doctest::Approx(sample["jacobian"][r * 7 + i].get<double>())
                        .epsilon(1e-10));
        }
    }
}
