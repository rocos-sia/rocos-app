// Copyright 2026, ROCOS contributors
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <cmath>
#include <filesystem>
#include <string>

#include "doctest.h"
#include "model.hpp"

namespace {
const auto kModels = std::filesystem::path(ROCOS_DUALARM_MODEL_DIR);
}

TEST_CASE("Shoulder bases retain seven axes and equivalent FK, IK and dynamics") {
  for (const auto& side : {std::string("left"), std::string("right")}) {
    const std::string prefix = side == "left" ? "Left" : "Right";
    rocos::Model original((kModels / "dualarm.urdf").string(), "base_link",
                          prefix + "_Link7");
    rocos::Model trial((kModels / "dualarm_shoulder.urdf").string(),
                       side + "_base", prefix + "_Link7");
    REQUIRE(original.GetJointNum() == 7);
    REQUIRE(trial.GetJointNum() == 7);
    CHECK(trial.GetJointNames() == original.GetJointNames());
    CHECK(trial.GetJointNames().front() == prefix + "_Joint1");
    const rocos::Frame mount(rocos::Rotation::Identity(),
        rocos::Vector(0.011578, side == "left" ? -0.063 : -0.267, 0.091501));
    const rocos::Frame tool(rocos::Rotation::RPY(0.1, -0.2, 0.3),
                            rocos::Vector(0.02, 0.01, 0.13));
    for (int sample = 0; sample < 12; ++sample) {
      INFO(side << " sample " << sample);
      rocos::JntArray q(7), seed(7), v(7), acceleration(7), tau_old, tau_new;
      for (int j = 0; j < 7; ++j) {
        q(j) = 0.55 * std::sin(0.6 * sample + 0.41 * j);
        seed(j) = q(j) + 0.015 * std::cos(sample + j);
        v(j) = 0.1 * std::cos(sample + j);
        acceleration(j) = 0.05 * std::sin(sample + j);
        CHECK(trial.GetPosLowerLimit()(j) == original.GetPosLowerLimit()(j));
        CHECK(trial.GetPosUpperLimit()(j) == original.GetPosUpperLimit()(j));
      }
      rocos::Frame old_flange, new_flange;
      REQUIRE(original.ForwardKinematics(q, old_flange) == rocos::Result::NoError);
      REQUIRE(trial.ForwardKinematics(q, new_flange) == rocos::Result::NoError);
      CHECK(rocos::Equal(mount * new_flange, old_flange, 1e-10));
      REQUIRE(original.InverseDynamics(q, v, acceleration, {}, tau_old) ==
              rocos::Result::NoError);
      REQUIRE(trial.InverseDynamics(q, v, acceleration, {}, tau_new) ==
              rocos::Result::NoError);
      CHECK((tau_old.data - tau_new.data).norm() < 1e-9);
      // Convert an old-base TCP target to the selected base, then to flange.
      const auto tcp_target = mount.Inverse() * (old_flange * tool);
      const auto flange_target = tcp_target * tool.Inverse();
      rocos::JntArray solution;
      REQUIRE(trial.InverseKinematics(seed, flange_target, solution) ==
              rocos::Result::NoError);
      rocos::Frame reached;
      REQUIRE(trial.ForwardKinematics(solution, reached) == rocos::Result::NoError);
      CHECK(rocos::Equal(mount * reached * tool, old_flange * tool, 2e-4));
      // Changing only J1 must still move the flange in the selected chain.
      q(0) += 0.1;
      rocos::Frame moved;
      REQUIRE(trial.ForwardKinematics(q, moved) == rocos::Result::NoError);
      CHECK_FALSE(rocos::Equal(moved, new_flange, 1e-5));
    }
  }
}

TEST_CASE("Both shoulder J1 axes follow the mounting disk normal") {
  for (const std::string side : {"left", "right"}) {
    const std::string prefix = side == "left" ? "Left" : "Right";
    rocos::Model model((kModels / "dualarm_shoulder.urdf").string(),
                       side + "_base", prefix + "_Link7");
    rocos::JntArray q(7);
    q.data.setZero();
    rocos::Jacobian jacobian(7);
    REQUIRE(model.GetJacobian(q, jacobian) == rocos::Result::NoError);
    // Angular velocity for +J1 is about base +Y, not the displayed base Z.
    CHECK(std::abs(jacobian(3, 0)) < 1e-4);
    CHECK(std::abs(jacobian(4, 0) - 1) < 1e-4);
    CHECK(std::abs(jacobian(5, 0)) < 1e-4);
  }
}
