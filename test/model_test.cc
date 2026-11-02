// Copyright 2021, Yang Luo"
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

//#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "src/model.hpp"

#include <test/doctest.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <thread>

namespace {
struct ModelFile {
    std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("rocos_model_test_" + std::to_string(getpid()) + ".urdf");
    explicit ModelFile(const std::string& xml) { std::ofstream(path) << xml; }
    ~ModelFile() { std::filesystem::remove(path); }
};
const char* pendulum = R"(<robot name="pendulum">
  <link name="base"/>
  <link name="arm"><inertial><origin xyz="0.5 0 0"/>
    <mass value="2"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
  </inertial></link>
  <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="2" effort="20"/>
    <hardware><limit lower="-9" upper="9" vel="99" effort="999"/></hardware>
  </joint>
  <link name="tip"/>
  <joint name="fixed_tip" type="fixed"><parent link="arm"/><child link="tip"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>)";
}  // namespace

TEST_CASE("Model uses standard URDF limits and preserves fixed tip dynamics") {
    ModelFile file(pendulum);
    rocos::Model model(file.path.string(), "base", "tip");
    CHECK(model.GetJointNames() == std::vector<std::string>{"hinge"});
    CHECK(model.GetPosLowerLimit()(0) == -1);
    CHECK(model.GetPosUpperLimit()(0) == 1);
    CHECK(model.GetVelocityLimit()(0) == 2);
    CHECK(model.GetEffortLimit()(0) == 20);
    rocos::JntArray q(1), zero(1), tau, acceleration;
    rocos::Frame pose;
    REQUIRE(model.ForwardKinematics(q, pose) == rocos::Result::NoError);
    CHECK(rocos::Equal(pose.p, rocos::Vector(1, 0, 0), 1e-12));
    rocos::Jacobian jac;
    REQUIRE(model.GetJacobian(q, jac) == rocos::Result::NoError);
    CHECK(jac(2, 0) == doctest::Approx(-1));
    CHECK(jac(4, 0) == doctest::Approx(1));
    REQUIRE(model.InverseDynamics(q, zero, zero, {}, tau) ==
            rocos::Result::NoError);
    CHECK(tau(0) == doctest::Approx(-9.81));
    REQUIRE(model.ForwardDynamics(q, zero, tau, {}, acceleration) ==
            rocos::Result::NoError);
    CHECK(acceleration(0) == doctest::Approx(0).epsilon(1e-9));
    // 外力按 base→tip 每个连杆排列，力矩关于该连杆原点，在连杆系表达。
    rocos::Wrenches forces(2);
    forces[1].force = rocos::Vector(0, 0, 3);
    REQUIRE(model.InverseDynamics(q, zero, zero, forces, tau) ==
            rocos::Result::NoError);
    CHECK(tau(0) == doctest::Approx(-6.81));
    REQUIRE(model.ForwardDynamics(q, zero, tau, forces, acceleration) ==
            rocos::Result::NoError);
    CHECK(acceleration(0) == doctest::Approx(0).epsilon(1e-9));
    model.SetGravity(rocos::Vector::Zero());
    rocos::JntArray desired_acceleration(1);
    desired_acceleration(0) = 2;
    REQUIRE(model.InverseDynamics(q, zero, desired_acceleration, {}, tau) ==
            rocos::Result::NoError);
    CHECK(tau(0) == doctest::Approx(1.4));  // Iyy + m * com_x^2 = 0.7
}

TEST_CASE(
    "Model Jacobian matches FK finite differences at nonzero configurations") {
    rocos::Model model("config/robots/talon/robot.urdf", "base_link", "link_7");
    rocos::JntArray q(7);
    q.data << 0.2, -0.4, 0.3, 0.6, -0.2, 0.5, 0.1;
    rocos::Jacobian jac;
    REQUIRE(model.GetJacobian(q, jac) == rocos::Result::NoError);
    const double h = 1e-6;
    for (unsigned int i = 0; i < q.rows(); ++i) {
        auto plus = q, minus = q;
        plus(i) += h;
        minus(i) -= h;
        rocos::Frame a, b;
        REQUIRE(model.ForwardKinematics(minus, a) == rocos::Result::NoError);
        REQUIRE(model.ForwardKinematics(plus, b) == rocos::Result::NoError);
        const auto derivative = rocos::diff(a, b, 2 * h);
        for (unsigned int r = 0; r < 6; ++r)
            CHECK(jac(r, i) == doctest::Approx(derivative(r)).epsilon(1e-6));
    }
}

TEST_CASE(
    "IK keeps bounded joints inside URDF limits and rejects unreachable "
    "targets") {
    ModelFile file(pendulum);
    rocos::Model model(file.path.string(), "base", "tip");
    rocos::JntArray q(1), seed(1), solution(1);
    rocos::Frame target, actual;
    for (double angle : {-0.99, -0.3, 0.2, 0.99}) {
        q(0) = angle;
        seed(0) = angle * 0.95;
        REQUIRE(model.ForwardKinematics(q, target) == rocos::Result::NoError);
        REQUIRE(model.InverseKinematics(seed, target, solution) ==
                rocos::Result::NoError);
        CHECK(solution(0) >= -1);
        CHECK(solution(0) <= 1);
        REQUIRE(model.ForwardKinematics(solution, actual) ==
                rocos::Result::NoError);
        CHECK((actual.p - target.p).Norm() < 2e-5);
        CHECK((actual.M.Inverse() * target.M).GetRot().Norm() < 2e-5);
    }
    q(0) = 2;
    REQUIRE(model.ForwardKinematics(q, target) == rocos::Result::NoError);
    solution(0) = 0.123;
    CHECK(model.InverseKinematics(seed, target, solution) ==
          rocos::Result::IkCalcFail);
    CHECK(solution(0) == 0.123);
}

TEST_CASE("Model rejects invalid numerical input without replacing output") {
    ModelFile file(pendulum);
    rocos::Model model(file.path.string(), "base", "tip");
    rocos::JntArray q(1), out(1), empty;
    out(0) = 42;
    rocos::Frame pose(rocos::Vector(7, 8, 9));
    CHECK(model.ForwardKinematics(empty, pose) == rocos::Result::FkCalcFail);
    CHECK(pose.p.x() == 7);
    q(0) = std::numeric_limits<double>::quiet_NaN();
    CHECK(model.InverseDynamics(q, q, q, {}, out) == rocos::Result::IdCalcFail);
    CHECK(out(0) == 42);
    q(0) = 0;
    CHECK(model.ForwardDynamics(q, empty, q, {}, out) ==
          rocos::Result::FdCalcFail);
    CHECK(model.InverseDynamics(q, q, q, rocos::Wrenches(1), out) ==
          rocos::Result::IdCalcFail);
    rocos::Frame bad;
    bad.M(0, 0) = 2;
    CHECK(model.InverseKinematics(q, bad, out) == rocos::Result::IkCalcFail);
    CHECK_FALSE(model.ParseUrdf("<robot/>", "base", "tip"));
    CHECK(model.ForwardKinematics(q, pose) == rocos::Result::NoError);
    CHECK(rocos::Equal(pose.p, rocos::Vector(1, 0, 0), 1e-12));
}

TEST_CASE("Model extracts a non-root base and keeps fixed-link inertia") {
    const std::string xml = R"(<robot name="subchain">
      <link name="world"/><link name="base"/><link name="arm"/>
      <link name="tip"><inertial><mass value="2"/>
        <inertia ixx="0.1" iyy="0.2" izz="0.2" ixy="0" ixz="0" iyz="0"/>
      </inertial></link><link name="unrelated"/>
      <joint name="mount" type="fixed"><parent link="world"/><child link="base"/>
        <origin xyz="3 4 5" rpy="0.2 0.3 0.4"/></joint>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
        <axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="2" effort="20"/></joint>
      <joint name="fixed_tip" type="fixed"><parent link="arm"/><child link="tip"/>
        <origin xyz="0.5 0 0" rpy="0.3 0 0"/></joint>
      <joint name="other" type="revolute"><parent link="world"/><child link="unrelated"/>
        <limit lower="-1" upper="1" velocity="2" effort="20"/></joint>
    </robot>)";
    ModelFile file(xml);
    rocos::Model model(file.path.string(), "base", "tip");
    rocos::JntArray zero(1), tau;
    rocos::Frame pose;
    CHECK(model.GetJointNum() == 1);
    REQUIRE(model.ForwardKinematics(zero, pose) == rocos::Result::NoError);
    CHECK(rocos::Equal(
        pose,
        rocos::Frame(rocos::Rotation::RotX(0.3), rocos::Vector(0.5, 0, 0)),
        1e-12));
    REQUIRE(model.InverseDynamics(zero, zero, zero, {}, tau) ==
            rocos::Result::NoError);
    CHECK(tau(0) == doctest::Approx(-9.81));
    CHECK_THROWS(rocos::Model(file.path.string(), "unrelated", "tip"));
}

TEST_CASE(
    "Continuous configurations preserve scalar winding and prismatic "
    "coordinates") {
    ModelFile file(
        R"(<robot name="mixed"><link name="base"/><link name="a"/><link name="tip"/>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="a"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="2" effort="20"/></joint>
      <joint name="spin" type="continuous"><parent link="a"/><child link="tip"/>
        <axis xyz="0 0 1"/><limit velocity="2" effort="20"/></joint></robot>)");
    rocos::Model model(file.path.string(), "base", "tip");
    rocos::JntArray q(2), seed(2), result;
    q(0) = 0.3;
    q(1) = 6 * M_PI + 0.2;
    seed(0) = 0.29;
    seed(1) = 6 * M_PI + 0.19;
    rocos::Frame target;
    REQUIRE(model.ForwardKinematics(q, target) == rocos::Result::NoError);
    CHECK(rocos::Equal(target.p, rocos::Vector(0.3, 0, 0), 1e-12));
    REQUIRE(model.InverseKinematics(seed, target, result) ==
            rocos::Result::NoError);
    CHECK(result(0) == doctest::Approx(q(0)).epsilon(2e-5));
    CHECK(std::abs(result(1) - q(1)) < 2e-5);
    rocos::Jacobian jac;
    REQUIRE(model.GetJacobian(q, jac) == rocos::Result::NoError);
    CHECK(jac(0, 0) == doctest::Approx(1));
    CHECK(jac(5, 1) == doctest::Approx(1));
}

TEST_CASE(
    "Talon IK converges from nearby seeds across deterministic "
    "configurations") {
    rocos::Model model("config/robots/talon/robot.urdf", "base_link", "link_7");
    std::mt19937 random(20261102);
    std::uniform_real_distribution<double> sample(-0.8, 0.8);
    for (int k = 0; k < 40; ++k) {
        rocos::JntArray q(7), seed(7), result;
        for (int j = 0; j < 7; ++j) {
            q(j) = sample(random);
            seed(j) = q(j) + 0.01;
        }
        rocos::Frame target, actual;
        REQUIRE(model.ForwardKinematics(q, target) == rocos::Result::NoError);
        REQUIRE(model.InverseKinematics(seed, target, result) ==
                rocos::Result::NoError);
        REQUIRE(model.ForwardKinematics(result, actual) ==
                rocos::Result::NoError);
        CHECK((actual.p - target.p).Norm() <= 2e-5);
        CHECK((actual.M.Inverse() * target.M).GetRot().Norm() <= 2e-5);
        for (int j = 0; j < 7; ++j) {
            CHECK(result(j) >= model.GetPosLowerLimit()(j));
            CHECK(result(j) <= model.GetPosUpperLimit()(j));
        }
    }
}

TEST_CASE("Model serializes independent FK and IK callers") {
    ModelFile file(pendulum);
    rocos::Model model(file.path.string(), "base", "tip");
    std::atomic<bool> ok{true};
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t)
        threads.emplace_back([&, t] {
            rocos::JntArray q(1), result;
            q(0) = 0.1 * t;
            for (int k = 0; k < 20; ++k) {
                rocos::Frame f;
                if (model.ForwardKinematics(q, f) != rocos::Result::NoError ||
                    std::abs(f.p.x() - std::cos(q(0))) > 1e-10 ||
                    model.InverseKinematics(q, f, result) !=
                        rocos::Result::NoError)
                    ok = false;
            }
        });
    for (auto& thread : threads) thread.join();
    CHECK(ok.load());
}

// talon URDF 的限位在标准 <joint>/<limit> 标签内。
TEST_CASE("Model - ParseUrdf with talon robot") {
    const std::string urdf_path = "config/robots/talon/robot.urdf";
    const std::string base_link = "base_link";
    const std::string tip_link = "link_7";

    rocos::Model model(urdf_path, base_link, tip_link);

    SUBCASE("关节链包含 7 个活动关节") { CHECK(model.GetJointNum() == 7u); }

    SUBCASE("按 model joint order 暴露 URDF joint limit") {
        const auto& q_min = model.GetPosLowerLimit();
        const auto& q_max = model.GetPosUpperLimit();
        const auto& q_vel = model.GetVelocityLimit();
        const auto& q_effort = model.GetEffortLimit();
        REQUIRE(q_min.rows() == 7u);
        REQUIRE(q_max.rows() == 7u);
        REQUIRE(q_vel.rows() == 7u);
        REQUIRE(q_effort.rows() == 7u);

        const double expected_lower[] = {-2.84, -2.22, -2.84, -2.23,
                                         -2.71, -2.11, -2.84};
        const double expected_upper[] = {2.84, 2.22, 2.84, 2.23,
                                         2.71, 2.11, 3.14};
        for (unsigned int i = 0; i < 7u; ++i) {
            CHECK(q_min(i) == doctest::Approx(expected_lower[i]).epsilon(1e-9));
            CHECK(q_max(i) == doctest::Approx(expected_upper[i]).epsilon(1e-9));
            CHECK(q_vel(i) == doctest::Approx(3.0).epsilon(1e-9));
            CHECK(q_effort(i) == doctest::Approx(30.0).epsilon(1e-9));
        }
    }

    SUBCASE("零位 FK 返回有效位姿") {
        rocos::JntArray q_zero(7);
        rocos::SetToZero(q_zero);
        rocos::Frame p_out;

        const int rc = static_cast<int>(model.ForwardKinematics(q_zero, p_out));
        CHECK(rc == 0);  // NoError = 0
        // 验证旋转矩阵正交性：X·X = 1, X·Y = 0
        const rocos::Vector col_x = p_out.M.UnitX();
        const rocos::Vector col_y = p_out.M.UnitY();
        CHECK(rocos::dot(col_x, col_x) == doctest::Approx(1.0).epsilon(1e-9));
        CHECK(rocos::dot(col_x, col_y) == doctest::Approx(0.0).epsilon(1e-9));
    }

    SUBCASE("IK 与 FK 互为逆运算") {
        rocos::JntArray q_init(7);
        rocos::SetToZero(q_init);

        rocos::Frame target;
        REQUIRE(static_cast<int>(model.ForwardKinematics(q_init, target)) == 0);

        rocos::JntArray q_solution(7);
        const int ik_rc = static_cast<int>(
            model.InverseKinematics(q_init, target, q_solution));
        CHECK(ik_rc == 0);

        // IK 解代回 FK，末端位置应与目标吻合
        rocos::Frame p_verify;
        REQUIRE(static_cast<int>(
                    model.ForwardKinematics(q_solution, p_verify)) == 0);
        for (int r = 0; r < 3; ++r) {
            CHECK(p_verify.p(r) == doctest::Approx(target.p(r)).epsilon(1e-4));
        }
    }
}
