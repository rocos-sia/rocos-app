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

#include <test/doctest.h>

#include "src/model.hpp"

// talon URDF 的限位在自定义 <hardware>/<limit> 标签内。
TEST_CASE("Model - ParseUrdf with talon robot") {
    const std::string urdf_path = "config/robots/talon/robot.urdf";
    const std::string base_link = "base_link";
    const std::string tip_link  = "link_7";

    rocos::Model model(urdf_path, base_link, tip_link);

    SUBCASE("关节链包含 7 个活动关节") {
        CHECK(model.GetChain().getNrOfJoints() == 7u);
    }

    SUBCASE("按 model joint order 暴露 URDF hardware limit") {
        const auto& q_min = model.GetPosLowerLimit();
        const auto& q_max = model.GetPosUpperLimit();
        const auto& q_vel = model.GetVelocityLimit();
        const auto& q_effort = model.GetEffortLimit();
        REQUIRE(q_min.rows() == 7u);
        REQUIRE(q_max.rows() == 7u);
        REQUIRE(q_vel.rows() == 7u);
        REQUIRE(q_effort.rows() == 7u);

        const double expected_lower[] = {-2.84, -2.22, -2.84, -2.23, -2.71, -2.11, -2.84};
        const double expected_upper[] = { 2.84,  2.22,  2.84,  2.23,  2.71,  2.11,  3.14};
        for (unsigned int i = 0; i < 7u; ++i) {
            CHECK(q_min(i) == doctest::Approx(expected_lower[i]).epsilon(1e-9));
            CHECK(q_max(i) == doctest::Approx(expected_upper[i]).epsilon(1e-9));
            CHECK(q_vel(i) == doctest::Approx(3.0).epsilon(1e-9));
            CHECK(q_effort(i) == doctest::Approx(30.0).epsilon(1e-9));
        }
    }

    SUBCASE("零位 FK 返回有效位姿") {
        KDL::JntArray q_zero(7);
        KDL::SetToZero(q_zero);
        KDL::Frame p_out;

        const int rc = static_cast<int>(model.ForwardKinematics(q_zero, p_out));
        CHECK(rc == 0);  // NoError = 0
        // 验证旋转矩阵正交性：X·X = 1, X·Y = 0
        const KDL::Vector col_x = p_out.M.UnitX();
        const KDL::Vector col_y = p_out.M.UnitY();
        CHECK(KDL::dot(col_x, col_x) == doctest::Approx(1.0).epsilon(1e-9));
        CHECK(KDL::dot(col_x, col_y) == doctest::Approx(0.0).epsilon(1e-9));
    }

    SUBCASE("IK 与 FK 互为逆运算") {
        KDL::JntArray q_init(7);
        KDL::SetToZero(q_init);

        KDL::Frame target;
        REQUIRE(static_cast<int>(model.ForwardKinematics(q_init, target)) == 0);

        KDL::JntArray q_solution(7);
        const int ik_rc = static_cast<int>(model.InverseKinematics(q_init, target, q_solution));
        CHECK(ik_rc == 0);

        // IK 解代回 FK，末端位置应与目标吻合
        KDL::Frame p_verify;
        REQUIRE(static_cast<int>(model.ForwardKinematics(q_solution, p_verify)) == 0);
        for (int r = 0; r < 3; ++r) {
            CHECK(p_verify.p(r) == doctest::Approx(target.p(r)).epsilon(1e-4));
        }
    }
}
