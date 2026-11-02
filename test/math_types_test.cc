#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <cmath>

#include "doctest.h"
#include "src/types.hpp"

using namespace rocos;

TEST_CASE("App geometry preserves base-frame pose and twist conventions") {
    const Frame base(Rotation::RotZ(M_PI_2), Vector(1, 2, 3));
    const Frame tool(Rotation::RotX(0.3), Vector(0.2, 0, 0));
    const auto tcp = base * tool;
    CHECK(Equal(tcp.p, Vector(1, 2.2, 3), 1e-12));
    CHECK(Equal(base.Inverse() * tcp, tool, 1e-12));
    CHECK(Equal(base * base.Inverse(), Frame::Identity(), 1e-12));
    const auto error =
        diff(base, Frame(Rotation::RotZ(M_PI_2 + 0.2), Vector(2, 2, 3)));
    CHECK(Equal(error.vel, Vector(1, 0, 0), 1e-12));
    CHECK(Equal(error.rot, Vector(0, 0, 0.2), 1e-12));
}

TEST_CASE("App rotation handles quaternion equivalence and pi rotations") {
    const auto r = Rotation::RPY(0.4, -0.5, 1.2);
    double x, y, z, w;
    r.GetQuaternion(x, y, z, w);
    CHECK(Equal(r, Rotation::Quaternion(x, y, z, w), 1e-12));
    CHECK(Equal(r, Rotation::Quaternion(-x, -y, -z, -w), 1e-12));
    for (double angle : {0.0, 1e-10, M_PI - 1e-8, M_PI}) {
        const auto rotation = Rotation::Rot(Vector(1, 2, 3), angle);
        const auto rot = rotation.GetRot();
        CHECK(Equal(rotation, Rotation::Rot(rot, rot.Norm()), 1e-12));
    }
    const Rotation row_major(0, -1, 0, 1, 0, 0, 0, 0, 1);
    CHECK(Equal(row_major * Vector(1, 0, 0), Vector(0, 1, 0), 1e-12));
}

TEST_CASE(
    "App joint and Jacobian storage uses linear then angular components") {
    JntArray q(2);
    CHECK(q.data.isZero());
    q(0) = 2;
    q(1) = -1;
    Jacobian j(2);
    j.setColumn(0, Twist(Vector(1, 0, 0), Vector(0, 0, 1)));
    j.setColumn(1, Twist(Vector(0, 1, 0), Vector(0, 1, 0)));
    const Eigen::VectorXd v = j.data * q.data;
    CHECK(v(0) == 2);
    CHECK(v(1) == -1);
    CHECK(v(4) == -1);
    CHECK(v(5) == 2);
}
