#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "src/model.hpp"
#include "src/move_jog.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using namespace rocos;
constexpr double kDt = 0.001;
constexpr double kSpeed = 0.2;
const Frame kTool(Rotation::RPY(0.3, -0.4, 0.5), Vector(0.12, -0.08, 0.2));

JntArray home() {
    JntArray q(7);
    q.data << 0.0, 1.0471975512, 0.0, 1.5707963268, 0.0, -1.0471975512, 0.0;
    return q;
}

Frame fk(Model& model, const JntArray& q) {
    Frame pose;
    REQUIRE(model.ForwardKinematics(q, pose) == Result::NoError);
    return pose;
}

struct RotationRun {
    double drift{0}, angle{0}, steady_speed{0};
    bool stopped{false};
};

RotationRun rotate(Model& model, int axis, bool compensated, bool timeout) {
    MoveJog jog(kDt, 0.1, &model);
    JntArray q = home();
    jog.setInitialPosition(q);
    REQUIRE(jog.Reset() == Result::NoError);
    const Frame start = fk(model, q) * kTool;
    Vector angular;
    angular(axis) = 1.0;
    const Twist direction(Vector::Zero(), start.M * angular);
    const Vector offset = compensated ? kTool.p : Vector::Zero();
    RotationRun run;
    for (int step = 0; step < 4000; ++step) {
        if (step < 1200 && step % 20 == 0)
            REQUIRE(jog.FeedJog(direction, kSpeed, offset) == Result::NoError);
        if (step == 1200 && !timeout) REQUIRE(jog.Stop() == Result::NoError);
        Reference reference;
        const Result result = jog.GenerateRef(reference);
        REQUIRE((result == Result::NoError || result == Result::PlanFinished));
        const JntArray next = std::get<JntArray>(reference);
        if (step == 1000) {
            Jacobian jacobian;
            REQUIRE(model.GetJacobian(q, jacobian) == Result::NoError);
            const Eigen::VectorXd velocity = jacobian.data * ((next.data - q.data) / kDt);
            run.steady_speed = velocity.tail<3>().dot(direction.rot.data);
        }
        q = next;
        const Frame tcp = fk(model, q) * kTool;
        run.drift = std::max(run.drift, (tcp.p - start.p).Norm());
        run.angle = (start.M.Inverse() * tcp.M).GetRot().Norm();
        if (result == Result::PlanFinished) {
            run.stopped = true;
            break;
        }
    }
    return run;
}
}

TEST_CASE("TOOL rotation holds an offset TCP through explicit and watchdog stops") {
    Model model("config/robots/talon/robot.urdf", "base_link", "link_7");
    for (int axis = 0; axis < 3; ++axis) {
        for (bool timeout : {false, true}) {
            CAPTURE(axis);
            CAPTURE(timeout);
            const auto run = rotate(model, axis, true, timeout);
            MESSAGE("TCP drift = ", run.drift, " m");
            CHECK(run.stopped);
            CHECK(run.angle > 0.15);
            CHECK(run.drift < 1e-4);
            CHECK(run.steady_speed == doctest::Approx(kSpeed).epsilon(1e-6));
        }
    }
    // 对照原法兰旋转行为，确认非零偏置足以暴露回归。
    const auto flange_rotation = rotate(model, 0, false, false);
    CHECK(flange_rotation.drift > 0.01);
}

TEST_CASE("TCP translation and mixed twists preserve requested direction and speed") {
    Model model("config/robots/talon/robot.urdf", "base_link", "link_7");
    for (const Twist& direction : {
             Twist(Vector(1, 0, 0), Vector::Zero()),
             Twist(Vector(0.2, -0.3, 0.1), Vector(0.4, 0.1, -0.2))}) {
        MoveJog jog(kDt, 0.1, &model);
        JntArray q = home();
        jog.setInitialPosition(q);
        REQUIRE(jog.Reset() == Result::NoError);
        const double norm = std::sqrt(direction.vel.data.squaredNorm() +
                                      direction.rot.data.squaredNorm());
        for (int step = 0; step < 1000; ++step) {
            if (step % 20 == 0)
                REQUIRE(jog.FeedJog(direction, 0.03, kTool.p) == Result::NoError);
            Reference reference;
            REQUIRE(jog.GenerateRef(reference) == Result::NoError);
            const JntArray next = std::get<JntArray>(reference);
            if (step > 800) {
                Jacobian jacobian;
                REQUIRE(model.GetJacobian(q, jacobian) == Result::NoError);
                const Eigen::VectorXd velocity = jacobian.data * ((next.data - q.data) / kDt);
                const Vector angular(velocity.tail<3>());
                const Vector linear = Vector(velocity.head<3>()) +
                                       angular * (fk(model, q).M * kTool.p);
                CHECK((linear - direction.vel * (0.03 / norm)).Norm() < 1e-9);
                CHECK((angular - direction.rot * (0.03 / norm)).Norm() < 1e-9);
            }
            q = next;
        }
    }
}

TEST_CASE("Zero offset retains flange motion and a jog cannot change reference point") {
    Model model("config/robots/talon/robot.urdf", "base_link", "link_7");
    MoveJog legacy(kDt, 0.1, &model), explicit_zero(kDt, 0.1, &model);
    for (auto* jog : {&legacy, &explicit_zero}) {
        jog->setInitialPosition(home());
        REQUIRE(jog->Reset() == Result::NoError);
    }
    const Twist direction(Vector::Zero(), Vector(0, 0, 1));
    for (int step = 0; step < 500; ++step) {
        REQUIRE(legacy.FeedJog(direction, kSpeed) == Result::NoError);
        REQUIRE(explicit_zero.FeedJog(direction, kSpeed, Vector::Zero()) == Result::NoError);
        Reference a, b;
        REQUIRE(legacy.GenerateRef(a) == Result::NoError);
        REQUIRE(explicit_zero.GenerateRef(b) == Result::NoError);
        CHECK((std::get<JntArray>(a).data - std::get<JntArray>(b).data).norm() == 0.0);
    }
    CHECK(legacy.FeedJog(direction, kSpeed, kTool.p) == Result::IllegalParameter);
    REQUIRE(legacy.Reset() == Result::NoError);
    REQUIRE(legacy.FeedJog(direction, kSpeed, kTool.p) == Result::NoError);
    CHECK(legacy.FeedJog(direction, kSpeed) == Result::IllegalParameter);
    CHECK(legacy.FeedJog(direction, kSpeed, Vector(std::numeric_limits<double>::quiet_NaN(), 0, 0))
          == Result::ParameterNanOrInf);
}

TEST_CASE("TCP compensation propagates kinematic failures without publishing a target") {
    class FailingModel : public Model {
    public:
        using Model::Model;
        Result ForwardKinematics(const JntArray&, Frame&) override { return Result::FkCalcFail; }
    } model("config/robots/talon/robot.urdf", "base_link", "link_7");
    MoveJog jog(kDt, 0.1, &model);
    jog.setInitialPosition(home());
    REQUIRE(jog.Reset() == Result::NoError);
    REQUIRE(jog.FeedJog(Twist(Vector::Zero(), Vector(0, 0, 1)), kSpeed, kTool.p) == Result::NoError);
    Reference reference = home();
    CHECK(jog.GenerateRef(reference) == Result::FkCalcFail);
    CHECK((std::get<JntArray>(reference).data - home().data).norm() == 0.0);
}
