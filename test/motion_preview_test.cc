#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "src/motion_preview.hpp"

#include <future>
#include <limits>

#include "src/model.hpp"
#include "src/move_joint.hpp"
#include "src/move_line_offline.hpp"
#include "src/position_command_validation.hpp"
#include "src/robot_http_server.hpp"
#include "test/doctest.h"

namespace rocos {
struct RobotHttpServerTestAccess {
    static int start(RobotHttpServer& server, ModelInterface& model) {
        server.motion_preview_ = [&model](const auto& request, auto& output) {
            return GenerateMotionPreview(model, request, 0.001, output);
        };
        const int port = server.server_->bind_to_any_port("127.0.0.1");
        server.thread_ = std::make_unique<std::thread>(
            [&] { server.server_->listen_after_bind(); });
        server.server_->wait_until_ready();
        return port;
    }
};
}  // namespace rocos
namespace {
using namespace rocos;
using Json = nlohmann::json;
struct Fixture {
    Model model{PREVIEW_TEST_URDF, "base_link", "link_7"};
    MotionPreviewRequest request;
    MotionPreviewResult output;
    Fixture() {
        request.motion_type = "MoveJ";
        request.start_joints.resize(model.GetJointNum());
        request.joints.resize(model.GetJointNum());
        request.joints(0) = 0.2;
        request.max_points = 31;
    }
};
}  // namespace

TEST_CASE_FIXTURE(
    Fixture, "Preview clone preserves configuration and owns its workspaces") {
    request.start_joints(1) = M_PI / 3;
    request.start_joints(3) = M_PI / 2;
    request.start_joints(5) = -M_PI / 3;
    auto lower = model.GetPosLowerLimit();
    auto upper = model.GetPosUpperLimit();
    upper(0) = 0.5;
    model.SetPosLimits(lower, upper);
    model.SetGravity(Vector(0, -9.81, 0));
    auto copy = model.CloneForPreview();
    CHECK(copy->GetJointNames() == model.GetJointNames());
    CHECK(copy->GetPosUpperLimit().data.isApprox(upper.data));
    CHECK(
        copy->GetVelocityLimit().data.isApprox(model.GetVelocityLimit().data));
    CHECK(copy->GetEffortLimit().data.isApprox(model.GetEffortLimit().data));
    Frame expected;
    REQUIRE(model.ForwardKinematics(request.start_joints, expected) ==
            Result::NoError);
    JntArray zero(model.GetJointNum()), source_torque, clone_torque;
    REQUIRE(model.InverseDynamics(request.start_joints, zero, zero, {},
                                  source_torque) == Result::NoError);
    REQUIRE(copy->InverseDynamics(request.start_joints, zero, zero, {},
                                  clone_torque) == Result::NoError);
    CHECK(clone_torque.data.isApprox(source_torque.data));
    copy->SetGravity(Vector::Zero());
    upper(0) = 0.3;
    copy->SetPosLimits(lower, upper);
    CHECK(model.GetPosUpperLimit()(0) == doctest::Approx(0.5));
    REQUIRE(copy->InverseDynamics(request.start_joints, zero, zero, {},
                                  clone_torque) == Result::NoError);
    CHECK(clone_torque.data.norm() < 1e-10);
    JntArray unchanged;
    REQUIRE(model.InverseDynamics(request.start_joints, zero, zero, {},
                                  unchanged) == Result::NoError);
    CHECK(unchanged.data.isApprox(source_torque.data));
    auto check_fk = [](Model& target, JntArray q, Frame expected_pose) {
        for (int i = 0; i < 100; ++i) {
            Frame actual;
            if (target.ForwardKinematics(q, actual) != Result::NoError ||
                !Equal(actual, expected_pose, 1e-10))
                return false;
        }
        return true;
    };
    auto other = request.start_joints;
    other(0) = 0.2;
    Frame other_pose;
    REQUIRE(copy->ForwardKinematics(other, other_pose) == Result::NoError);
    auto worker = std::async(std::launch::async, check_fk, std::ref(model),
                             request.start_joints, expected);
    CHECK(check_fk(*copy, other, other_pose));
    CHECK(worker.get());
}

TEST_CASE_FIXTURE(
    Fixture,
    "Preview MoveJ matches the existing planner and retains exact endpoints") {
    auto copy = model.CloneForPreview();
    REQUIRE(GenerateMotionPreview(*copy, request, 0.001, output) ==
            Result::NoError);
    REQUIRE(output.points.size() == 31);
    CHECK(output.points.front().time_from_start == 0);
    CHECK(output.points.front().positions(0) == 0);
    CHECK(output.points.back().positions(0) == doctest::Approx(0.2));
    CHECK(output.points.back().time_from_start == output.duration);
    CHECK(output.joint_names == model.GetJointNames());
    MoveJoint original(request.start_joints, request.joints, 1, 2, 10, 0.001);
    REQUIRE(original.Reset() == Result::NoError);
    double time = 0;
    size_t index = 1;
    for (;;) {
        Reference ref;
        const auto result = original.GenerateRef(ref);
        time += 0.001;
        if (index < output.points.size() &&
            std::abs(time - output.points[index].time_from_start) < 1e-8) {
            CHECK(output.points[index].positions(0) ==
                  doctest::Approx(std::get<JntArray>(ref)(0)));
            ++index;
        }
        if (result == Result::PlanFinished) break;
        REQUIRE(result == Result::NoError);
    }
    CHECK(index == output.points.size());
}

TEST_CASE_FIXTURE(Fixture,
                  "No-op succeeds with one sample; invalid input never returns "
                  "partial trajectories") {
    request.joints = request.start_joints;
    REQUIRE(GenerateMotionPreview(model, request, 0.001, output) ==
            Result::NoError);
    CHECK(output.duration == 0);
    REQUIRE(output.points.size() == 1);
    request.joints(0) = 999;
    CHECK(GenerateMotionPreview(model, request, 0.001, output) ==
          Result::PosLimit);
    CHECK(output.points.empty());
    request.joints(0) = std::numeric_limits<double>::quiet_NaN();
    CHECK(GenerateMotionPreview(model, request, 0.001, output) ==
          Result::ParameterNanOrInf);
    request.start_joints.resize(1);
    CHECK(GenerateMotionPreview(model, request, 0.001, output) ==
          Result::UnmatchedJointsNumber);
}

TEST_CASE_FIXTURE(
    Fixture,
    "Preview validates every step, not just downsampled display points") {
    request.velocity = 100;
    request.acceleration = 100000;
    request.jerk = 100000000;
    request.joints(0) = 1;
    request.max_points = 2;
    CHECK(GenerateMotionPreview(model, request, 0.001, output) ==
          Result::SpeedLimit);
    CHECK(output.points.empty());
}

TEST_CASE_FIXTURE(Fixture,
                  "Preview bounds both joint and Cartesian planning work") {
    request.velocity = 0.0001;
    CHECK(GenerateMotionPreview(model, request, 0.001, output) ==
          Result::MotionLimit);
    Frame start;
    REQUIRE(model.ForwardKinematics(request.start_joints, start) ==
            Result::NoError);
    Frame goal = start;
    goal.p.x(goal.p.x() + 0.01);
    MoveLineOffline motion(start, goal, &model, 0.0001, 2, 10, 0.001);
    motion.SetInitialJointPosition(request.start_joints);
    motion.SetPlanningBudget(
        10, std::chrono::steady_clock::now() + std::chrono::seconds(1));
    CHECK(motion.Reset() == Result::MotionLimit);
}

TEST_CASE_FIXTURE(Fixture,
                  "Cartesian previews use isolated model solvers and support "
                  "stationary targets") {
    REQUIRE(model.ForwardKinematics(request.start_joints, request.pose) ==
            Result::NoError);
    for (const std::string type : {"MoveL", "MoveJ_IK", "MoveL_FK"}) {
        request.motion_type = type;
        request.joints = request.start_joints;
        auto copy = model.CloneForPreview();
        CHECK(GenerateMotionPreview(*copy, request, 0.001, output) ==
              Result::NoError);
        CHECK(output.duration == doctest::Approx(0).epsilon(0.001));
    }
    request.motion_type = "MoveL";
    request.pose.p = Vector(100, 100, 100);
    CHECK(GenerateMotionPreview(model, request, 0.001, output) !=
          Result::NoError);
    CHECK(output.points.empty());
}

TEST_CASE_FIXTURE(Fixture,
                  "Moving Cartesian previews reach the requested flange pose") {
    request.start_joints(1) = M_PI / 3;
    request.start_joints(3) = M_PI / 2;
    request.start_joints(5) = -M_PI / 3;
    request.joints = request.start_joints;
    request.joints(0) += 0.02;
    REQUIRE(model.ForwardKinematics(request.joints, request.pose) ==
            Result::NoError);
    request.velocity = 0.05;
    request.acceleration = 0.1;
    request.jerk = 1;
    for (const std::string type : {"MoveL", "MoveJ_IK", "MoveL_FK"}) {
        request.motion_type = type;
        auto copy = model.CloneForPreview();
        REQUIRE(GenerateMotionPreview(*copy, request, 0.001, output) ==
                Result::NoError);
        CHECK(output.duration > 0);
        REQUIRE(output.points.size() > 1);
        const auto error =
            rocos::diff(output.points.back().flange, request.pose);
        CHECK(error.vel.Norm() < 1e-4);
        CHECK(error.rot.Norm() < 1e-3);
    }
}

TEST_CASE_FIXTURE(Fixture,
                  "HTTP preview needs no control token or hardware; motion "
                  "endpoints remain protected") {
    RobotHttpServer server(nullptr);
    const int port = RobotHttpServerTestAccess::start(server, model);
    httplib::Client client("127.0.0.1", port);
    Json body{{"motion_type", "MoveJ"},
              {"start_joints", std::vector<double>(7, 0)},
              {"joints", {0.2, 0, 0, 0, 0, 0, 0}},
              {"max_points", 9}};
    auto post = [&](const Json& input) {
        const auto reply = client.Post("/api/robot/motion/preview",
                                       input.dump(), "application/json");
        REQUIRE(reply);
        REQUIRE(reply->status == 200);
        return Json::parse(reply->body);
    };
    const auto result = post(body);
    REQUIRE(result["success"] == true);
    CHECK(result["data"]["points"].size() == 9);
    CHECK(result["data"]["collision_checked"] == false);
    CHECK(result["data"]["execution_checked"] == false);
    CHECK_FALSE(result["data"].contains("plan_id"));
    for (const Json invalid :
         {Json::array(), Json{{"motion_type", 3}},
          Json{{"motion_type", "MoveJ"}, {"start_joints", "bad"}}})
        CHECK(post(invalid)["success"] == false);
    auto invalid = body;
    invalid["max_points"] = 1;
    CHECK(post(invalid)["success"] == false);
    invalid = body;
    invalid["velocity"] = "fast";
    CHECK(post(invalid)["success"] == false);
    invalid = body;
    invalid["tool_name"] = "ignored";
    CHECK(post(invalid)["success"] == false);
    invalid = body;
    invalid["motion_type"] = "MoveL";
    invalid.erase("joints");
    invalid["pose"] = {
        {"position", {{"x", 0}, {"y", 0}, {"z", 0}}},
        {"orientation", {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 0}}}};
    CHECK(post(invalid)["success"] == false);
    const auto move =
        client.Post("/api/robot/movej", body.dump(), "application/json");
    REQUIRE(move);
    CHECK(Json::parse(move->body)["success"] == false);
    server.stop();
}
