#include "motion_preview.hpp"

#include <algorithm>
#include <chrono>
#include <memory>

#include "move_joint.hpp"
#include "move_line_offline.hpp"
#include "position_command_validation.hpp"

namespace rocos {
Result GenerateMotionPreview(ModelInterface& model, const MotionPreviewRequest& input, double dt,
                             MotionPreviewResult& result) {
    result = {};
    auto request = input;
    if (request.max_points < 2 || request.max_points > 2000 ||
        !std::isfinite(dt) || dt <= 0 || dt > 1 ||
        !std::isfinite(request.velocity) || request.velocity <= 0 ||
        !std::isfinite(request.acceleration) || request.acceleration <= 0 ||
        !std::isfinite(request.jerk) || request.jerk <= 0)
        return Result::IllegalParameter;
    const bool joint_motion =
        request.motion_type == "MoveJ" || request.motion_type == "MoveJ_IK";
    const bool joint_target =
        request.motion_type == "MoveJ" || request.motion_type == "MoveL_FK";
    if (!joint_motion && request.motion_type != "MoveL" &&
        request.motion_type != "MoveL_FK")
        return Result::FunctionNotSupported;
    auto rc = model.ApplyPositionCoupling(request.start_joints);
    if (rc != Result::NoError) return rc;
    rc = ValidateJointCommand(model, request.start_joints, nullptr, dt);
    if (rc != Result::NoError) return rc;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    const auto max_steps =
        static_cast<size_t>(std::min(120000.0, std::floor(120.0 / dt)));
    Frame start_pose, target_pose = request.pose;
    rc = model.ForwardKinematics(request.start_joints, start_pose);
    if (rc != Result::NoError) return rc;
    JntArray target = request.joints;
    if (joint_target) {
        rc = model.ApplyPositionCoupling(target);
        if (rc != Result::NoError) return rc;
        rc = ValidateJointCommand(model, target, nullptr, dt);
        if (rc != Result::NoError) return rc;
        if (!joint_motion) {
            rc = model.ForwardKinematics(target, target_pose);
            if (rc != Result::NoError) return rc;
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(target_pose.p(i)))
                return Result::ParameterNanOrInf;
            for (int j = 0; j < 3; ++j)
                if (!std::isfinite(target_pose.M(i, j)))
                    return Result::ParameterNanOrInf;
        }
        if (joint_motion) {
            target.resize(request.start_joints.rows());
            rc = model.InverseKinematics(request.start_joints, target_pose,
                                         target);
            if (rc != Result::NoError) return rc;
            rc = ValidateJointCommand(model, target, nullptr, dt);
            if (rc != Result::NoError) return rc;
        }
    }
    std::unique_ptr<MotionInterface> motion;
    if (joint_motion) {
        motion = std::make_unique<MoveJoint>(
            request.start_joints, target, request.velocity,
            request.acceleration, request.jerk, dt);
    } else {
        auto linear = std::make_unique<MoveLineOffline>(
            start_pose, target_pose, &model, request.velocity,
            request.acceleration, request.jerk, dt);
        linear->SetInitialJointPosition(request.start_joints);
        linear->SetPlanningBudget(max_steps, deadline);
        motion = std::move(linear);
    }
    rc = motion->Reset();
    if (rc != Result::NoError && rc != Result::PlanFinished) return rc;
    std::vector<JntArray> samples{request.start_joints};
    if (rc != Result::PlanFinished) {
        for (size_t step = 1;; ++step) {
            if (step > max_steps ||
                std::chrono::steady_clock::now() >= deadline)
                return Result::MotionLimit;
            Reference ref;
            rc = motion->GenerateRef(ref);
            if (rc != Result::NoError && rc != Result::PlanFinished) return rc;
            auto* q = std::get_if<JntArray>(&ref);
            if (!q) return Result::MoveInput;
            const auto coupled = model.ApplyPositionCoupling(*q);
            if (coupled != Result::NoError) return coupled;
            const auto valid =
                ValidateJointCommand(model, *q, &samples.back(), dt);
            if (valid != Result::NoError) return valid;
            samples.push_back(*q);
            if (rc == Result::PlanFinished) break;
        }
    }
    // OTG/IK 数值异常不能把未到达目标的部分轨迹报告为成功。
    if (joint_motion) {
        for (unsigned int i = 0; i < target.rows(); ++i)
            if (std::abs(samples.back()(i) - target(i)) > 1e-6)
                return Result::PlanError;
    } else {
        Frame end;
        rc = model.ForwardKinematics(samples.back(), end);
        if (rc != Result::NoError) return rc;
        const auto error = rocos::diff(end, target_pose);
        if (error.vel.Norm() > 1e-4 || error.rot.Norm() > 1e-3)
            return Result::PlanError;
    }
    MotionPreviewResult output;
    output.motion_type = request.motion_type;
    output.joint_names = model.GetJointNames();
    output.duration = (samples.size() - 1) * dt;
    const size_t count =
        std::min(samples.size(), static_cast<size_t>(request.max_points));
    for (size_t i = 0; i < count; ++i) {
        if (std::chrono::steady_clock::now() >= deadline)
            return Result::MotionLimit;
        const size_t index =
            count == 1 ? 0 : i * (samples.size() - 1) / (count - 1);
        MotionPreviewPoint point;
        point.time_from_start = index * dt;
        point.positions = samples[index];
        rc = model.ForwardKinematics(point.positions, point.flange);
        if (rc != Result::NoError) return rc;
        output.points.push_back(std::move(point));
    }
    result = std::move(output);
    return Result::NoError;
}
}  // namespace rocos
