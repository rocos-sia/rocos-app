#pragma once
#include "hardware_interface.hpp"
#include "position_command_validation.hpp"

namespace rocos {
// 反馈保护与指令差分分开；跟随误差使用显式容差，而非单周期速度检查。
inline Result ValidateFeedback(const ModelInterface& model,
                               HardwareInterface& hw,
                               const FeedbackSnapshot& sample,
                               const JntArray* previous_command = nullptr) {
    const double dt = hw.GetDt() * 1e-6;
    auto rc = ValidateJointCommand(model, sample.position, nullptr, dt);
    if (rc != Result::NoError) return rc;
    if (sample.velocity.rows() != sample.position.rows())
        return Result::UnmatchedJointsNumber;
    rc = hw.CheckFeedbackFreshness(uint64_t(hw.GetDt()) * 20);
    if (rc != Result::NoError) return rc;
    const auto limits = hw.GetJointDerivativeLimits();
    for (unsigned int i = 0; i < sample.position.rows(); ++i) {
        if (!std::isfinite(sample.velocity(i)))
            return Result::ParameterNanOrInf;
        if (std::abs(sample.velocity(i)) > model.GetVelocityLimit()(i))
            return Result::SpeedLimit;
        const double tolerance = limits.size() == sample.position.rows() &&
                                         limits[i].following_error > 0
                                     ? limits[i].following_error
                                     : model.GetVelocityLimit()(i) * dt * 20;
        if (previous_command &&
            previous_command->rows() == sample.position.rows() &&
            std::abs((*previous_command)(i)-sample.position(i)) > tolerance)
            return Result::NotFollowPositionCmd;
    }
    return Result::NoError;
}
}  // namespace rocos
