#pragma once
#include "hardware_interface.hpp"
#include "position_command_validation.hpp"

namespace rocos {
// 只校验实际反馈的范围和有效性；不以实际位置与指令的偏差触发停止。
inline Result ValidateFeedback(const ModelInterface& model, HardwareInterface& hw,
                               const FeedbackSnapshot& sample) {
    const double dt = hw.GetDt() * 1e-6;
    auto rc = ValidateJointCommand(model, sample.position, nullptr, dt);
    if (rc != Result::NoError) return rc;
    if (sample.velocity.rows() != sample.position.rows())
        return Result::UnmatchedJointsNumber;
    rc = hw.CheckFeedbackFreshness(uint64_t(hw.GetDt()) * 20);
    if (rc != Result::NoError) return rc;
    for (unsigned int i = 0; i < sample.position.rows(); ++i) {
        if (!std::isfinite(sample.velocity(i)))
            return Result::ParameterNanOrInf;
        if (std::abs(sample.velocity(i)) > model.GetVelocityLimit()(i))
            return Result::SpeedLimit;
    }
    return Result::NoError;
}
}  // namespace rocos
