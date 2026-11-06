#pragma once

#include <cmath>

#include "model_interface.hpp"

namespace rocos {
struct JointCommandViolation {
    int joint{-1};
    double value{0}, lower{0}, upper{0};
};

// 预览和实机共用的位置/指令步进检查；不访问硬件，不改变控制器状态。
inline Result ValidateJointCommand(const ModelInterface& model,
                                   const JntArray& q, const JntArray* previous,
                                   double dt,
                                   JointCommandViolation* violation = nullptr) {
    const auto n = static_cast<unsigned int>(model.GetJointNum());
    const auto& lower = model.GetPosLowerLimit();
    const auto& upper = model.GetPosUpperLimit();
    const auto& velocity = model.GetVelocityLimit();
    if (!n || q.rows() != n || lower.rows() != n || upper.rows() != n ||
        velocity.rows() != n || (previous && previous->rows() != n))
        return Result::UnmatchedJointsNumber;
    if (!std::isfinite(dt) || dt <= 0) return Result::IllegalParameter;
    for (unsigned int i = 0; i < n; ++i) {
        if (!std::isfinite(q(i)) || std::isnan(lower(i)) || lower(i) == INFINITY ||
            std::isnan(upper(i)) || upper(i) == -INFINITY || !std::isfinite(velocity(i)) ||
            (previous && !std::isfinite((*previous)(i))))
            return Result::ParameterNanOrInf;
        if (q(i) < lower(i) || q(i) > upper(i)) {
            if (violation)
                *violation = {static_cast<int>(i), q(i), lower(i), upper(i)};
            return Result::PosLimit;
        }
        if (lower(i) > upper(i) || velocity(i) < 0) return Result::IllegalParameter;
        if (previous && std::abs(q(i) - (*previous)(i)) / dt > velocity(i)) {
            if (violation)
                *violation = {static_cast<int>(i),
                              std::abs(q(i) - (*previous)(i)) / dt, 0,
                              velocity(i)};
            return Result::SpeedLimit;
        }
    }
    return Result::NoError;
}
}  // namespace rocos
