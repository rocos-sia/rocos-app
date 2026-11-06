#include "servo_signal_processor.hpp"

#include <algorithm>
#include <cmath>

#include "position_command_validation.hpp"

namespace rocos {
struct ServoSignalProcessor::Impl {
    Impl(size_t n, double period)
        : q(n), filtered(n), v(n), a(n), dec(n), offset(n), brakes(n), dt(period) {}
    JntArray q, filtered, v, a, dec, offset;
    std::vector<JointDerivativeLimits> limits;
    std::vector<ServoBrake> brakes;
    ModelInterface* model{nullptr};
    double dt, alpha{1};
    ServoSettings settings;
    Frame tool;
};

ServoSignalProcessor::ServoSignalProcessor() = default;
ServoSignalProcessor::~ServoSignalProcessor() = default;

Result ServoSignalProcessor::Reset(
    ModelInterface* model, const std::vector<JointDerivativeLimits>& limits,
    const JntArray& actual, double dt, const ServoSettings& settings,
    const Frame& tool) {
    if (!model) return Result::ParameterPointerEqualsNullptr;
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(settings.cutoff_hz) ||
        settings.cutoff_hz <= 0)
        return Result::IllegalParameter;
    auto rc = ValidateJointCommand(*model, actual, nullptr, dt);
    if (rc != Result::NoError) return rc;
    const auto n = actual.rows();
    if (limits.size() != n) return Result::UnmatchedJointsNumber;
    auto state = std::make_unique<Impl>(n, dt);
    state->model = model;
    state->settings = settings;
    state->tool = tool;
    state->q = state->filtered = actual;
    state->alpha =
        settings.lowpass_enabled
            ? dt / (dt + 1.0 / (2.0 * std::acos(-1.0) * settings.cutoff_hz))
            : 1;
    state->limits = limits;
    for (unsigned int i = 0; i < n; ++i) {
        const auto& l = limits[i];
        if (!std::isfinite(l.acceleration) || l.acceleration <= 0 ||
            !std::isfinite(l.jerk) || l.jerk <= 0 ||
            !std::isfinite(l.deceleration) || l.deceleration <= 0 ||
            !std::isfinite(l.velocity_offset) || l.velocity_offset < 0 ||
            model->GetVelocityLimit()(i) <= 0)
            return Result::IllegalParameter;
        state->dec(i) = l.deceleration;
        state->offset(i) = l.velocity_offset;
        rc = model->MakeServoBrake(i, {actual(i), 0, 0}, l, dt,
                                   state->brakes[i]);
        if (rc != Result::NoError) return rc;
    }
    impl_ = std::move(state);
    return Result::NoError;
}

Result ServoSignalProcessor::Step(const Reference& target, bool stopping,
                                  JntArray& output) {
    if (!impl_) return Result::ServoNotInitialized;
    auto& s = *impl_;
    JntArray goal(s.q.rows());
    auto rc = Result::NoError;
    if (!stopping) {
        if (const auto* q = std::get_if<JntArray>(&target)) {
            goal = *q;
            rc = s.model->ApplyPositionCoupling(goal);
            if (rc != Result::NoError) return rc;
            rc = ValidateJointCommand(*s.model, goal, nullptr, s.dt);
            if (rc != Result::NoError) return rc;
            for (unsigned int i = 0; i < goal.rows(); ++i)
                goal(i) = s.filtered(i) + s.alpha * (goal(i) - s.filtered(i));
        } else {
            const auto* pose = std::get_if<Frame>(&target);
            if (!pose) return Result::IllegalParameter;
            // 原始 IK 不可达/越界不能通过滤波隐藏。
            rc =
                s.model->InverseKinematics(s.q, *pose * s.tool.Inverse(), goal);
            if (rc != Result::NoError) return rc;
            rc = ValidateJointCommand(*s.model, goal, nullptr, s.dt);
            if (rc != Result::NoError) return rc;
            Frame previous;
            rc = s.model->ForwardKinematics(s.filtered, previous);
            if (rc != Result::NoError) return rc;
            previous = previous * s.tool;
            Frame filtered;
            filtered.p = previous.p + s.alpha * (pose->p - previous.p);
            const auto rotation = (previous.M.Inverse() * pose->M).GetRot();
            const double angle = rotation.Norm();
            filtered.M = angle < 1e-12
                             ? previous.M
                             : previous.M * Rotation::Rot(rotation / angle,
                                                          s.alpha * angle);
            rc = s.model->InverseKinematics(s.q, filtered * s.tool.Inverse(),
                                            goal);
            if (rc != Result::NoError) return rc;
        }
        rc = s.model->ApplyPositionCoupling(goal);
        if (rc != Result::NoError) return rc;
        rc = ValidateJointCommand(*s.model, goal, nullptr, s.dt);
        if (rc != Result::NoError) return rc;
    }
    JntArray lower, upper;
    rc = s.model->GetPositionVelocityLimits(s.q, s.dec, s.offset, lower, upper);
    if (rc != Result::NoError) return rc;
    JntArray next(s.q.rows()), velocity(s.q.rows()), acceleration(s.q.rows());
    for (unsigned int i = 0; i < next.rows(); ++i) {
        ServoJointState candidate;
        ServoBrake brake;
        bool accepted = false;
        if (!stopping) {
            if (s.settings.rate_limit_enabled) {
                rc = s.model->LimitServoPositionStep(i, goal(i),
                                                  {s.q(i), s.v(i), s.a(i)},
                                                  s.limits[i], s.dt, candidate);
            } else {
                candidate = {goal(i), (goal(i) - s.q(i)) / s.dt, 0};
                candidate.acceleration = (candidate.velocity - s.v(i)) / s.dt;
                if (std::abs(candidate.velocity) >
                    s.model->GetVelocityLimit()(i))
                    return Result::SpeedLimit;
                if (std::abs(candidate.acceleration) > s.limits[i].acceleration)
                    return Result::AccLimit;
                if (std::abs(candidate.acceleration - s.a(i)) / s.dt >
                    s.limits[i].jerk)
                    return Result::JerkLimit;
                rc = Result::NoError;
            }
            accepted = rc == Result::NoError &&
                       s.model->MakeServoBrake(i, candidate, s.limits[i], s.dt,
                                               brake) == Result::NoError;
            if (!accepted && !s.settings.rate_limit_enabled)
                return Result::MotionLimit;
        }
        if (accepted)
            s.brakes[i] = brake;
        else
            candidate = s.brakes[i].Advance(s.dt);
        next(i) = candidate.position;
        velocity(i) = candidate.velocity;
        acceleration(i) = candidate.acceleration;
    }
    // 联动不得在控制器下发时悄悄改变已验证的轨迹。
    auto coupled = next;
    rc = s.model->ApplyPositionCoupling(coupled);
    if (rc != Result::NoError) return rc;
    for (unsigned int i = 0; i < next.rows(); ++i)
        if (std::abs(coupled(i) - next(i)) > 1e-9) return Result::MotionLimit;
    rc = ValidateJointCommand(*s.model, next, &s.q, s.dt);
    if (rc != Result::NoError) return rc;
    rc =
        s.model->GetPositionVelocityLimits(next, s.dec, s.offset, lower, upper);
    if (rc != Result::NoError) return rc;
    for (unsigned int i = 0; i < next.rows(); ++i) {
        if (!std::isfinite(velocity(i)) || !std::isfinite(acceleration(i)))
            return Result::ParameterNanOrInf;
        if (velocity(i) < lower(i) - 1e-9 || velocity(i) > upper(i) + 1e-9)
            return Result::SpeedLimit;
        if (std::abs(acceleration(i)) > s.limits[i].acceleration + 1e-9)
            return Result::AccLimit;
        if (std::abs(acceleration(i) - s.a(i)) / s.dt > s.limits[i].jerk + 1e-6)
            return Result::JerkLimit;
    }
    if (!stopping) s.filtered = goal;
    s.q = next;
    s.v = velocity;
    s.a = acceleration;
    output = next;
    return Result::NoError;
}

const JntArray& ServoSignalProcessor::Position() const { return impl_->q; }
const JntArray& ServoSignalProcessor::Velocity() const { return impl_->v; }
const JntArray& ServoSignalProcessor::Acceleration() const { return impl_->a; }
bool ServoSignalProcessor::AtRest() const {
    if (!impl_) return true;
    for (unsigned int i = 0; i < impl_->q.rows(); ++i)
        if (std::abs(impl_->v(i)) > 1e-10 || std::abs(impl_->a(i)) > 1e-10)
            return false;
    return true;
}
}  // namespace rocos
