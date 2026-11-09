#include "servo_signal_processor.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "position_command_validation.hpp"
#include "log_manager.hpp"
#include "servo_trace.hpp"

namespace rocos {
struct ServoSignalProcessor::Impl {
    Impl(size_t n, double period)
        : q(n), filtered(n), v(n), a(n), dec(n), offset(n), brakes(n), pending_brakes(n),
          dt(period) {}
    JntArray q, filtered, v, a, dec, offset;
    std::vector<JointDerivativeLimits> limits;
    std::vector<ServoBrake> brakes, pending_brakes;
    std::optional<Frame> filtered_pose;
    ModelInterface* model{nullptr};
    double dt, alpha{1};
    ServoSettings settings;
    Frame tool;
};

namespace {
// 与现有反馈保护一致：位置允许显式型号容差，否则取 20 个周期的额定行程。
constexpr double kTrackingCycles = 20;
double trackingTolerance(const JointDerivativeLimits& limits, double vmax, double dt) {
    return limits.following_error > 0 ? limits.following_error : vmax * dt * kTrackingCycles;
}
}

ServoSignalProcessor::ServoSignalProcessor() : trace_(ServoTrace::Create()) {}
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
        settings.lowpass_enabled && settings.cutoff_hz < 1000
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

Result ServoSignalProcessor::CheckFeedback(const JntArray& actual,
                                           const JntArray& velocity) const {
    if (!impl_) return Result::ServoNotInitialized;
    const auto& s = *impl_;
    if (actual.rows() != s.q.rows() || velocity.rows() != s.q.rows())
        return Result::UnmatchedJointsNumber;
    if (!actual.data.allFinite() || !velocity.data.allFinite())
        return Result::ParameterNanOrInf;
    for (unsigned int i = 0; i < actual.rows(); ++i) {
        const double tolerance = trackingTolerance(s.limits[i], s.model->GetVelocityLimit()(i), s.dt);
        if (std::abs(actual(i)-s.q(i)) > tolerance) {
            emitLogEvent(LogEventType::ServoFeedbackPosition, LogModule::MoveServo,
                         static_cast<int>(Result::NotFollowPositionCmd), i, actual(i), s.q(i), tolerance);
            return Result::NotFollowPositionCmd;
        }
        const double velocity_tolerance = s.limits[i].acceleration * s.dt * kTrackingCycles;
        if (std::abs(velocity(i)-s.v(i)) > velocity_tolerance) {
            emitLogEvent(LogEventType::ServoFeedbackVelocity, LogModule::MoveServo,
                         static_cast<int>(Result::OutRangeFeedback), i, velocity(i), s.v(i), velocity_tolerance);
            return Result::OutRangeFeedback;
        }
    }
    return Result::NoError;
}

Result ServoSignalProcessor::Step(const Reference& target, bool stopping,
                                  JntArray& output) {
    if (!trace_ || !impl_) return stepImpl(target, stopping, output, nullptr);
    ServoTraceRecord record;
    record.sequence = ++trace_sequence_;
    record.time_ns = ServoTraceNow();
    record.joints = impl_->q.rows();
    record.stopping = stopping;
    record.dt = impl_->dt;
    record.alpha = impl_->alpha;
    for (size_t i = 0; i < std::min<size_t>(record.joints, kServoTraceJoints); ++i) {
        record.q[i] = impl_->q(i);
        record.filtered[i] = impl_->filtered(i);
        record.v[i] = impl_->v(i);
        record.a[i] = impl_->a(i);
    }
    try {
        const auto result = stepImpl(target, stopping, output, &record);
        record.result = static_cast<int32_t>(result);
        for (size_t i = 0; i < std::min<size_t>(record.joints, kServoTraceJoints); ++i) {
            record.next[i] = impl_->q(i);
            record.next_v[i] = impl_->v(i);
            record.next_a[i] = impl_->a(i);
        }
        record.elapsed_ns = ServoTraceNow() - record.time_ns;
        trace_->Push(record);
        return result;
    } catch (...) {
        record.elapsed_ns = ServoTraceNow() - record.time_ns;
        trace_->Push(record); // result 保持哨兵值，不能将异常误写为成功。
        throw;
    }
}

Result ServoSignalProcessor::stepImpl(const Reference& target, bool stopping,
                                     JntArray& output, ServoTraceRecord* trace) {
    if (!impl_) return Result::ServoNotInitialized;
    auto& s = *impl_;
    JntArray goal(s.q.rows());
    const bool cartesian = std::holds_alternative<Frame>(target);
    auto filtered_pose = s.filtered_pose;
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
            // 首点由 MoveServo 独立检查；正常周期仅对滤波位姿求一次 IK。
            // 保留原始位姿合法性检查，避免低通掩盖非法旋转或非有限值。
            if (!pose->p.data.allFinite() || !pose->M.data.allFinite())
                return Result::ParameterNanOrInf;
            if (!(pose->M.data.transpose() * pose->M.data)
                     .isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
                std::abs(pose->M.data.determinant() - 1.0) >= 1e-8)
                return Result::IllegalParameter;
            // 与 libfranka 一致：首帧初始化，随后使用上一已接受的命令位姿。
            // 独立保存滤波历史，避免 IK 容差经 FK 回灌到低通递推中。
            const Frame& previous = s.filtered_pose ? *s.filtered_pose : *pose;
            Frame filtered = *pose;
            if (s.alpha < 1) {
                filtered.p = previous.p + s.alpha * (pose->p - previous.p);
                filtered.M.data = Eigen::Quaterniond(previous.M.data)
                                      .slerp(s.alpha, Eigen::Quaterniond(pose->M.data))
                                      .normalized().toRotationMatrix();
            }
            filtered_pose = filtered;
            {
                ServoIkTraceScope scope(trace ? &trace->ik[1] : nullptr);
                rc = s.model->InverseKinematics(s.filtered, filtered * s.tool.Inverse(), goal);
                if (trace) trace->ik[1].result = static_cast<int32_t>(rc);
            }
            if (rc != Result::NoError) return rc;
        }
        rc = s.model->ApplyPositionCoupling(goal);
        if (rc != Result::NoError) return rc;
        rc = ValidateJointCommand(*s.model, goal, nullptr, s.dt);
        if (rc != Result::NoError) return rc;
    }
    if (trace && !stopping)
        for (size_t i = 0; i < std::min<size_t>(goal.rows(), kServoTraceJoints); ++i)
            trace->goal[i] = goal(i);
    if (!stopping && cartesian) {
        for (unsigned int i = 0; i < goal.rows(); ++i) {
            const double continuity = s.model->GetVelocityLimit()(i) * s.dt;
            if (std::abs(goal(i)-s.filtered(i)) > continuity + 1e-12) {
                emitLogEvent(LogEventType::ServoIkDiscontinuity, LogModule::MoveServo,
                             static_cast<int>(Result::IkTrack), i, goal(i), s.filtered(i), continuity);
                return Result::IkTrack;
            }
            const double deviation = trackingTolerance(s.limits[i], s.model->GetVelocityLimit()(i), s.dt);
            if (std::abs(goal(i)-s.q(i)) > deviation + 1e-12) {
                emitLogEvent(LogEventType::ServoReferenceDeviation, LogModule::MoveServo,
                             static_cast<int>(Result::MotionLimit), i, goal(i), s.q(i), deviation);
                return Result::MotionLimit;
            }
        }
    }
    JntArray lower, upper;
    rc = s.model->GetPositionVelocityLimits(s.q, s.dec, s.offset, lower, upper);
    if (rc != Result::NoError) return rc;
    JntArray next(s.q.rows()), velocity(s.q.rows()), acceleration(s.q.rows());
    // 候选全部通过后才提交，失败时仍可使用上一周期的完整制动轨迹。
    s.pending_brakes = s.brakes;
    for (unsigned int i = 0; i < next.rows(); ++i) {
        ServoJointState candidate;
        ServoBrake brake;
        bool accepted = false;
        if (!stopping) {
            if (s.settings.rate_limit_enabled) {
                if (cartesian) {
                    // libfranka 位置限速：从上一已接受参考出发计算速度，
                    // 由 Model 约束加速度/jerk 后积分，避免对 IK 历史做前馈。
                    const double desired_velocity = (goal(i) - s.q(i)) / s.dt;
                    rc = s.model->LimitServoJointStep(i, desired_velocity,
                                                     {s.q(i), s.v(i), s.a(i)},
                                                     s.limits[i], s.dt, candidate);
                } else {
                    rc = s.model->LimitServoPositionStep(i, goal(i),
                                                         {s.q(i), s.v(i), s.a(i)},
                                                         s.limits[i], s.dt, candidate);
                }
            } else {
                candidate = {goal(i), (goal(i) - s.q(i)) / s.dt, 0};
                candidate.acceleration = (candidate.velocity - s.v(i)) / s.dt;
                if (std::abs(candidate.velocity) >
                    s.model->GetVelocityLimit()(i))
                    rc = Result::SpeedLimit;
                else if (std::abs(candidate.acceleration) > s.limits[i].acceleration)
                    rc = Result::AccLimit;
                else if (std::abs(candidate.acceleration - s.a(i)) / s.dt >
                    s.limits[i].jerk)
                    rc = Result::JerkLimit;
                else rc = Result::NoError;
            }
            if (!cartesian && !s.settings.rate_limit_enabled && rc != Result::NoError)
                return rc;
            if (rc == Result::NoError)
                rc = s.model->MakeServoBrake(i, candidate, s.limits[i], s.dt, brake);
            accepted = rc == Result::NoError;
            if (!accepted && cartesian) {
                emitLogEvent(LogEventType::ServoReferenceInfeasible, LogModule::MoveServo,
                             static_cast<int>(rc == Result::NoError ? Result::MotionLimit : rc),
                             i, s.v(i), s.a(i), goal(i));
                return Result::MotionLimit;
            }
            if (!accepted && !s.settings.rate_limit_enabled)
                return Result::MotionLimit;
        }
        if (accepted)
            s.pending_brakes[i] = brake;
        else {
            if (trace && i < 32) trace->brake_mask |= uint32_t{1} << i;
            candidate = s.pending_brakes[i].Advance(s.dt);
        }
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
    if (!stopping) {
        s.filtered = goal;
        s.filtered_pose = cartesian ? filtered_pose : std::nullopt;
    }
    s.brakes.swap(s.pending_brakes);
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
