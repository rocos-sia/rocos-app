#include <algorithm>
#include <cmath>

#include "model_interface.hpp"

namespace rocos {
Result ModelInterface::GetPositionVelocityLimits(
    const JntArray& position, const JntArray& deceleration,
    const JntArray& velocity_offset, JntArray& lower_velocity,
    JntArray& upper_velocity) const {
    const auto n = static_cast<unsigned int>(GetJointNum());
    const auto& qmin = GetPosLowerLimit();
    const auto& qmax = GetPosUpperLimit();
    const auto& vmax = GetVelocityLimit();
    if (!n || position.rows() != n || deceleration.rows() != n ||
        velocity_offset.rows() != n || qmin.rows() != n || qmax.rows() != n ||
        vmax.rows() != n || &lower_velocity == &upper_velocity)
        return Result::UnmatchedJointsNumber;
    JntArray lower(n), upper(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double q = position(i), a = deceleration(i),
                     offset = velocity_offset(i);
        if (!std::isfinite(q) || std::isnan(qmin(i)) || qmin(i) == INFINITY ||
            std::isnan(qmax(i)) || qmax(i) == -INFINITY ||
            !std::isfinite(vmax(i)) || !std::isfinite(a) ||
            !std::isfinite(offset))
            return Result::ParameterNanOrInf;
        if (qmin(i) > qmax(i) || vmax(i) < 0 || a <= 0 || offset < 0)
            return Result::IllegalParameter;
        if (q < qmin(i) || q > qmax(i)) return Result::PosLimit;
        // 使用 long double 避免有限输入的中间乘积溢出。
        auto speed = [&](long double distance) {
            const auto braking = std::sqrt(2.0L * a * distance) - offset;
            return static_cast<double>(
                std::min<long double>(vmax(i), std::max(0.0L, braking)));
        };
        upper(i) = speed(static_cast<long double>(qmax(i)) - q);
        lower(i) = -speed(static_cast<long double>(q) - qmin(i));
    }
    lower_velocity = std::move(lower);
    upper_velocity = std::move(upper);
    return Result::NoError;
}
}  // namespace rocos

#include <array>
#include <limits>

namespace rocos {
namespace {
// 常 jerk 离散段：a[k]=a[0]+d*k，随后依次积分 v、q，与下发差分定义一致。
ServoJointState integrateServo(const ServoJointState& s, double d, double k,
                               double dt) {
    return {s.position +
                dt * (k * s.velocity + dt * (s.acceleration * k * (k + 1) / 2 +
                                             d * k * (k + 1) * (k + 2) / 6)),
            s.velocity + dt * (k * s.acceleration + d * k * (k + 1) / 2),
            s.acceleration + d * k};
}
// 三次多项式的驻点分区后求根；有界迭代，不依赖轨迹规划器。
template <typename Check>
bool checkRoots(double a, double b, double c, double d, double end,
                Check check) {
    std::array<double, 4> cuts{0, end, end, end};
    size_t n = 2;
    if (std::abs(a) > 1e-30) {
        const double discriminant = b * b - 3 * a * c;
        if (discriminant >= 0) {
            for (double x : {(-b - std::sqrt(discriminant)) / (3 * a),
                             (-b + std::sqrt(discriminant)) / (3 * a)})
                if (x > 0 && x < end) cuts[n++] = x;
        }
    } else if (std::abs(b) > 1e-30) {
        const double x = -c / (2 * b);
        if (x > 0 && x < end) cuts[n++] = x;
    }
    std::sort(cuts.begin(), cuts.begin() + n);
    auto f = [&](double x) { return ((a * x + b) * x + c) * x + d; };
    for (size_t i = 0; i < n; ++i) {
        if (!check(cuts[i])) return false;
        if (!i) continue;
        double lo = cuts[i - 1], hi = cuts[i], flo = f(lo), fhi = f(hi);
        if ((flo < 0) == (fhi < 0)) continue;
        for (int iteration = 0; iteration < 48; ++iteration) {
            const double mid = (lo + hi) / 2;
            if ((f(mid) < 0) == (flo < 0))
                lo = mid;
            else
                hi = mid;
        }
        if (!check((lo + hi) / 2)) return false;
    }
    return true;
}
bool safeSegment(const ServoJointState& s, double d, uint64_t cycles, double dt,
                 double lower, double upper, double vmax, double dec,
                 double offset) {
    auto check = [&](double k) {
        const auto x = integrateServo(s, d, k, dt);
        if (!std::isfinite(x.position) || !std::isfinite(x.velocity) ||
            x.position < lower - 1e-10 || x.position > upper + 1e-10)
            return false;
        const double distance =
            x.velocity >= 0 ? upper - x.position : x.position - lower;
        const double bound = std::min(
            vmax, std::max(0.0, std::sqrt(2 * dec * std::max(0.0, distance)) -
                                    offset));
        return std::abs(x.velocity) <= bound + 1e-10;
    };
    const double b0 = s.velocity, b1 = dt * (s.acceleration + d / 2),
                 b2 = dt * d / 2;
    const double c1 = dt * s.velocity + dt * dt * (s.acceleration / 2 + d / 3);
    const double c2 = dt * dt * (s.acceleration + d) / 2, c3 = dt * dt * d / 6;
    if (!checkRoots(0, 3 * c3, 2 * c2, c1, cycles, check) ||
        !checkRoots(0, 0, 2 * b2, b1, cycles, check))
        return false;
    for (double sign : {-1.0, 1.0})
        if (!checkRoots(
                2 * b2 * b2, 3 * b1 * b2 + 3 * sign * dec * c3,
                b1 * b1 + 2 * b2 * (b0 + sign * offset) + 2 * sign * dec * c2,
                b1 * (b0 + sign * offset) + sign * dec * c1, cycles, check))
            return false;
    return true;
}
}  // namespace
ServoJointState ServoBrake::Advance(double dt) {
    while (segment < segments.size() && elapsed >= segments[segment].cycles) {
        ++segment;
        elapsed = 0;
    }
    if (segment < segments.size()) {
        state =
            integrateServo(state, segments[segment].acceleration_step, 1, dt);
        ++elapsed;
        if (segment == segments.size() - 1 &&
            elapsed == segments[segment].cycles)
            state.velocity = state.acceleration = 0;
    }
    return state;
}
Result ModelInterface::LimitServoJointStep(
    unsigned int axis, double desired_velocity, const ServoJointState& s,
    const JointDerivativeLimits& l, double dt, ServoJointState& next) const {
    const auto axis_count = static_cast<unsigned int>(GetJointNum());
    if (axis >= axis_count || GetPosLowerLimit().rows() != axis_count ||
        GetPosUpperLimit().rows() != axis_count ||
        GetVelocityLimit().rows() != axis_count)
        return Result::UnmatchedJointsNumber;
    const double qmin = GetPosLowerLimit()(axis),
                 qmax = GetPosUpperLimit()(axis),
                 vmax_check = GetVelocityLimit()(axis);
    if (std::isnan(qmin) || std::isnan(qmax) || qmin == INFINITY ||
        qmax == -INFINITY || qmin > qmax || !std::isfinite(vmax_check) ||
        vmax_check <= 0)
        return Result::IllegalParameter;
    if (!std::isfinite(s.position) || !std::isfinite(s.velocity) ||
        !std::isfinite(s.acceleration) || !std::isfinite(l.acceleration) ||
        !std::isfinite(l.jerk) || !std::isfinite(l.deceleration) ||
        l.deceleration <= 0 || !std::isfinite(l.velocity_offset) ||
        l.velocity_offset < 0)
        return Result::IllegalParameter;

    if (axis >= static_cast<unsigned int>(GetJointNum()) ||
        !std::isfinite(desired_velocity) || !std::isfinite(dt) || dt <= 0 ||
        l.acceleration <= 0 || l.jerk <= 0)
        return Result::IllegalParameter;
    const double vmax = GetVelocityLimit()(axis) * (1 - 1e-9);
    const double upper = std::min(
        vmax, std::max(0.0, std::sqrt(2 * l.deceleration *
                                      std::max(0.0, GetPosUpperLimit()(axis) -
                                                        s.position)) -
                                l.velocity_offset));
    const double lower = -std::min(
        vmax,
        std::max(0.0, std::sqrt(2 * l.deceleration *
                                std::max(0.0, s.position -
                                                  GetPosLowerLimit()(axis))) -
                          l.velocity_offset));
    // Franka 风格：对速度差分，再对加速度/jerk 投影；额外与 jerk 区间求交。
    const double lo =
        std::max({-l.acceleration, s.acceleration - l.jerk * dt,
                  (l.jerk / l.acceleration) * (lower - s.velocity)});
    const double hi =
        std::min({l.acceleration, s.acceleration + l.jerk * dt,
                  (l.jerk / l.acceleration) * (upper - s.velocity)});
    if (lo > hi) return Result::MotionLimit;
    const double desired_jerk =
        ((desired_velocity - s.velocity) / dt - s.acceleration) / dt;
    const double jerk_limited_acceleration =
        s.acceleration + std::clamp(desired_jerk, -l.jerk, l.jerk) * dt;
    next.acceleration = std::clamp(jerk_limited_acceleration, lo, hi);
    next.velocity = s.velocity + next.acceleration * dt;
    next.position = s.position + next.velocity * dt;
    return Result::NoError;
}
Result ModelInterface::LimitServoPositionStep(
    unsigned int axis, double target, const ServoJointState& s,
    const JointDerivativeLimits& l, double dt, ServoJointState& next) const {
    if (!std::isfinite(target) || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(l.acceleration) || l.acceleration <= 0 ||
        !std::isfinite(l.jerk) || l.jerk <= 0)
        return Result::IllegalParameter;
    if (axis >= static_cast<unsigned int>(GetJointNum()) ||
        GetVelocityLimit().rows() != GetJointNum())
        return Result::UnmatchedJointsNumber;
    // libfranka 位置限速：可行目标本周期到达，导数由统一限制器约束。
    const double desired_velocity = (target - s.position) / dt;
    return LimitServoJointStep(axis, desired_velocity, s, l, dt, next);
}

Result ModelInterface::LimitServoReferenceStep(
    unsigned int axis, const ServoJointState& reference,
    const ServoJointState& reference_next, const ServoJointState& s,
    const JointDerivativeLimits& l, double dt, ServoJointState& next) const {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(reference.position) ||
        !std::isfinite(reference.velocity) ||
        !std::isfinite(reference.acceleration) ||
        !std::isfinite(reference_next.position) ||
        !std::isfinite(reference_next.velocity) ||
        !std::isfinite(reference_next.acceleration))
        return Result::IllegalParameter;
    if (std::abs(reference_next.position - reference.position -
                 dt * reference_next.velocity) > 1e-9 ||
        std::abs(reference_next.velocity - reference.velocity -
                 dt * reference_next.acceleration) > 1e-9)
        return Result::IllegalParameter;
    // 与 a+=h*j, v+=h*a, q+=h*v 一致的三重离散极点 exp(-80*h)。
    // 前馈提供目标运动；反馈只纠正误差，不能对目标速度本身施加阻尼。
    const double d = -std::expm1(-80 * dt);
    const double rho = 1 - d;
    const double kp = std::pow(d / dt, 3);
    const double kv = (d / dt) * (d / dt) * (1 + 2 * rho);
    const double ka = -std::expm1(-240 * dt) / dt;
    const double jerk =
        (reference_next.acceleration - reference.acceleration) / dt -
        kp * (s.position - reference.position) -
        kv * (s.velocity - reference.velocity) -
        ka * (s.acceleration - reference.acceleration);
    return LimitServoJointStep(
        axis, s.velocity + dt * (s.acceleration + dt * jerk), s, l, dt, next);
}

Result ModelInterface::MakeServoBrake(unsigned int axis,
                                      const ServoJointState& state,
                                      const JointDerivativeLimits& l, double dt,
                                      ServoBrake& brake) const {
    const auto axis_count = static_cast<unsigned int>(GetJointNum());
    if (axis >= axis_count || GetPosLowerLimit().rows() != axis_count ||
        GetPosUpperLimit().rows() != axis_count ||
        GetVelocityLimit().rows() != axis_count)
        return Result::UnmatchedJointsNumber;
    const double qmin = GetPosLowerLimit()(axis),
                 qmax = GetPosUpperLimit()(axis),
                 vmax_check = GetVelocityLimit()(axis);
    if (std::isnan(qmin) || std::isnan(qmax) || qmin == INFINITY ||
        qmax == -INFINITY || qmin > qmax || !std::isfinite(vmax_check) ||
        vmax_check <= 0)
        return Result::IllegalParameter;

    if (axis >= static_cast<unsigned int>(GetJointNum()) ||
        !std::isfinite(dt) || dt <= 0 || !std::isfinite(l.acceleration) ||
        l.acceleration <= 0 || !std::isfinite(l.jerk) || l.jerk <= 0 ||
        !std::isfinite(l.deceleration) || l.deceleration <= 0 ||
        !std::isfinite(l.velocity_offset) || l.velocity_offset < 0 ||
        !std::isfinite(state.acceleration) ||
        std::abs(state.acceleration) > l.acceleration + 1e-9)
        return Result::IllegalParameter;
    ServoBrake result;
    result.state = state;
    const double bmax = std::min(l.acceleration, l.deceleration);
    const double n = std::max(
        1.0, std::ceil((std::abs(state.acceleration) + bmax) / (l.jerk * dt)));
    const double impulse =
        state.velocity / dt + state.acceleration * (n - 1) / 2;
    const double hold = std::max(0.0, std::ceil(std::abs(impulse) / bmax - n));
    if (!std::isfinite(hold) || n > 1e9 || hold > 1e9)
        return Result::IllegalParameter;
    const double b = -impulse / (n + hold);
    result.segments = {
        {{static_cast<uint64_t>(n), (b - state.acceleration) / n},
         {static_cast<uint64_t>(hold), 0},
         {static_cast<uint64_t>(n), -b / n}}};
    auto x = state;
    for (const auto& segment : result.segments) {
        if (!safeSegment(x, segment.acceleration_step, segment.cycles, dt,
                         GetPosLowerLimit()(axis), GetPosUpperLimit()(axis),
                         GetVelocityLimit()(axis), l.deceleration,
                         l.velocity_offset))
            return Result::MotionLimit;
        x = integrateServo(x, segment.acceleration_step, segment.cycles, dt);
    }
    brake = result;
    return Result::NoError;
}
}  // namespace rocos
