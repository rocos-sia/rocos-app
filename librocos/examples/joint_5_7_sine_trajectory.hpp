#pragma once

#include <librocos/robot.h>

#include <limits>

#include "joint_sine_trajectory.hpp"

namespace joint_sine {

class Joint5To7Sine {
   public:
    static constexpr std::size_t kDof = 7;
    static constexpr std::size_t kFirstMovingJoint = 4;
    static constexpr double kAmplitude = kPi / 6;
    static constexpr double kMaxVelocity = 0.5;
    // 给 App 的伺服滤波和跟踪留出速度余量。
    static constexpr double kNominalVelocity = 0.45;
    static constexpr double kAngularVelocity = kNominalVelocity / kAmplitude;
    static constexpr double kRamp = 2.0;
    static constexpr int kCycles = 3;
    static constexpr double kCycleDuration = 2 * kPi / kAngularVelocity;
    static constexpr double kMinimumDuration = kRamp + kCycleDuration;
    static constexpr double kDuration = kRamp + kCycles * kCycleDuration;

    explicit Joint5To7Sine(const librocos::JointArray& initial, double duration = kDuration)
        : initial_(initial), duration_(duration) {
        constexpr double max_duration = std::numeric_limits<std::uint64_t>::max() / 1000.0;
        if (!std::isfinite(duration_) || duration_ < kMinimumDuration || duration_ >= max_duration)
            throw std::invalid_argument("Duration must fit the control cycle counter and include at least one full sine cycle (about 9.31 s)");
        const double cycles = duration_ == kDuration
                                  ? kCycles
                                  : std::max(1.0, std::floor((duration_ - kRamp) / kCycleDuration));
        total_phase_ = 2 * kPi * cycles;
        angular_velocity_ = total_phase_ / (duration_ - kRamp);

        for (std::size_t joint = kFirstMovingJoint; joint < kDof; ++joint) {
            if (!std::isfinite(initial_[joint]) || std::abs(initial_[joint]) > kAmplitude)
                throw std::invalid_argument("J" + std::to_string(joint + 1) +
                                            " must start within [-30, +30] degrees");
            phase_offset_[joint - kFirstMovingJoint] = std::asin(initial_[joint] / kAmplitude);
        }
    }

    [[nodiscard]] double duration() const { return duration_; }

    [[nodiscard]] librocos::JointPositions command(double elapsed) const {
        librocos::JointPositions target(initial_, kDof);
        if (elapsed <= 0 || elapsed >= duration_) return target;

        double phase = angular_velocity_ * (elapsed - kRamp / 2);
        if (elapsed < kRamp)
            phase = rampPhase(elapsed);
        else if (elapsed > duration_ - kRamp)
            phase = total_phase_ - rampPhase(duration_ - elapsed);

        for (std::size_t joint = kFirstMovingJoint; joint < kDof; ++joint)
            target.q[joint] =
                kAmplitude * std::sin(phase + phase_offset_[joint - kFirstMovingJoint]);
        return target;
    }

   private:
    double rampPhase(double time) const {
        const double u = time / kRamp;
        // 积分五次速度过渡，保证相位速度不超过 kAngularVelocity。
        const double u2 = u * u;
        return angular_velocity_ * kRamp * u2 * u2 * (2.5 - 3 * u + u2);
    }

    librocos::JointArray initial_;
    double duration_;
    double total_phase_{};
    double angular_velocity_{};
    std::array<double, kDof - kFirstMovingJoint> phase_offset_{};
};

}  // namespace joint_sine
