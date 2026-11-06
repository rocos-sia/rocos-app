#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rocos {
// 当前 URDF 导入器未提供这些导数限制；由驱动配置按模型轴序补充。
struct JointDerivativeLimits {
    double acceleration{0};
    double jerk{0};
    double deceleration{0};
    double velocity_offset{0};
    double torque_rate{200};
    double following_error{0};  // 0: 额定速度 × 20 个控制周期
};

struct ServoJointState {
    double position{0}, velocity{0}, acceleration{0};
};
struct ServoBrake {
    struct Segment {
        uint64_t cycles{0};
        double acceleration_step{0};
    };
    std::array<Segment, 3> segments{};
    ServoJointState state;
    size_t segment{0};
    uint64_t elapsed{0};
    ServoJointState Advance(double dt);
};

struct ServoSettings {
    bool lowpass_enabled{true};
    bool rate_limit_enabled{true};
    double cutoff_hz{100};
    bool communication_guard_enabled{true};
    uint32_t max_missing_cycles{20};
    double minimum_success_rate{0.90};
    uint32_t low_quality_cycles{100};
    uint32_t first_command_timeout_ms{1000};
};
}  // namespace rocos
