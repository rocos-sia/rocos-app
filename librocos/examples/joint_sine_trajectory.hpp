#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace joint_sine {
constexpr double kPi = 3.14159265358979323846;

struct Parameters {
    double amplitude{kPi / 3};
    double frequency{0.2};
    double duration{180};
    double ramp{5};
    double hold{1};

    void validate() const {
        for (double value : {amplitude, frequency, duration, ramp, hold})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("Trajectory parameters must be finite and positive");
        if (duration < 2 * ramp)
            throw std::invalid_argument("Duration must include both amplitude ramps");
    }
};

struct Point {
    double q{}, v{}, a{}, j{};
};

// 七次幅值包络，r 及其前三阶导数在连接处连续。
inline Point evaluate(const Parameters& p, double t) {
    if (t <= 0 || t >= p.duration) return {};
    double r = 1, dr = 0, ddr = 0, dddr = 0;
    if (t < p.ramp || t > p.duration - p.ramp) {
        const bool ending = t > p.duration - p.ramp;
        const double u = (ending ? p.duration - t : t) / p.ramp;
        const double sign = ending ? -1 : 1;
        r = std::pow(u, 4) * (35 + u * (-84 + u * (70 - 20 * u)));
        dr = sign * 140 * std::pow(u * (1 - u), 3) / p.ramp;
        ddr = 420 * u * u * (1 - u) * (1 - u) * (1 - 2 * u) /
              (p.ramp * p.ramp);
        dddr = sign * (840 * u - 5040 * u * u + 8400 * u * u * u -
                       4200 * std::pow(u, 4)) / std::pow(p.ramp, 3);
    }
    const double w = 2 * kPi * p.frequency;
    const double s = std::sin(w * t), c = std::cos(w * t);
    return {p.amplitude * r * s,
            p.amplitude * (dr * s + r * w * c),
            p.amplitude * ((ddr - r * w * w) * s + 2 * dr * w * c),
            p.amplitude * ((dddr - 3 * dr * w * w) * s +
                           (3 * ddr * w - r * w * w * w) * c)};
}

// 包含启停包络的解析保守上界，避免仅检查稳态正弦的导数。
inline Point bounds(const Parameters& p) {
    p.validate();
    const double w = 2 * kPi * p.frequency;
    const double r1 = 2.1875 / p.ramp;
    const double r2 = 8 / (p.ramp * p.ramp);
    const double r3 = 53 / std::pow(p.ramp, 3);
    return {p.amplitude, p.amplitude * (w + r1),
            p.amplitude * (w * w + 2 * w * r1 + r2),
            p.amplitude * (w * w * w + 3 * w * w * r1 + 3 * w * r2 + r3)};
}

struct Limits {
    double lower{}, upper{}, velocity{}, acceleration{}, jerk{}, following{};
};

inline void checkTrajectory(const Parameters& p, const Limits& limit) {
    const auto peak = bounds(p);
    if (limit.lower >= -peak.q || limit.upper <= peak.q)
        throw std::invalid_argument("Sine range reaches a joint position limit");
    if (peak.v > limit.velocity || peak.a > limit.acceleration || peak.j > limit.jerk)
        throw std::invalid_argument("Sine including ramps exceeds derivative limits");
}
}  // namespace joint_sine
