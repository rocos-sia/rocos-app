#pragma once

#include <yaml-cpp/yaml.h>

#include <array>

#include "experiment_common.hpp"

namespace rocos::experiment {
// 与 Studio evaluate_fourier_trajectory 使用相同的 sin^4 包络及解析导数。
inline std::array<double, 4> evaluateFourier(const std::vector<double>& a,
                                             const std::vector<double>& b, double center, double t,
                                             double duration) {
    require(std::isfinite(t), "Nonfinite trajectory time");
    if (t <= 0 || t >= duration) return {center, 0, 0, 0};
    const double w = std::acos(-1.0) / duration;
    const double s = std::sin(w * t), c = std::cos(w * t);
    const double e = std::pow(s, 4), de = 4 * w * s * s * s * c;
    const double dde = 4 * w * w * (3 * s * s * c * c - e);
    const double ddde = 4 * w * w * w * (6 * s * c * c * c - 10 * s * s * s * c);
    double f = 0, df = 0, ddf = 0, dddf = 0;
    for (size_t k = 0; k < a.size(); ++k) {
        const double frequency = 2 * w * (k + 1);
        const double p = a[k] * std::sin(frequency * t) + b[k] * std::cos(frequency * t);
        const double v =
            frequency * (a[k] * std::cos(frequency * t) - b[k] * std::sin(frequency * t));
        f += p;
        df += v;
        ddf -= frequency * frequency * p;
        dddf -= frequency * frequency * v;
    }
    return {center + e * f, de * f + e * df, dde * f + 2 * de * df + e * ddf,
            ddde * f + 3 * dde * df + 3 * de * ddf + e * dddf};
}

struct ExcitationTrajectory {
    std::vector<std::string> names;
    std::vector<Drive> drives;
    std::vector<std::vector<double>> a, b;
    Eigen::VectorXd centers, lower, upper, velocity, acceleration;
    double duration = 0;
    std::string urdfHash;

    ExcitationTrajectory(std::string path, const HardwareConfig& hardware) {
        if (std::filesystem::path(path).extension() == ".csv")
            path = relativeTo(path, "trajectory.json");
        const auto root = YAML::LoadFile(path);
        require(root["limits_verified"].as<bool>(false),
                "Studio trajectory limits are not verified");
        names = root["joint_names"].as<std::vector<std::string>>();
        drives = orderedDrives(hardware, names);
        urdfHash = root["urdf_hash"].as<std::string>();
        duration = root["config"]["duration"].as<double>();
        require(std::isfinite(duration) && duration > 0 && duration <= 600,
                "Invalid trajectory duration");
        const auto bounds = root["config"]["bounds"], coefficients = root["coefficients"];
        const size_t n = names.size();
        require(bounds.IsSequence() && bounds.size() == n && coefficients.IsSequence() &&
                    coefficients.size() == n,
                "Trajectory joint count mismatch");
        centers.resize(n);
        lower.resize(n);
        upper.resize(n);
        velocity.resize(n);
        acceleration.resize(n);
        for (size_t j = 0; j < n; ++j) {
            const auto bound = bounds[j];
            const auto& lim = drives[j].limit;
            auto value = [&](const char* key) {
                const double x = bound[key].as<double>();
                require(std::isfinite(x), "Nonfinite trajectory bound");
                return x;
            };
            centers[j] = value("center");
            lower[j] = std::max(lim.lower, value("lower"));
            upper[j] = std::min(lim.upper, value("upper"));
            velocity[j] = std::min(lim.vel, value("velocity"));
            acceleration[j] = std::min(lim.acc, value("acceleration"));
            require(lower[j] < upper[j] && centers[j] >= lower[j] && centers[j] <= upper[j] &&
                        velocity[j] > 0 && acceleration[j] > 0,
                    "Invalid trajectory bounds");
            require(coefficients[j].IsSequence() && coefficients[j].size() == 2,
                    "Invalid Fourier coefficients");
            a.push_back(coefficients[j][0].as<std::vector<double>>());
            b.push_back(coefficients[j][1].as<std::vector<double>>());
            require(
                !a.back().empty() && a.back().size() == b.back().size() && a.back().size() <= 50,
                "Invalid Fourier harmonic count");
            for (const auto* values : {&a.back(), &b.back()})
                for (double x : *values) require(std::isfinite(x), "Nonfinite Fourier coefficient");
        }
    }

    void evaluate(double t, double scale, JntArray& target) const {
        require(target.rows() == names.size(), "Wrong trajectory target size");
        for (size_t j = 0; j < names.size(); ++j) {
            auto state = evaluateFourier(a[j], b[j], centers[j], t, duration);
            state[1] *= scale;
            state[2] *= scale * scale;
            state[3] *= scale * scale * scale;
            require(std::isfinite(state[0]) && std::isfinite(state[1]) && std::isfinite(state[2]) &&
                        std::isfinite(state[3]) && state[0] >= lower[j] && state[0] <= upper[j] &&
                        std::abs(state[1]) <= velocity[j] &&
                        std::abs(state[2]) <= acceleration[j] &&
                        std::abs(state[3]) <= drives[j].limit.jerk,
                    "Excitation exceeds limits: " + names[j]);
            target(j) = state[0];
        }
    }

    void validate(double scale, double dt) const {
        require(std::isfinite(scale) && scale > 0 && scale <= 1 && std::isfinite(dt) && dt > 0,
                "Invalid time_scale or period");
        const double step = std::min(dt * scale, duration / 10000.0);
        const double count = std::ceil(duration / step);
        require(count * names.size() <= 10000000, "Trajectory validation exceeds sample budget");
        JntArray target(names.size());
        for (size_t i = 0; i <= static_cast<size_t>(count); ++i)
            evaluate(std::min(duration, i * step), scale, target);
    }
};

// 五次过渡的速度/加速度/jerk 上界，确保当前姿态连续到达激励中心。
inline double approachDuration(const Eigen::VectorXd& start,
                               const ExcitationTrajectory& trajectory) {
    double duration = 1.0;
    for (Eigen::Index j = 0; j < start.size(); ++j) {
        const double distance = std::abs(trajectory.centers[j] - start[j]);
        duration = std::max({duration, 1.875 * distance / trajectory.velocity[j],
                             std::sqrt(5.774 * distance / trajectory.acceleration[j]),
                             std::cbrt(60 * distance / trajectory.drives[j].limit.jerk)});
    }
    return duration * 1.05;
}
}  // namespace rocos::experiment
