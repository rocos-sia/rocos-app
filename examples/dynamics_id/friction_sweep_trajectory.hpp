#pragma once

#include <yaml-cpp/yaml.h>

#include "experiment_common.hpp"

namespace rocos::experiment {
inline constexpr double kFrictionApproachSpeed = 0.2;
inline constexpr double kQuinticRampAcceleration = 1.875;
inline const double kQuinticRampJerk = 10 * std::sqrt(3.0) / 3;

inline double quinticVelocityIntegral(double u) {
  const double u2 = u * u;
  return u2 * u2 * (2.5 + u * (-3 + u));
}

// 所有轴共用一个 0 到 1 的位移进度，五次速度过渡在两端的加速度为零。
class FrictionApproach {
 public:
  FrictionApproach(const Eigen::VectorXd& displacement,
                   const std::vector<Drive>& drives, double dt) {
    double acceleration = std::numeric_limits<double>::infinity();
    double jerk = std::numeric_limits<double>::infinity();
    double speed = 1 / dt;
    if (displacement.cwiseAbs().maxCoeff() == 0) return;
    for (Eigen::Index j = 0; j < displacement.size(); ++j) {
      const double distance = std::abs(displacement[j]);
      if (distance == 0) continue;
      const auto& limit = drives[j].limit;
      speed = std::min(speed, std::min(kFrictionApproachSpeed, limit.vel) / distance);
      acceleration = std::min(acceleration, limit.acc / distance);
      jerk = std::min(jerk, limit.jerk / distance);
    }
    // 短行程降峰值速度，使两段过渡的位移之和不超过总位移。
    speed_ = std::min({speed, std::sqrt(acceleration / kQuinticRampAcceleration),
                      std::cbrt(jerk / kQuinticRampJerk)});
    const double half = 1 / speed_;
    ramp_ = std::min(half, std::max({dt, kQuinticRampAcceleration * speed_ / acceleration,
                                   std::sqrt(kQuinticRampJerk * speed_ / jerk)}));
    duration_ = half + ramp_;
    require(std::isfinite(duration_), "Cannot plan friction approach with hardware limits");
  }

  double duration() const { return duration_; }

  double progress(double time) const {
    if (time >= duration_) return 1;
    if (time <= 0) return 0;
    if (time < ramp_) return rampDistance(time);
    const double remaining = duration_ - time;
    if (remaining < ramp_) return 1 - rampDistance(remaining);
    return speed_ * (time - ramp_ / 2);
  }

 private:
  double rampDistance(double time) const {
    return speed_ * ramp_ * quinticVelocityIntegral(time / ramp_);
  }

  double speed_ = 0, ramp_ = 0, duration_ = 0;
};

enum class SweepPhase { Ramp, Steady, Dwell };

struct SweepReference {
  double time = 0;
  size_t repeat = 0, pair = 0, sweep = 0;
  SweepPhase phase = SweepPhase::Ramp;
  double velocity = 0;
};

// Studio friction_protocol.json describes analytic sweeps independent of the
// CSV sample period.
struct FrictionSweepTrajectory {
  std::vector<std::string> names;
  std::vector<Drive> drives;
  Eigen::VectorXd centers;
  std::string urdfHash;
  size_t joint = 0;
  int repeats = 0;
  double duration = 0, sampleRate = 0;

  FrictionSweepTrajectory(const std::string& path,
                          const HardwareConfig& hardware) {
    const auto root = YAML::LoadFile(path);
    const auto config = root["config"];
    names = root["joint_names"].as<std::vector<std::string>>();
    drives = orderedDrives(hardware, names);
    urdfHash = root["urdf_hash"].as<std::string>();
    joint = root["joint_index"].as<size_t>();
    require(joint < names.size() &&
                root["joint"].as<std::string>() == names[joint] &&
                config["joint"].as<std::string>() == names[joint],
            "Friction joint name/index does not match hardware");
    lower_ = config["lower"].as<double>();
    upper_ = config["upper"].as<double>();
    const auto& limit = drives[joint].limit;
    require(lower_ >= limit.lower && lower_ < upper_ && upper_ <= limit.upper,
            "Friction sweep exceeds hardware position limits: " + names[joint]);
    const auto initial = root["initial_pose"].as<std::vector<double>>();
    const auto pose = config["pose"].as<std::vector<double>>();
    require(initial.size() == names.size() && (pose.empty() || pose.size() == names.size()),
            "Wrong friction pose dimension");
    centers.resize(names.size());
    for (size_t j = 0; j < names.size(); ++j) {
      const auto& l = drives[j].limit;
      // With an empty pose Studio exports the URDF midpoints in initial_pose.
      require(initial[j] >= l.lower && initial[j] <= l.upper &&
                  (j == joint ? initial[j] == lower_ : pose.empty() || initial[j] == pose[j]),
              "Invalid friction start/hold position: " + names[j]);
      centers[j] = initial[j];
    }
    const double acc = config["acceleration"].as<double>();
    const double dwell = config["dwell_seconds"].as<double>();
    const double minSteady = config["minimum_steady_seconds"].as<double>();
    sampleRate = config["sample_rate"].as<double>();
    repeats = config["repeats"].as<int>();
    const auto speeds = config["speeds"].as<std::vector<double>>();
    require(acc > 0 && acc <= limit.acc,
            "Friction sweep exceeds hardware acceleration limit");
    require(dwell >= 0 && dwell <= 60 && sampleRate >= 10 &&
                sampleRate <= 2000 && std::isfinite(minSteady) &&
                minSteady > 0 && repeats >= 1 && repeats <= 20 &&
                speeds.size() >= 2 && speeds.size() <= 30,
            "Invalid friction protocol timing");
    std::set<double> seen;
    for (double speed : speeds) {
      require(speed > 0 && speed <= limit.vel,
              "Friction sweep exceeds hardware speed limit");
      require(seen.insert(speed).second, "Duplicate friction sweep speed");
      const double ramp = kQuinticRampAcceleration * speed / acc;
      const double steady = (upper_ - lower_) / speed - ramp;
      const double jerk = speed / (ramp * ramp) * kQuinticRampJerk;
      require(std::isfinite(steady) && steady >= minSteady + 0.3,
              "Friction sweep has insufficient steady duration");
      require(jerk <= limit.jerk,
              "Friction sweep exceeds hardware jerk limit: " + names[joint]);
    }
    for (int repeat = 0; repeat < repeats; ++repeat) {
      for (double speed : speeds) {
        const double ramp = kQuinticRampAcceleration * speed / acc;
        const double steady = (upper_ - lower_) / speed - ramp;
        for (double direction : {1., -1.}) {
          const double motionEnd = duration + (2 * ramp + steady);
          const double end = motionEnd + std::max(dwell, 1 / sampleRate);
          sweeps_.push_back(
              {duration, motionEnd, end, ramp, steady, direction * speed});
          duration = end;
        }
      }
    }
    sweepsPerRepeat_ = 2 * speeds.size();
    const double expected = root["trajectory_duration_seconds"].as<double>();
    require(std::isfinite(duration) && std::isfinite(expected) &&
                std::abs(duration - expected) <= 1e-10 * duration &&
                root["pairs"].as<size_t>() == sweeps_.size() / 2,
            "Friction schedule does not match protocol duration or pair count");
    require(duration * sampleRate * names.size() <= 1010000,
            "Friction recording exceeds the Studio trajectory sample budget");
  }

  SweepReference evaluate(double time, JntArray& target) const {
    target.data = centers;
    const auto it =
        std::upper_bound(sweeps_.begin(), sweeps_.end(), time,
                         [](double t, const Sweep& s) { return t < s.end; });
    const auto& sweep = it == sweeps_.end() ? sweeps_.back() : *it;
    const size_t index = &sweep - sweeps_.data();
    SweepReference reference{time,  index / sweepsPerRepeat_, index / 2,
                             index, SweepPhase::Ramp,         sweep.velocity};
    const bool forward = sweep.velocity > 0;
    const double origin = forward ? lower_ : upper_;
    const double t = time - sweep.start;
    if (time >= sweep.motionEnd) {
      target(joint) = forward ? upper_ : lower_;
      reference.phase = SweepPhase::Dwell;
      reference.velocity = 0;
      return reference;
    }
    double position;
    if (t < sweep.ramp) {
      position = sweep.ramp * quinticVelocityIntegral(t / sweep.ramp);
    } else if (t < sweep.ramp + sweep.steady) {
      position = t - sweep.ramp / 2;
      reference.phase = SweepPhase::Steady;
    } else {
      const double u =
          std::min((t - sweep.ramp - sweep.steady) / sweep.ramp, 1.0);
      position = sweep.ramp / 2 + sweep.steady +
                 sweep.ramp * (u - quinticVelocityIntegral(u));
    }
    target(joint) =
        std::clamp(origin + sweep.velocity * position, lower_, upper_);
    return reference;
  }

 private:
  struct Sweep {
    double start, motionEnd, end, ramp, steady, velocity;
  };
  std::vector<Sweep> sweeps_;
  size_t sweepsPerRepeat_ = 0;
  double lower_ = 0, upper_ = 0;
};
}  // namespace rocos::experiment
