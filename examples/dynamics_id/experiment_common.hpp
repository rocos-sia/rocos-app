#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>

#include "src/hardware.hpp"

namespace rocos::experiment {
using Clock = std::chrono::steady_clock;
using File = std::unique_ptr<FILE, decltype(&std::fclose)>;

inline void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

inline std::string relativeTo(const std::string& owner, const std::string& path) {
    return (std::filesystem::path(owner).parent_path() / path).lexically_normal().string();
}

inline File createOutput(const std::string& path) {
    File result(std::fopen(path.c_str(), "wx"), std::fclose);
    require(bool(result), "Cannot create new output: " + path);
    return result;
}

inline void finishOutput(FILE* file) {
    require(std::fflush(file) == 0 && !std::ferror(file), "Output write failed");
}

// 整机实验要求名称与驱动器一一对应；不允许隐式使能额外轴。
inline std::vector<Drive> orderedDrives(const HardwareConfig& hardware,
                                        const std::vector<std::string>& names) {
    require(!names.empty() && names.size() == hardware.drives.size(),
            "Whole-robot experiment requires every configured drive exactly once");
    std::set<std::string> seenNames;
    std::set<int> seenIds;
    for (const auto& drive : hardware.drives) {
        require(drive.id >= 0 && seenIds.insert(drive.id).second, "Invalid/duplicate drive ID");
        require(!drive.joint_name.empty() && seenNames.insert(drive.joint_name).second,
                "Invalid/duplicate hardware joint name");
    }
    seenNames.clear();
    std::vector<Drive> result;
    for (const auto& name : names) {
        require(seenNames.insert(name).second, "Duplicate model joint: " + name);
        const auto it = std::find_if(hardware.drives.begin(), hardware.drives.end(),
                                     [&](const Drive& d) { return d.joint_name == name; });
        require(it != hardware.drives.end(), "Joint missing from hardware: " + name);
        const auto& d = *it;
        const auto& t = d.transform;
        const auto& l = d.limit;
        require(t.user_unit_name == "rad", "Expected rad position units: " + name);
        require(std::isfinite(t.cnt_per_unit) && t.cnt_per_unit != 0 && std::isfinite(t.ratio) &&
                    t.ratio != 0 && std::isfinite(t.torque_per_unit) && t.torque_per_unit != 0,
                "Invalid hardware conversion: " + name);
        require(std::isfinite(l.lower) && std::isfinite(l.upper) && l.lower < l.upper &&
                    std::isfinite(l.vel) && l.vel > 0 && std::isfinite(l.acc) && l.acc > 0 &&
                    std::isfinite(l.jerk) && l.jerk > 0 && std::isfinite(l.effort) && l.effort > 0,
                "Invalid hardware limits: " + name);
        for (double q : {l.lower, l.upper}) {
            const double raw = q * t.cnt_per_unit * t.ratio + t.offset_pos_cnt;
            require(std::isfinite(raw) && raw >= std::numeric_limits<int32_t>::min() &&
                        raw <= std::numeric_limits<int32_t>::max(),
                    "Position range exceeds int32 PDO: " + name);
        }
        result.push_back(d);
    }
    return result;
}

inline void bindAndValidate(Hardware& hardware, const std::vector<Drive>& drives, int busId,
                            bool torqueMode) {
    std::vector<int32_t> ids;
    auto& bus = *EcatConfig::getInstance(busId);
    for (const auto& d : drives) {
        ids.push_back(d.id);
        // Hardware 对缺失 PDO 会返回零或忽略写入；实验启动时必须显式拒绝。
        require(bool(PdoEntry<uint16_t>::input(bus, d.id, d.inputs.status_word)),
                "Missing status PDO");
        require(bool(PdoEntry<int32_t>::input(bus, d.id, d.inputs.position_actual_value)),
                "Missing position PDO");
        require(bool(PdoEntry<int32_t>::input(bus, d.id, d.inputs.velocity_actual_value)),
                "Missing velocity PDO");
        require(bool(PdoEntry<int16_t>::input(bus, d.id, d.inputs.torque_actual_value)),
                "Missing torque PDO");
        require(bool(PdoEntry<uint16_t>::output(bus, d.id, d.outputs.control_word)),
                "Missing control PDO");
        require(bool(PdoEntry<int8_t>::output(bus, d.id, d.outputs.mode_of_operation)),
                "Missing mode PDO");
        if (torqueMode) {
            require(bool(PdoEntry<int16_t>::output(bus, d.id, d.outputs.target_torque)),
                    "Missing target torque PDO");
        } else {
            require(bool(PdoEntry<int32_t>::output(bus, d.id, d.outputs.target_position)),
                    "Missing target position PDO");
        }
    }
    require(hardware.SetJointBinding(ids) == Result::NoError, "Cannot bind model joints");
}

// 在 Hardware 销毁前显式下使能，包括异常和用户中断路径。
class DisableGuard {
   public:
    explicit DisableGuard(Hardware& hardware) : hardware_(hardware) {}
    ~DisableGuard() {
        if (!finished_) {
            try {
                stop();
            } catch (const std::exception& e) {
                std::cerr << "Disable failed: " << e.what() << '\n';
            }
        }
    }
    void stop() {
        hardware_.SetDisabled();
        require(hardware_.GetState() == JntState::DISABLED, "Drive disable was not confirmed");
        finished_ = true;
    }
    // 启动阶段下使能后，仍需在后续退出时再次执行下使能。
    void prepare() {
        stop();
        finished_ = false;
    }

   private:
    Hardware& hardware_;
    bool finished_ = false;
};

struct Feedback {
    Eigen::VectorXd q, dq, tau;
    explicit Feedback(size_t n) : q(n), dq(n), tau(n) {}
};

inline void readFeedback(Hardware& hardware, const std::vector<Drive>& drives, Feedback& state) {
    for (size_t j = 0; j < drives.size(); ++j) {
        const auto& drive = drives[j];
        state.q[j] = hardware.GetJointPosition(drive.id);
        state.dq[j] = hardware.GetJointVelocity(drive.id);
        state.tau[j] = hardware.GetJointTorque(drive.id);
        require(
            std::isfinite(state.q[j]) && std::isfinite(state.dq[j]) && std::isfinite(state.tau[j]),
            "Nonfinite feedback: " + drive.joint_name);
        require(state.q[j] >= drive.limit.lower && state.q[j] <= drive.limit.upper,
                "Position limit exceeded: " + drive.joint_name);
        require(std::abs(state.dq[j]) <= drive.limit.vel,
                "Velocity limit exceeded: " + drive.joint_name);
    }
}

class CycleClock {
   public:
    CycleClock(Hardware& hardware, double maxGap) : hardware_(hardware), maxGap_(maxGap) {
        const double dt = hardware.GetDt() * 1e-6;
        require(dt > 0 && std::isfinite(maxGap) && maxGap >= dt, "Invalid cycle period/max gap");
    }
    void reset() { last_ = Clock::now(); }
    double wait() {
        hardware_.WaitForSignal();
        const auto now = Clock::now();
        const double dt = std::chrono::duration<double>(now - last_).count();
        last_ = now;
        require(dt > 0 && dt <= maxGap_, "Control cycle gap exceeded");
        return dt;
    }

   private:
    Hardware& hardware_;
    double maxGap_;
    Clock::time_point last_ = Clock::now();
};

inline void checkEnabled(Hardware& hardware) {
    require(hardware.GetState() == JntState::ENABLED, "Not all drives are enabled");
}

inline void limitTorque(const Eigen::VectorXd& requested, const Eigen::VectorXd& magnitude,
                        const Eigen::VectorXd& rate, double dt, Eigen::VectorXd& sent) {
    require(requested.allFinite() && dt > 0 && std::isfinite(dt), "Invalid torque/time step");
    for (Eigen::Index j = 0; j < requested.size(); ++j) {
        const double bounded = std::clamp(requested[j], -magnitude[j], magnitude[j]);
        sent[j] = std::clamp(bounded, sent[j] - rate[j] * dt, sent[j] + rate[j] * dt);
    }
}
}  // namespace rocos::experiment
