#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "servo_settings.hpp"

namespace rocos {
inline ServoSettings LoadServoSettings(const YAML::Node& node) {
    ServoSettings s;
    if (!node || node.IsNull()) return s;
    if (!node.IsMap()) throw std::invalid_argument("robot.servo must be a mapping");
    const std::set<std::string> allowed{"lowpass_enabled",    "rate_limit_enabled",
                                        "cutoff_hz",          "communication_guard_enabled",
                                        "max_missing_cycles", "minimum_success_rate",
                                        "low_quality_cycles", "first_command_timeout_ms"};
    for (const auto& entry : node)
        if (!allowed.count(entry.first.as<std::string>()))
            throw std::invalid_argument("Unknown robot.servo field: " +
                                        entry.first.as<std::string>());
    auto read = [&](const char* name, auto& value) {
        if (node[name]) value = node[name].as<std::decay_t<decltype(value)>>();
    };
    read("lowpass_enabled", s.lowpass_enabled);
    read("rate_limit_enabled", s.rate_limit_enabled);
    read("communication_guard_enabled", s.communication_guard_enabled);
    read("cutoff_hz", s.cutoff_hz);
    read("minimum_success_rate", s.minimum_success_rate);
    auto integer = [&](const char* name, uint32_t& value) {
        if (!node[name]) return;
        const auto n = node[name].as<int64_t>();
        if (n < 1 || n > 1000000)
            throw std::invalid_argument(std::string("Invalid robot.servo.") + name);
        value = static_cast<uint32_t>(n);
    };
    integer("max_missing_cycles", s.max_missing_cycles);
    integer("low_quality_cycles", s.low_quality_cycles);
    integer("first_command_timeout_ms", s.first_command_timeout_ms);
    if (!std::isfinite(s.cutoff_hz) || s.cutoff_hz <= 0 || !std::isfinite(s.minimum_success_rate) ||
        s.minimum_success_rate < 0 || s.minimum_success_rate > 1)
        throw std::invalid_argument("Invalid robot.servo filter/quality settings");
    return s;
}
inline YAML::Node ServoSettingsToYaml(const ServoSettings& s) {
    YAML::Node n;
    n["lowpass_enabled"] = s.lowpass_enabled;
    n["rate_limit_enabled"] = s.rate_limit_enabled;
    n["cutoff_hz"] = s.cutoff_hz;
    n["communication_guard_enabled"] = s.communication_guard_enabled;
    n["max_missing_cycles"] = s.max_missing_cycles;
    n["minimum_success_rate"] = s.minimum_success_rate;
    n["low_quality_cycles"] = s.low_quality_cycles;
    n["first_command_timeout_ms"] = s.first_command_timeout_ms;
    return n;
}
}  // namespace rocos
