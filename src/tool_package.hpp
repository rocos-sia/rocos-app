#pragma once
#include "types.hpp"
#include <map>
#include <cstdint>
#include <string>
#include <yaml-cpp/yaml.h>

namespace rocos {
struct ToolPackage {
    std::string name, version, digest, urdf, directory;
    rocos::Frame default_mount, default_tcp, mount, tcp;
    std::string calibration_yaml;
    uint64_t revision{1};
};
// 包资源按内容摘要保存为不可变版本；本机覆盖不修改原包。
class ToolPackageFiles {
public:
    static ToolPackage read(const std::string& directory);
    static ToolPackage installZip(const std::string& bytes, const std::string& library);
    static ToolPackage freeze(const std::string& directory, const std::string& library);
    static std::string exportZip(const ToolPackage& tool, const std::string& version, bool calibrated);
    static std::string asset(const ToolPackage& tool, const std::string& path);
    static rocos::Frame pose(const YAML::Node& node);
    static YAML::Node pose(const rocos::Frame& frame);
};
} // namespace rocos
