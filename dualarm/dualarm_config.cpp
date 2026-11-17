#include "dualarm_config.hpp"

#include <filesystem>
#include <set>
#include <stdexcept>

#include "model_bundle.hpp"

namespace rocos::dualarm {
namespace {
namespace fs = std::filesystem;
void Keys(const YAML::Node& node, std::initializer_list<std::string> allowed,
          const std::string& name) {
  if (!node.IsMap()) throw std::invalid_argument(name + " must be a mapping");
  const std::set<std::string> keys(allowed);
  for (const auto& field : node) {
    if (!keys.count(field.first.as<std::string>()))
      throw std::invalid_argument("Unknown " + name +
                                  " field: " + field.first.as<std::string>());
  }
}
std::string Resolve(const fs::path& base, const std::string& value) {
  if (value.empty()) throw std::invalid_argument("Empty configuration path");
  return fs::weakly_canonical(base / value).string();
}
void DifferentPath(const std::string& left, const std::string& right,
                   const std::string& label) {
  if (!left.empty() && !right.empty() &&
      fs::weakly_canonical(left) == fs::weakly_canonical(right))
    throw std::invalid_argument("Arms must use different " + label);
}
}  // namespace

DualArmConfig DualArmConfig::Load(const std::string& filename) {
  const auto file = fs::absolute(filename);
  const auto base = file.parent_path();
  const auto root = YAML::LoadFile(file.string());
  Keys(root,
       {"schema_version", "arms", "http_host", "http_port",
        "http_control_ttl_seconds", "logging", "diagnostics", "workcell"},
       "dualarm");
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1)
    throw std::invalid_argument("dualarm schema_version must be 1");
  Keys(root["arms"], {"left", "right"}, "arms");
  DualArmConfig config;
  for (int i = 0; i < 2; ++i) {
    const std::string side = i == 0 ? "left" : "right";
    const auto node = root["arms"][side];
    Keys(node, {"config"}, side);
    config.arms[i] =
        AppConfig::load(Resolve(base, node["config"].as<std::string>()));
    if (!config.arms[i].robot.composed_configuration)
      throw std::invalid_argument(
          "Dual-arm children require schema_version: 1 App configs");
  }
  const auto& left = config.arms[0].robot;
  const auto& right = config.arms[1].robot;
  if (const auto model = root["workcell"]) {
    Keys(model, {"urdf", "asset_root"}, "workcell");
    config.model_urdf = Resolve(base, model["urdf"].as<std::string>());
    config.model_asset_root =
        Resolve(base, model["asset_root"].as<std::string>());
    // Initial workcell support uses the same complete model as both chains.
    if (fs::weakly_canonical(left.urdf) != config.model_urdf ||
        fs::weakly_canonical(right.urdf) != config.model_urdf)
      throw std::invalid_argument("workcell URDF must match both arm models");
  }
  if (left.ecat_id == right.ecat_id)
    throw std::invalid_argument("Arms must use different ecat_id values");
  if (left.id == right.id)
    throw std::invalid_argument("Arms must use different device IDs");
  DifferentPath(left.frames, right.frames, "frame paths");
  DifferentPath(left.payload, right.payload, "payload paths");
  DifferentPath(left.tool_library, right.tool_library, "tool libraries");
  if (fs::weakly_canonical(left.urdf) == fs::weakly_canonical(right.urdf)) {
    std::set<std::string> joints;
    for (const auto& drive : left.hardware->drives)
      joints.insert(drive.joint_name);
    for (const auto& drive : right.hardware->drives)
      if (joints.count(drive.joint_name))
        throw std::invalid_argument(
            "Arms map the same joint in the shared model: " + drive.joint_name);
  }
  if (root["http_host"]) config.http_host = root["http_host"].as<std::string>();
  if (root["http_port"]) config.http_port = root["http_port"].as<int>();
  if (root["http_control_ttl_seconds"])
    config.control_ttl_seconds = root["http_control_ttl_seconds"].as<int>();
  config.logging.directory = Resolve(base, "../../build/dualarm/logs");
  if (const auto logging = root["logging"]) {
    Keys(logging, {"directory", "file_level", "console_level"}, "logging");
    if (logging["directory"])
      config.logging.directory =
          Resolve(base, logging["directory"].as<std::string>());
    if (logging["file_level"])
      config.logging.file_level =
          parseLogLevel(logging["file_level"].as<std::string>());
    if (logging["console_level"])
      config.logging.console_level =
          parseLogLevel(logging["console_level"].as<std::string>());
  }
  config.diagnostics.host = "127.0.0.1";
  if (const auto diagnostics = root["diagnostics"]) {
    Keys(diagnostics, {"host", "port"}, "diagnostics");
    if (diagnostics["host"])
      config.diagnostics.host = diagnostics["host"].as<std::string>();
    if (diagnostics["port"])
      config.diagnostics.port = diagnostics["port"].as<int>();
  }
  if (config.http_host.empty() || config.http_port < 1 ||
      config.http_port > 65535 || config.control_ttl_seconds < 1 ||
      config.diagnostics.port == 0 ||
      config.http_port == config.diagnostics.port)
    throw std::invalid_argument("Invalid dual-arm host, ports or control TTL");
  config.logging.validate();
  config.diagnostics.validate();
  // Device snapshots must describe the services actually owned by this process.
  for (auto& arm : config.arms) {
    arm.http_host = config.http_host;
    arm.http_port = config.http_port;
    arm.http_control_ttl_seconds = config.control_ttl_seconds;
    arm.logging = config.logging;
    arm.diagnostics = config.diagnostics;
    arm.source_files.push_back(file.string());
    auto software = arm.effective["software"];
    auto logging = software["logging"];
    logging["directory"] = config.logging.directory;
    logging["file_level"] = logLevelName(config.logging.file_level);
    logging["console_level"] = logLevelName(config.logging.console_level);
    logging["max_file_size_mib"] = config.logging.max_file_size / (1024 * 1024);
    logging["max_files"] = config.logging.max_files;
    logging["flush_interval_ms"] = config.logging.flush_interval_ms;
    auto diagnostics = software["diagnostics"];
    diagnostics["host"] = config.diagnostics.host;
    diagnostics["port"] = config.diagnostics.port;
    diagnostics["max_sse_clients"] = config.diagnostics.max_sse_clients;
    diagnostics["max_downloads"] = config.diagnostics.max_downloads;
  }
  return config;
}

void DualArmConfig::ValidateResources() const {
  for (const auto& arm : arms) arm.validateResources();
  if (!model_urdf.empty()) ModelBundle(model_urdf, model_asset_root);
}
}  // namespace rocos::dualarm
