#pragma once
#include <array>
#include <string>

#include "app_config.hpp"

namespace rocos::dualarm {
struct DualArmConfig {
  std::array<AppConfig, 2> arms;
  std::string http_host = "127.0.0.1";
  int http_port = 8080;
  int control_ttl_seconds = 300;
  std::string model_urdf;
  std::string model_asset_root;
  LoggingConfig logging;
  DiagnosticsConfig diagnostics;
  static DualArmConfig Load(const std::string& filename);
  void ValidateResources() const;
};
}  // namespace rocos::dualarm
