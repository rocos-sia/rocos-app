#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>

#include "dualarm_http_server.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void OnSignal(int) { stop_requested = 1; }
}  // namespace
int main(int argc, char** argv) {
  using namespace rocos;
  using namespace rocos::dualarm;
  std::string file = "dualarm/config/app.yaml";
  bool check = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--check_config")
      check = true;
    else if (arg == "--config" && i + 1 < argc)
      file = argv[++i];
    else if (arg.rfind("--config=", 0) == 0)
      file = arg.substr(9);
    else if (arg == "--help") {
      std::cout << "rocosDualArmMain [--config FILE] [--check_config]\n";
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << '\n';
      return 1;
    }
  }
  bool logging = false;
  int result = 0;
  try {
    auto config = DualArmConfig::Load(file);
    LogManager::configure(config.logging);
    logging = true;
    LogManager::instance().enableDetailedEvents(
        {config.arms[0].robot.id, config.arms[1].robot.id});
    config.ValidateResources();
    if (check) {
      std::cout << "Dual-arm configuration validated: "
                << std::filesystem::absolute(file)
                << " (left=" << config.arms[0].robot.ecat_id
                << ", right=" << config.arms[1].robot.ecat_id << ")\n";
    } else {
      const char* user_home = std::getenv("HOME");
      if (!user_home || !std::filesystem::path(user_home).is_absolute())
        throw std::runtime_error("HOME must be absolute for device snapshots");
      for (const auto& arm : config.arms) {
        const auto snapshot = std::filesystem::path(user_home) /
                              ".local/state/rocos/config_snapshots" /
                              arm.robot.id / "effective.yaml";
        arm.writeSnapshot(snapshot.string());
      }
      std::signal(SIGINT, OnSignal);
      std::signal(SIGTERM, OnSignal);
      // Ownership order guarantees HTTP workers stop before either Robot dies.
      DualArmController controller(config);
      DiagnosticsHttpServer diagnostics(LogManager::instance(),
                                        config.diagnostics);
      diagnostics.start();
      DualArmHttpServer server(controller);
      server.Start();
      Logger::getInstance("DualArm.Startup")
          ->info("Two Robots ready; HTTP {}:{}; diagnostics {}:{}",
                 config.http_host, config.http_port, config.diagnostics.host,
                 config.diagnostics.port);
      while (!stop_requested && server.Running())
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (!stop_requested) result = 1;
      server.Stop();
      if (!controller.Shutdown()) {
        std::cerr << "Dual-arm shutdown did not confirm both arms disabled\n";
        Logger::getInstance("DualArm.Shutdown")
            ->error("Shutdown did not confirm both arms disabled");
        result = 1;
      }
      diagnostics.stop();
    }
  } catch (const std::exception& error) {
    std::cerr << "Dual-arm startup/runtime failed: " << error.what() << '\n';
    if (logging)
      Logger::getInstance("DualArm.Startup")
          ->error("Startup/runtime failed: {}", error.what());
    result = 1;
  }
  if (logging) LogManager::shutdown();
  return result;
}
