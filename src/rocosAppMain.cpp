#include <gflags/gflags.h>

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "app_config.hpp"
#include "lua_script_engine.hpp"
#include "robot.hpp"
#include "robot_http_server.hpp"

DEFINE_string(config, ROCOS_DEFAULT_CONFIG_FILE, "Path to the startup YAML configuration");
DEFINE_bool(check_config, false, "Validate configuration and model without connecting to hardware");
DEFINE_string(export_config, "", "Validate and export effective/master YAML to this directory, then exit");
DEFINE_string(http_host, "0.0.0.0", "Override HTTP server listen host from config");
DEFINE_int32(http_port, 8080, "Override HTTP server listen port from config");
DEFINE_int32(http_control_ttl_seconds, 300, "Override HTTP control rights TTL from config (positive)");

volatile std::sig_atomic_t isRuning = 1;
void signalHandler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) isRuning = 0;
}

int main(int argc, char* argv[]) {
    using namespace rocos;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    bool logging_initialized = false;
    try {
        AppConfig config = AppConfig::load(FLAGS_config);
        // 仅显式传入的命令行参数覆盖 YAML，未传入时使用文件配置。
        if (!gflags::GetCommandLineFlagInfoOrDie("http_host").is_default) config.http_host = FLAGS_http_host;
        if (!gflags::GetCommandLineFlagInfoOrDie("http_port").is_default) config.http_port = FLAGS_http_port;
        if (!gflags::GetCommandLineFlagInfoOrDie("http_control_ttl_seconds").is_default) {
            config.http_control_ttl_seconds = FLAGS_http_control_ttl_seconds;
        }
        config.validate();
        LogManager::configure(config.logging);
        logging_initialized = true;
        config.validateResources();
        if (FLAGS_check_config || !FLAGS_export_config.empty()) {
            if (!FLAGS_export_config.empty()) config.exportConfiguration(FLAGS_export_config);
            std::cout << "Configuration validated: " << std::filesystem::absolute(FLAGS_config).lexically_normal() << std::endl;
            LogManager::shutdown();
            return 0;
        }
        if (config.robot.composed_configuration) {
            const char* user_home = std::getenv("HOME");
            if (!user_home || !std::filesystem::path(user_home).is_absolute()) {
                throw std::runtime_error("HOME must be an absolute path to store configuration snapshots");
            }
            const auto snapshot = std::filesystem::path(user_home) / ".local/state/rocos/config_snapshots" /
                                  config.robot.id / "effective.yaml";
            config.writeSnapshot(snapshot.string());
        }
        auto& logs = LogManager::instance();
        DiagnosticsHttpServer diagnostics(logs, config.diagnostics);
        diagnostics.start();
        auto startup_log = Logger::getInstance("Startup");

        startup_log->info("Startup configuration loaded: {}", std::filesystem::absolute(FLAGS_config).lexically_normal().string());

        Robot robot(config.robot);
        if (signal(SIGINT, signalHandler) == SIG_ERR || signal(SIGTERM, signalHandler) == SIG_ERR) {
            startup_log->error("Cannot register process shutdown signal handler.");
        }
        LuaScriptEngine script_engine(robot, config.scripts);
        RobotHttpServer httpServer(&robot, &script_engine, config.http_control_ttl_seconds, config.web_root);
        httpServer.runAsync(config.http_host, config.http_port);

        startup_log->info("Control HTTP service listening on {}:{}; diagnostics listening on {}:{}.",
                          config.http_host, config.http_port, config.diagnostics.host, config.diagnostics.port);
        while (isRuning) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        robot.setDisabled();
        diagnostics.stop();
    } catch (const std::exception& error) {
        if (logging_initialized) {
            Logger::getInstance("Startup")->error("Startup failed (config={}): {}", FLAGS_config, error.what());
        } else {
            std::cerr << "[ERROR] Startup failed (config=" << FLAGS_config << "): " << error.what() << std::endl;
        }
        LogManager::shutdown();
        return 1;
    }
    LogManager::shutdown();
    return 0;
}
