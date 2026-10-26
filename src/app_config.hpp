#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot.hpp"
#include "diagnostics_http_server.hpp"

namespace rocos {

struct AppConfig {
    std::string http_host{"0.0.0.0"};
    int http_port{8080};
    int http_control_ttl_seconds{300};
    LoggingConfig logging;
    DiagnosticsConfig diagnostics;
    Robot::Config robot;
    std::string scripts{"scripts"};
    std::string web_root{"../web"};
    // 新格式的完整有效配置；旧入口保持原有读取语义。
    YAML::Node effective;
    std::vector<std::string> source_files;
    std::string entry_file;

    static AppConfig load(const std::string& filename);
    void validateResources() const;
    void writeSnapshot(const std::string& filename) const;
    void exportConfiguration(const std::string& directory) const;

    void validate() const {
        logging.validate();
        diagnostics.validate();
        if (diagnostics.port == 0 || diagnostics.port == http_port) throw std::invalid_argument("diagnostics.port must be nonzero and differ from http_port");
        if (http_host.empty()) throw std::invalid_argument("http_host must not be empty");
        if (http_port < 1 || http_port > 65535) throw std::invalid_argument("http_port must be in [1, 65535]");
        if (http_control_ttl_seconds <= 0) {
            throw std::invalid_argument("http_control_ttl_seconds must be positive");
        }
        if (robot.ecat_id < 0) throw std::invalid_argument("robot.ecat_id must be nonnegative");
        if (robot.base_link.empty() || robot.tip_link.empty()) {
            throw std::invalid_argument("robot.base_link and robot.tip_link must not be empty");
        }
        if (!FrameRegistry::validName(robot.id)) throw std::invalid_argument("robot.id must be a valid stable name");
    }

    static AppConfig loadLegacy(const std::string& filename) {
        namespace fs = std::filesystem;
        const fs::path config_path = fs::absolute(filename).lexically_normal();
        const YAML::Node root = YAML::LoadFile(config_path.string());
        if (!root.IsMap()) throw std::invalid_argument("config.yaml must contain a mapping");

        // 缺项使用默认值，存在但类型错误的字段必须报错。
        auto read = [](const YAML::Node& node, const char* key, auto fallback) {
            return node[key] ? node[key].template as<decltype(fallback)>() : fallback;
        };
        AppConfig config;
        config.http_host = read(root, "http_host", config.http_host);
        config.http_port = read(root, "http_port", config.http_port);
        config.http_control_ttl_seconds = read(root, "http_control_ttl_seconds", config.http_control_ttl_seconds);
        config.scripts = read(root, "scripts", config.scripts);
        config.web_root = read(root, "web_root", config.web_root);
        const auto robot = root["robot"];
        if (robot) {
            if (!robot.IsMap()) throw std::invalid_argument("robot must contain a mapping");
            config.robot.hardware_config = read(robot, "hardware_config", config.robot.hardware_config);
            config.robot.ecat_id = read(robot, "ecat_id", config.robot.ecat_id);
            config.robot.urdf = read(robot, "urdf", config.robot.urdf);
            config.robot.base_link = read(robot, "base_link", config.robot.base_link);
            config.robot.tip_link = read(robot, "tip_link", config.robot.tip_link);
            config.robot.joint_binding = read(robot, "joint_binding", config.robot.joint_binding);
            config.robot.payload = read(robot, "payload", config.robot.payload);
            config.robot.frames = read(robot, "frames", config.robot.frames);
            config.robot.tool_library = read(robot, "tool_library", config.robot.tool_library);
            config.robot.id = read(robot, "id", config.robot.id);
        }
        const auto logging = root["logging"];
        if (logging) {
            if (!logging.IsMap()) throw std::invalid_argument("logging must contain a mapping");
            config.logging.directory = read(logging, "directory", std::string("../logs"));
            config.logging.file_level = parseLogLevel(read(logging, "file_level", std::string("info")));
            config.logging.console_level = parseLogLevel(read(logging, "console_level", std::string("info")));
            const auto mib = read(logging, "max_file_size_mib", 50);
            const auto files = read(logging, "max_files", 20);
            if (mib < 1 || mib > 1024 || files < 1) throw std::invalid_argument("Invalid logging file limits");
            config.logging.max_file_size = static_cast<size_t>(mib) * 1024 * 1024;
            config.logging.max_files = static_cast<size_t>(files);
            config.logging.flush_interval_ms = read(logging, "flush_interval_ms", 1000);
        } else config.logging.directory = "../logs";
        const auto diagnostics = root["diagnostics"];
        if (diagnostics) {
            if (!diagnostics.IsMap()) throw std::invalid_argument("diagnostics must contain a mapping");
            config.diagnostics.host = read(diagnostics, "host", config.diagnostics.host);
            config.diagnostics.port = read(diagnostics, "port", config.diagnostics.port);
            const auto clients = read(diagnostics, "max_sse_clients", 4);
            const auto downloads = read(diagnostics, "max_downloads", 1);
            if (clients < 1 || downloads < 1) throw std::invalid_argument("Invalid diagnostics quotas");
            config.diagnostics.max_sse_clients = clients;
            config.diagnostics.max_downloads = downloads;
        }
        // 相对路径统一以启动配置所在目录为基准，不依赖进程工作目录。
        auto resolve = [&](std::string& path) {
            if (path.empty()) throw std::invalid_argument("configured paths must not be empty");
            path = (config_path.parent_path() / fs::path(path)).lexically_normal().string();
        };
        resolve(config.robot.hardware_config);
        resolve(config.robot.urdf);
        resolve(config.robot.joint_binding);
        resolve(config.robot.payload);
        resolve(config.robot.frames);
        resolve(config.robot.tool_library);
        resolve(config.scripts);
        resolve(config.web_root);
        resolve(config.logging.directory);
        config.validate();
        return config;
    }
};

}  // namespace rocos
