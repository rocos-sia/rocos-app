#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "src/app_config.hpp"
#include "test/doctest.h"

#include <cstdlib>
#include <fstream>

namespace {
namespace fs = std::filesystem;

struct ConfigFixture {
    fs::path directory;
    fs::path original_directory = fs::current_path();

    ConfigFixture() {
        char pattern[] = "/tmp/rocos-app-config-XXXXXX";
        const char* path = mkdtemp(pattern);
        REQUIRE(path != nullptr);
        directory = path;
    }

    ~ConfigFixture() {
        fs::current_path(original_directory);
        fs::remove_all(directory);
    }

    rocos::AppConfig load(const std::string& text) {
        const auto file = directory / "config.yaml";
        std::ofstream(file) << text;
        return rocos::AppConfig::load(file.string());
    }

    void write(const fs::path& path, const YAML::Node& node) {
        fs::create_directories(path.parent_path());
        std::ofstream(path) << node;
    }

    fs::path composed() {
        const auto source = fs::path(ROCOS_TEST_CONFIG_FILE).parent_path();
        auto app = YAML::LoadFile((source / "app.yaml").string());
        app["robot_config"] = "robots/talon/robot.yaml";
        app["device_config"] = "devices/talon_001/device.yaml";
        app["user_config"] = "overrides/user.yaml";
        app["scripts"] = source.string() + "/scripts";
        app["web_root"] = (source.parent_path() / "web").string();
        app["logging"]["directory"] = "logs";
        app["tool_library"] = "library";
        write(directory / "app.yaml", app);
        auto robot = YAML::LoadFile((source / "robots/talon/robot.yaml").string());
        robot["urdf"] = (source / "robots/talon/robot.urdf").string();
        write(directory / "robots/talon/robot.yaml", robot);
        write(directory / "robots/talon/hardware.yaml", YAML::LoadFile((source / "robots/talon/hardware.yaml").string()));
        auto master = YAML::LoadFile((source / "robots/talon/ecm.yaml").string());
        master["master"]["eni"] = (source / "robots/talon/eni7.xml").string();
        master["master"]["license"] = "test-license";
        master["master"]["sp"] = YAML::Node(YAML::NodeType::Null);
        write(directory / "robots/talon/ecm.yaml", master);
        write(directory / "devices/talon_001/device.yaml", YAML::LoadFile((source / "devices/talon_001/device.yaml").string()));
        write(directory / "overrides/user.yaml", YAML::Load("{}"));
        return directory / "app.yaml";
    }

    void overrideWith(const std::string& yaml) {
        std::ofstream(directory / "overrides/user.yaml") << yaml;
    }
};
}  // namespace

TEST_CASE_FIXTURE(ConfigFixture, "AppConfig loads startup parameters and resolves paths beside YAML") {
    fs::create_directory(directory / "unrelated-cwd");
    fs::current_path(directory / "unrelated-cwd");
    const auto config = load(R"(
http_host: 127.0.0.1
http_port: 12345
http_control_ttl_seconds: 30
robot:
  hardware_config: hardware/driver.yaml
  ecat_id: 2
  urdf: models/robot.urdf
  base_link: base
  tip_link: tool
  joint_binding: bindings.yaml
  payload: load.yaml
  frames: tools/calibration.yaml
  id: robot_A
  tool_library: ../tool-library
scripts: lua
web_root: ../web
)");
    CHECK(config.http_host == "127.0.0.1");
    CHECK(config.http_port == 12345);
    CHECK(config.http_control_ttl_seconds == 30);
    CHECK(config.robot.ecat_id == 2);
    CHECK(config.robot.base_link == "base");
    CHECK(config.robot.tip_link == "tool");
    CHECK(config.robot.hardware_config == (directory / "hardware/driver.yaml").string());
    CHECK(config.robot.urdf == (directory / "models/robot.urdf").string());
    CHECK(config.robot.joint_binding == (directory / "bindings.yaml").string());
    CHECK(config.robot.payload == (directory / "load.yaml").string());
    CHECK(config.robot.frames == (directory / "tools/calibration.yaml").string());
    CHECK(config.robot.id == "robot_A");
    CHECK(config.robot.tool_library == (directory.parent_path() / "tool-library").string());
    CHECK(config.scripts == (directory / "lua").string());
    CHECK(config.web_root == (directory.parent_path() / "web").string());
    CHECK(fs::current_path() == directory / "unrelated-cwd");
}

TEST_CASE_FIXTURE(ConfigFixture, "AppConfig supports absolute resource paths and relative config paths") {
    fs::current_path(directory);
    const auto config = load("robot:\n  urdf: /opt/robot/model.urdf\nscripts: /opt/robot/scripts\n");
    CHECK(config.robot.urdf == "/opt/robot/model.urdf");
    CHECK(config.scripts == "/opt/robot/scripts");
    const auto relative = rocos::AppConfig::load("config.yaml");
    CHECK(relative.robot.hardware_config == (directory / "hardware_talon_config.yaml").string());
    CHECK(relative.robot.urdf == config.robot.urdf);
    CHECK(relative.http_port == 8080);
    CHECK(relative.http_control_ttl_seconds == 300);
}

TEST_CASE_FIXTURE(ConfigFixture, "AppConfig rejects missing malformed and invalid settings") {
    CHECK_THROWS(rocos::AppConfig::load((directory / "missing.yaml").string()));
    for (const auto* yaml :
         {"", "[]", "http_port: [", "http_port: invalid", "http_port: 0", "http_port: 65536", "http_host: ''",
          "http_control_ttl_seconds: 0", "http_control_ttl_seconds: -1", "robot: []", "robot:\n  ecat_id: -1",
          "robot:\n  urdf: ''", "robot:\n  base_link: ''", "scripts: ''"}) {
        CAPTURE(yaml);
        CHECK_THROWS(load(yaml));
    }
}

TEST_CASE("Repository startup config points to existing source resources") {
    const auto config = rocos::AppConfig::load(ROCOS_TEST_CONFIG_FILE);
    CHECK(std::filesystem::is_regular_file(config.robot.hardware_config));
    CHECK(std::filesystem::is_regular_file(config.robot.urdf));
    CHECK(config.robot.joint_binding.empty());
    REQUIRE(config.robot.hardware.has_value());
    CHECK(config.robot.hardware->drives.size() == 7);
    CHECK(config.robot.composed_configuration);
    CHECK_NOTHROW(config.validateResources());
    CHECK(std::filesystem::is_directory(config.scripts));
    CHECK(std::filesystem::is_directory(config.web_root));
}

TEST_CASE_FIXTURE(ConfigFixture, "Logging and diagnostics resolve paths and validate independent limits") {
    auto config = load(R"(
logging:
  directory: diagnostic-logs
  file_level: trace
  console_level: critical
  max_file_size_mib: 2
  max_files: 3
  flush_interval_ms: 20
diagnostics:
  host: 127.0.0.1
  port: 8099
  max_sse_clients: 2
  max_downloads: 1
)");
    CHECK(config.logging.directory == (directory / "diagnostic-logs").string());
    CHECK(config.logging.file_level == rocos::LogLevel::Trace);
    CHECK(config.logging.console_level == rocos::LogLevel::Critical);
    CHECK(config.logging.max_file_size == 2 * 1024 * 1024);
    CHECK(config.logging.max_files == 3);
    CHECK(config.diagnostics.port == 8099);
    CHECK(config.diagnostics.max_sse_clients == 2);
    for (auto yaml : {"logging: []", "logging: {directory: ''}", "logging: {file_level: error}",
                      "logging: {console_level: unknown}", "logging: {max_file_size_mib: -1}",
                      "logging: {max_files: 0}", "logging: {flush_interval_ms: 0}", "diagnostics: []",
                      "diagnostics: {port: 8080}", "diagnostics: {port: 0}", "diagnostics: {max_sse_clients: 0}",
                      "diagnostics: {max_downloads: 17}"}) {
        CAPTURE(yaml);
        CHECK_THROWS(load(yaml));
    }
}

TEST_CASE_FIXTURE(ConfigFixture, "Composed paths follow each declaring file and selectors precede reads") {
    const auto entry = composed();
    // 基础选择器故意无效，由 user 替换后不得尝试读取旧路径。
    auto app = YAML::LoadFile(entry.string());
    app["robot_config"] = "missing/robot.yaml";
    write(entry, app);
    overrideWith(R"(
software:
  robot_config: ../robots/talon/robot.yaml
  logging: {directory: local-logs}
  http_port: 18080
robot:
  master: {log: ethercat/master, duration: 0, ctloff: false}
)");
    fs::current_path("/tmp");
    const auto config = rocos::AppConfig::load(entry.string());
    CHECK(config.robot.hardware_config == (directory / "robots/talon/hardware.yaml").string());
    CHECK(config.logging.directory == (directory / "overrides/local-logs").string());
    CHECK(config.http_port == 18080);
    CHECK(config.effective["robot"]["master"]["log"].as<std::string>() == (directory / "overrides/ethercat/master").string());
    CHECK(config.effective["robot"]["master"]["duration"].as<int>() == 0);
    CHECK_FALSE(config.effective["robot"]["master"]["ctloff"].as<bool>());
    CHECK(config.effective["software"]["logging"]["max_files"].as<int>() == 20);
    CHECK(config.robot.initial_payload->first == 0);
    CHECK(config.effective["tool"].IsNull());
    CHECK_NOTHROW(config.validateResources());
}

TEST_CASE_FIXTURE(ConfigFixture, "Strict schema rejects typo duplicate null and identity overrides") {
    const auto entry = composed();
    for (const auto* yaml : {
        "software: {http_port: 8081, http_port: 8083}",
        "software: {http_ports: 8081}", "software: {user_config: other.yaml}",
        "software: {http_port: null}", "software: {logging: null}",
        "robot: {name: another}", "robot: {master: {cycle: 0}}",
        "robot: {master: {state: ready}}", "robot: {master: {dcmmode: 1}}",
        "robot: {master: {auxclk: 0}}", "robot: {master: {flash: 65535}}",
        "robot: {master: {id: 1}}", "robot: {master: {sp: 0}}",
        "robot: {binding: []}", "device: {id: another}",
        "device: {calibration: {payload: {mass: 0}}}",
        "device: {tool_install: {parent_link: link_7, position: [0,0,0], orientation: [0,0,0,1]}}",
        "tool: {tcp: {position: [0,0,0]}}", "snapshot: {}"
    }) {
        CAPTURE(yaml);
        overrideWith(yaml);
        CHECK_THROWS(rocos::AppConfig::load(entry.string()));
    }
}

TEST_CASE_FIXTURE(ConfigFixture, "Hardware arrays replace entirely and bindings never fall back to order") {
    const auto entry = composed();
    overrideWith("robot: {hardware: [{type: driver, id: 0, joint_name: joint_1}]}\n");
    auto config = rocos::AppConfig::load(entry.string());
    CHECK(config.robot.hardware->drives.size() == 1);
    CHECK(config.effective["robot"]["hardware"].size() == 1);
    CHECK_THROWS(config.validateResources());
    auto hardware = YAML::LoadFile((directory / "robots/talon/hardware.yaml").string());
    hardware["hardware"][0]["joint_name"] = "nonexistent_joint";
    YAML::Node patch;
    patch["robot"]["hardware"] = hardware["hardware"];
    write(directory / "overrides/user.yaml", patch);
    config = rocos::AppConfig::load(entry.string());
    CHECK_THROWS(config.validateResources());
    hardware["hardware"][0]["joint_name"] = hardware["hardware"][1]["joint_name"].as<std::string>();
    patch["robot"]["hardware"] = hardware["hardware"];
    write(directory / "overrides/user.yaml", patch);
    CHECK_THROWS(rocos::AppConfig::load(entry.string()));
}

TEST_CASE_FIXTURE(ConfigFixture, "Device zero calibration replaces nominal counts with contextual user override") {
    const auto entry = composed();
    auto hardware = YAML::LoadFile((directory / "robots/talon/hardware.yaml").string());
    const auto name = hardware["hardware"][0]["joint_name"].as<std::string>();
    hardware["hardware"][0]["transform"]["offset_pos_cnt"] = 300;
    write(directory / "robots/talon/hardware.yaml", hardware);
    auto record = YAML::Load("schema_version: 1\ncontext: {device_id: talon_001, robot: {name: talon, package_version: 1.0.0}}\n");
    record["joints"][name]["offset_pos_cnt"] = 100;
    write(directory / "overrides/measured/zero.yaml", record);
    YAML::Node patch;
    patch["device"]["calibration_files"]["joint_zero"] = "measured/zero.yaml";
    patch["device"]["calibration"]["joint_zero"][name]["offset_pos_cnt"] = 0;
    write(directory / "overrides/user.yaml", patch);
    const auto config = rocos::AppConfig::load(entry.string());
    CHECK(config.robot.hardware->drives[0].transform.offset_pos_cnt == 0);
    CHECK(config.effective["device"]["calibration"]["joint_zero"][name]["offset_pos_cnt"].as<int>() == 0);
    CHECK(YAML::LoadFile((directory / "overrides/measured/zero.yaml").string())["joints"][name]["offset_pos_cnt"].as<int>() == 100);
    record["context"]["device_id"] = "another_device";
    write(directory / "overrides/measured/zero.yaml", record);
    CHECK_THROWS(rocos::AppConfig::load(entry.string()));
}

TEST_CASE_FIXTURE(ConfigFixture, "Exports preserve inputs and redact license only in effective snapshot") {
    const auto entry = composed();
    auto config = rocos::AppConfig::load(entry.string());
    config.http_port = 12349;  // 模拟显式 CLI 覆盖。
    const auto output = directory / "export";
    config.exportConfiguration(output.string());
    const auto snapshot = YAML::LoadFile((output / "effective.yaml").string());
    CHECK(snapshot["software"]["http_port"].as<int>() == 12349);
    CHECK(snapshot["robot"]["master"]["license"].as<std::string>() == "<redacted>");
    CHECK(snapshot["robot"]["hardware"].size() == 7);
    CHECK(snapshot["snapshot"]["source_files"].size() == 6);
    CHECK(snapshot["snapshot"]["user_override_fields"].size() == 0);
    CHECK_FALSE(snapshot["robot"]["master"]["master"].IsDefined());
    const auto native = YAML::LoadFile((output / "ecm.yaml").string());
    CHECK(native["master"]["license"].as<std::string>() == "test-license");
    const auto legacy = YAML::LoadFile((output / "ecm-legacy.yaml").string());
    CHECK(legacy["dcmmode"].as<int>() == 1);
    CHECK(legacy["mode"].as<int>() == 1);
    CHECK_FALSE(legacy["sp"].IsDefined());
    CHECK_FALSE(legacy["flash"].IsDefined());
    CHECK((fs::status(output / "effective.yaml").permissions() & fs::perms::owner_write) == fs::perms::none);
    CHECK((fs::status(output / "ecm-legacy.yaml").permissions() & fs::perms::others_read) == fs::perms::none);
    CHECK_THROWS(config.exportConfiguration((directory / "robots/talon").string()));
    CHECK_THROWS(config.writeSnapshot(entry.string()));
    std::ofstream(directory / "model.xml") << "opaque resource";
    config.robot.urdf = (directory / "model.xml").string();
    CHECK_THROWS(config.writeSnapshot(config.robot.urdf));
    CHECK(YAML::LoadFile((directory / "robots/talon/ecm.yaml").string())["master"]["license"].as<std::string>() == "test-license");
}

TEST_CASE_FIXTURE(ConfigFixture, "Tool installation and measured local TCP compose once without legacy frames") {
    const auto entry = composed();
    const auto source = fs::path(ROCOS_TEST_CONFIG_FILE).parent_path();
    auto patch = YAML::Load(R"(
device:
  tool_install: {parent_link: link_7, position: [0.1, 0, 0], orientation: [0, 0, 0.7071067811865476, 0.7071067811865476]}
  calibration_files: {payload: results/payload.yaml, tool_tcp: results/tcp.yaml}
  calibration: {payload: {mass: 0}}
tool:
  tcp: {position: [0, 0, 0.5]}
)");
    patch["device"]["tool_config"] = (source / "tools/demo_gripper/tool.yaml").string();
    write(directory / "overrides/user.yaml", patch);
    auto record = YAML::Load(R"(
schema_version: 1
context:
  device_id: talon_001
  robot: {name: talon, package_version: 1.0.0}
  tool: {name: demo_gripper, package_version: 1.0.0}
)");
    record["context"]["tool_install"] = patch["device"]["tool_install"];
    auto payload = YAML::Clone(record);
    payload["mass"] = 1.5;
    payload["com"] = YAML::Load("[0, 0, 0.02]");
    write(directory / "overrides/results/payload.yaml", payload);
    record["tcp"] = YAML::Load("{position: [0.2, 0, 0], orientation: [0, 0, 0, 1]}");
    write(directory / "overrides/results/tcp.yaml", record);
    const auto config = rocos::AppConfig::load(entry.string());
    REQUIRE(config.robot.tool.has_value());
    CHECK(config.robot.initial_payload->first == 0);
    CHECK(config.robot.payload_overridden);
    CHECK(config.robot.payload_metadata["context"]["device_id"].as<std::string>() == "talon_001");
    CHECK(fs::path(config.effective["tool"]["model"]["urdf"].as<std::string>()).is_absolute());
    CHECK(config.robot.tool->tcp.p.x() == doctest::Approx(0.2));
    CHECK(config.robot.tool->tcp.p.z() == doctest::Approx(0));
    // 旧 frames 即使存在也不是新启动的覆盖来源。
    std::ofstream(config.robot.frames) << "invalid legacy frames";
    rocos::FrameRegistry frames(config.robot.frames, config.robot.tool_library, config.robot.id);
    frames.initializePackage(*config.robot.tool);
    const auto snapshot = frames.snapshot();
    CHECK(snapshot->active_tool == "demo_gripper");
    CHECK(snapshot->tools.at("demo_gripper").p.x() == doctest::Approx(0.1));
    CHECK(snapshot->tools.at("demo_gripper").p.y() == doctest::Approx(0.2));
    CHECK(KDL::Equal(snapshot->tools.at("demo_gripper"), config.robot.tool->mount * config.robot.tool->tcp, 1e-12));
    record["context"]["tool_install"]["position"] = YAML::Load("[0,0,0]");
    write(directory / "overrides/results/tcp.yaml", record);
    CHECK_THROWS(rocos::AppConfig::load(entry.string()));
}

TEST_CASE_FIXTURE(ConfigFixture, "IgH normalizes only the selected backend and retains PDO structure on export") {
    const auto entry = composed();
    auto master = YAML::Load(R"(
version: 2
master: {id: 0, cycle: 1000, state: safeop, dc: 'off'}
slaves:
  - id: 0
    name: test_slave
    rx_pdos:
      - index: 0x1600
        entries: [{name: Control word, index: 0x6040, sub_index: 0, bit_length: 16}]
    tx_pdos:
      - index: 0x1a00
        entries: [{name: Status word, index: 0x6041, sub_index: 0, bit_length: 16}]
    init_commands: [{index: 0x6060, sub_index: 0, type: int8, value: -1}]
)");
    write(directory / "overrides/igh.yaml", master);
    fs::remove(directory / "robots/talon/ecm.yaml");
    overrideWith("robot: {master_type: igh, master_config: igh.yaml, master: {duration: 0, verbose: warnings}}\n");
    const auto config = rocos::AppConfig::load(entry.string());
    CHECK(config.robot.master_state == "safeop");
    CHECK(config.effective["robot"]["master"]["slaves"].size() == 1);
    CHECK(config.effective["robot"]["master"]["state_timeout_ms"].as<int>() == 60000);
    CHECK_FALSE(config.effective["robot"]["master"]["eni"].IsDefined());
    config.exportConfiguration((directory / "export").string());
    const auto exported = YAML::LoadFile((directory / "export/igh.yaml").string());
    CHECK(exported["version"].as<int>() == 2);
    CHECK(exported["slaves"][0]["init_commands"][0]["value"].as<int>() == -1);
    CHECK_FALSE(exported["master"]["slaves"].IsDefined());
    master["slaves"][0]["init_commands"][0]["value"] = 128;
    write(directory / "overrides/igh.yaml", master);
    CHECK_THROWS(rocos::AppConfig::load(entry.string()));
    master["slaves"][0]["init_commands"][0]["type"] = "uint64";
    master["slaves"][0]["init_commands"][0]["value"] = -1;
    write(directory / "overrides/igh.yaml", master);
    CHECK_THROWS(rocos::AppConfig::load(entry.string()));
}
