#include "app_config.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>

#include "model.hpp"
#include "ver.h"

namespace rocos {
namespace {
namespace fs = std::filesystem;
using Node = YAML::Node;

void require(bool value, const std::string& message) {
    if (!value) throw std::invalid_argument(message);
}
void keys(const Node& node, std::initializer_list<const char*> allowed, const std::string& where) {
    require(node.IsMap(), where + " must be a mapping");
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        require(std::find(allowed.begin(), allowed.end(), key) != allowed.end(), where + ": unknown field " + key);
    }
}
void uniqueKeys(const Node& node, const std::string& where, unsigned depth = 0) {
    require(depth < 64, where + ": excessive YAML nesting");
    if (node.IsMap()) {
        std::set<std::string> found;
        for (const auto& item : node) {
            require(item.first.IsScalar(), where + ": mapping keys must be strings");
            const auto key = item.first.as<std::string>();
            require(found.insert(key).second, where + ": duplicate field " + key);
            uniqueKeys(item.second, where + "." + key, depth + 1);
        }
    } else if (node.IsSequence()) {
        for (size_t i = 0; i < node.size(); ++i) uniqueKeys(node[i], where + "[" + std::to_string(i) + "]", depth + 1);
    }
}
Node readFile(const std::string& path, std::vector<std::string>& sources) {
    require(fs::is_regular_file(path), "Configuration file not found: " + path);
    Node root = YAML::LoadFile(path);
    require(root.IsMap(), path + " must contain a mapping");
    uniqueKeys(root, path);
    sources.push_back(path);
    return root;
}
void schema(const Node& node, const std::string& file) {
    require(node["schema_version"] && node["schema_version"].as<int>() == 1, file + ": schema_version must be 1");
}
Node merge(const Node& base, const Node& patch) {
    if (!patch.IsDefined()) return YAML::Clone(base);
    if (!base.IsDefined()) return YAML::Clone(patch);
    if (!base.IsMap() || !patch.IsMap()) return YAML::Clone(patch);
    Node result = YAML::Clone(base);
    for (const auto& item : patch) {
        const auto key = item.first.as<std::string>();
        result[key] = merge(static_cast<const Node&>(base)[key], item.second);
    }
    return result;
}
Node child(const Node& node, const char* key) {
    return node.IsMap() && node[key].IsDefined() ? node[key] : Node(YAML::NodeType::Undefined);
}
void path(Node node, const char* key, const std::string& file, bool nullable = false, bool empty = false) {
    const auto value = static_cast<const Node&>(node)[key];
    if (!value.IsDefined()) return;
    if (value.IsNull() && nullable) return;
    require(value.IsScalar(), file + ": " + key + " must be a path string");
    const auto text = value.as<std::string>();
    if (empty && text.empty()) return;
    require(!text.empty(), file + ": " + key + " must not be empty");
    node[key] = (fs::path(file).parent_path() / text).lexically_normal().string();
}
std::string string(const Node& node, const char* key, const std::string& where, bool empty = false) {
    const auto value = node[key];
    require(value.IsScalar(), where + "." + key + " must be a string");
    const auto result = value.as<std::string>();
    require(empty || !result.empty(), where + "." + key + " must not be empty");
    return result;
}
int64_t integer(const Node& node, const char* key, int64_t low, int64_t high, const std::string& where) {
    try {
        const auto value = node[key].as<int64_t>();
        require(value >= low && value <= high, where + "." + key + " is out of range");
        return value;
    } catch (const YAML::Exception&) {
        throw std::invalid_argument(where + "." + key + " must be an integer in range");
    }
}
double number(const Node& node, const char* key, const std::string& where) {
    const auto value = node[key].as<double>();
    require(std::isfinite(value), where + "." + key + " must be finite");
    return value;
}
void choice(const Node& node, const char* key, std::initializer_list<const char*> allowed, const std::string& where) {
    const auto value = string(node, key, where);
    require(std::find(allowed.begin(), allowed.end(), value) != allowed.end(),
            where + "." + key + ": invalid value " + value);
}
void poseKeys(const Node& node, const std::string& where, bool parent = false) {
    if (parent)
        keys(node, {"parent_link", "position", "orientation"}, where);
    else
        keys(node, {"position", "orientation"}, where);
}
void softwareKeys(const Node& node, const std::string& where, bool entry) {
    if (entry)
        keys(node,
             {"schema_version", "robot_config", "device_config", "user_config", "http_host", "http_port",
              "http_control_ttl_seconds", "scripts", "web_root", "tool_library", "logging", "diagnostics"},
             where);
    else
        keys(node,
             {"robot_config", "device_config", "http_host", "http_port", "http_control_ttl_seconds", "scripts",
              "web_root", "tool_library", "logging", "diagnostics"},
             where);
    if (node["logging"])
        keys(node["logging"],
             {"directory", "file_level", "console_level", "max_file_size_mib", "max_files", "flush_interval_ms"},
             where + ".logging");
    if (node["diagnostics"])
        keys(node["diagnostics"], {"host", "port", "max_sse_clients", "max_downloads"}, where + ".diagnostics");
}
void softwarePaths(Node node, const std::string& file) {
    for (auto key : {"robot_config", "device_config", "scripts", "web_root", "tool_library"}) path(node, key, file);
    if (node["logging"]) path(node["logging"], "directory", file);
}
void robotKeys(const Node& node, const std::string& where, bool manifest) {
    if (manifest)
        keys(node,
             {"schema_version", "name", "package_version", "urdf", "base_link", "tip_link", "ecat_id",
              "hardware_config", "master_type", "master_config"},
             where);
    else
        keys(node,
             {"urdf", "base_link", "tip_link", "ecat_id", "hardware_config", "master_type", "master_config", "hardware",
              "master"},
             where);
}
void robotPaths(Node node, const std::string& file) {
    for (auto key : {"urdf", "hardware_config", "master_config"}) path(node, key, file);
    if (node["master"] && node["master"].IsMap()) {
        path(node["master"], "eni", file);
        path(node["master"], "log", file, false, true);
    }
}
void deviceKeys(const Node& node, const std::string& where, bool manifest) {
    if (manifest)
        keys(node, {"schema_version", "id", "expected_robot", "tool_config", "tool_install", "calibration_files"},
             where);
    else
        keys(node, {"tool_config", "tool_install", "calibration_files", "calibration"}, where);
    if (node["expected_robot"]) keys(node["expected_robot"], {"name", "package_version"}, where + ".expected_robot");
    if (node["tool_install"] && !node["tool_install"].IsNull())
        poseKeys(node["tool_install"], where + ".tool_install", true);
    if (node["calibration_files"])
        keys(node["calibration_files"], {"joint_zero", "kinematics", "dynamics", "payload", "tool_tcp"},
             where + ".calibration_files");
    if (node["calibration"]) keys(node["calibration"], {"joint_zero", "payload", "tool_tcp"}, where + ".calibration");
}
void devicePaths(Node node, const std::string& file) {
    path(node, "tool_config", file, true);
    if (node["calibration_files"])
        for (auto key : {"joint_zero", "kinematics", "dynamics", "payload", "tool_tcp"})
            path(node["calibration_files"], key, file, true);
}
Node masterConfig(Node master, const std::string& type, const std::string& where) {
    const bool ecm = type == "ecm";
    if (ecm)
        keys(master,
             {"id", "eni", "duration", "cycle", "verbose", "cpuidx", "perf", "sp", "log", "dcmmode", "ctloff", "link",
              "instance", "mode", "state", "license", "maxbusslaves", "mbxsrv"},
             where);
    else
        keys(master,
             {"id", "duration", "cycle", "verbose", "cpuidx", "state", "dc", "preop_timeout_ms", "state_timeout_ms",
              "pdo_config_attempts", "slaves"},
             where);
    Node defaults = YAML::Load("id: 0\nduration: 0\ncycle: 1000\nverbose: diagnostic\nstate: op\n");
    const auto backend = YAML::Load(
        ecm ? "perf: timing\nsp: null\nlog: ''\ndcmmode: bus_shift\nctloff: false\nlink: IntelGbe\ninstance: 1\nmode: "
              "polling\nlicense: ''\nmaxbusslaves: 256\nmbxsrv: null\n"
            : "cpuidx: null\ndc: 'off'\npreop_timeout_ms: 10000\nstate_timeout_ms: 60000\npdo_config_attempts: 5\n");
    master = merge(merge(defaults, backend), master);
    integer(master, "id", 0, std::numeric_limits<int>::max(), where);
    integer(master, "duration", 0, std::numeric_limits<int>::max(), where);
    // App 的控制周期保存在有符号整型配置中，拒绝不可表示的周期。
    integer(master, "cycle", ecm ? 1 : 1000, ecm ? 2147483 : std::numeric_limits<int>::max(), where);
    choice(master, "verbose", {"silent", "errors", "warnings", "diagnostic", "info", "verbose", "cyclic"}, where);
    choice(master, "state", {"init", "preop", "safeop", "op"}, where);
    if (ecm && !master["cpuidx"]) master["cpuidx"] = master["id"].as<int>();
    if (ecm || !master["cpuidx"].IsNull())
        integer(master, "cpuidx", 0, ecm ? 31 : std::numeric_limits<int>::max(), where);
    if (ecm) {
        const auto eni = string(master, "eni", where);
        require(fs::is_regular_file(eni), where + ".eni file not found: " + eni);
        choice(master, "perf", {"off", "timing", "histogram"}, where);
        choice(master, "dcmmode",
               {"off", "bus_shift", "master_shift", "link_layer_ref_clock", "master_ref_clock", "dcx"}, where);
        choice(master, "mode", {"interrupt", "polling"}, where);
        choice(master, "link", {"I8254x", "I8255x", "IntelGbe", "IntelGbeRec", "RTL8169"}, where);
        if (master["link"].as<std::string>() == "RTL8169")
            require(master["mode"].as<std::string>() == "polling" &&
                        master["dcmmode"].as<std::string>() != "link_layer_ref_clock",
                    where + ": unsupported RTL8169 mode");
        master["ctloff"].as<bool>();
        integer(master, "instance", 1, std::numeric_limits<int>::max(), where);
        integer(master, "maxbusslaves", 1, std::numeric_limits<int>::max(), where);
        for (auto key : {"sp", "mbxsrv"})
            if (!master[key].IsNull()) integer(master, key, 1, 65535, where);
        string(master, "log", where, true);
        string(master, "license", where, true);
    } else {
        choice(master, "dc", {"on", "off"}, where);
        if (master["dc"].as<std::string>() == "on") integer(master, "cycle", 1000, 4294967, where);
        for (auto key : {"preop_timeout_ms", "state_timeout_ms", "pdo_config_attempts"})
            integer(master, key, 1, UINT32_MAX, where);
        require(master["slaves"].IsSequence() && master["slaves"].size() > 0,
                where + ".slaves must be a nonempty list");
        require(master["slaves"].size() <= MAX_SLAVE_NUM, where + ".slaves exceeds App IPC capacity");
        size_t references = 0;
        for (size_t i = 0; i < master["slaves"].size(); ++i) {
            auto slave = master["slaves"][i];
            const auto at = where + ".slaves[" + std::to_string(i) + "]";
            keys(slave, {"id", "name", "dc_ref", "init_commands", "rx_pdos", "tx_pdos"}, at);
            require(integer(slave, "id", 0, 65535, at) == static_cast<int64_t>(i),
                    at + ".id must equal physical position");
            require(string(slave, "name", at).size() < MAX_SLAVE_NAME_LEN, at + ".name exceeds App IPC capacity");
            if (!slave["dc_ref"]) slave["dc_ref"] = false;
            references += slave["dc_ref"].as<bool>();
            if (!slave["init_commands"]) slave["init_commands"] = Node(YAML::NodeType::Sequence);
            require(slave["init_commands"].IsSequence(), at + ".init_commands must be a list");
            for (const auto& command : slave["init_commands"]) {
                keys(command, {"name", "index", "sub_index", "type", "value"}, at + ".init_commands[]");
                integer(command, "index", 0, 65535, at);
                integer(command, "sub_index", 0, 255, at);
                choice(command, "type", {"int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64"}, at);
                const auto data_type = command["type"].as<std::string>();
                const bool unsigned_type = data_type.front() == 'u';
                const auto bits = std::stoi(data_type.substr(unsigned_type ? 4 : 3));
                if (unsigned_type) {
                    const auto text = command["value"].as<std::string>();
                    require(!text.empty() && text.front() != '-', at + ": unsigned init value must be nonnegative");
                    const auto value = command["value"].as<uint64_t>();
                    require(bits == 64 || value < (uint64_t{1} << bits), at + ": init value overflow");
                } else if (bits == 64)
                    command["value"].as<int64_t>();
                else
                    integer(command, "value", -(int64_t{1} << (bits - 1)), (int64_t{1} << (bits - 1)) - 1, at);
            }
            for (auto direction : {"rx_pdos", "tx_pdos"}) {
                require(slave[direction].IsSequence() && slave[direction].size() > 0,
                        at + "." + direction + " must be a nonempty list");
                std::set<std::string> names;
                for (const auto& pdo : slave[direction]) {
                    keys(pdo, {"index", "entries"}, at + "." + direction);
                    integer(pdo, "index", 0, 65535, at);
                    require(pdo["entries"].IsSequence(), at + ".entries must be an explicit list");
                    for (const auto& entry : pdo["entries"]) {
                        keys(entry, {"name", "index", "sub_index", "bit_length"}, at + ".entries[]");
                        const auto name = string(entry, "name", at);
                        require(name.size() < MAX_PD_NAME_LEN, at + ": PDO name exceeds App IPC capacity");
                        require(names.insert(name).second, at + ": duplicate PDO name");
                        integer(entry, "index", 0, 65535, at);
                        integer(entry, "sub_index", 0, 255, at);
                        integer(entry, "bit_length", 1, 255, at);
                    }
                }
                const size_t capacity = std::string(direction) == "rx_pdos" ? MAX_PDOUTPUT_NUM : MAX_PDINPUT_NUM;
                require(names.size() <= capacity, at + ": PDO entries exceed App IPC capacity");
            }
        }
        require(references <= 1 && (master["dc"].as<std::string>() == "off" || references == 1),
                where + ": invalid DC reference count");
    }
    return master;
}

Node hardwareConfig(const Node& list, const std::string& where) {
    require(list.IsSequence() && list.size() > 0, where + " must be a nonempty list");
    Node result(YAML::NodeType::Sequence);
    std::set<std::pair<std::string, int>> ids;
    std::set<std::string> joints;
    for (size_t i = 0; i < list.size(); ++i) {
        auto node = YAML::Clone(list[i]);
        const auto at = where + "[" + std::to_string(i) + "]";
        choice(node, "type", {"driver", "ft_sensor", "io"}, at);
        const auto type = node["type"].as<std::string>();
        const auto id = static_cast<int>(integer(node, "id", 0, 65535, at));
        require(ids.insert({type, id}).second, at + ": duplicate hardware id");
        Node defaults(YAML::NodeType::Map);
        if (type == "driver") {
            keys(node, {"id", "type", "joint_name", "torque_source", "limit", "transform", "inputs", "outputs"}, at);
            require(joints.insert(string(node, "joint_name", at)).second, at + ": duplicate joint_name");
            defaults = YAML::Load(R"(
torque_source: load_torque
limit: {lower: -2.71, upper: 2.71, vel: 3.0, acc: 10.0, jerk: 150.0, effort: 30.0}
transform: {ratio: 1.0, offset_pos_cnt: 0, cnt_per_unit: 1.0, torque_per_unit: 1.0, user_unit_name: rad}
inputs: {status_word: '', position_actual_value: '', velocity_actual_value: '', torque_actual_value: '', load_torque_value: '', secondary_position_value: '', digital_inputs: '', digital_outputs: ''}
outputs: {control_word: '', mode_of_operation: '', target_position: '', target_velocity: '', target_torque: '', digital_outputs: ''}
)");
        } else if (type == "ft_sensor") {
            keys(node, {"id", "type", "inputs", "offset"}, at);
            defaults = YAML::Load(
                "inputs: {fx: '', fy: '', fz: '', tx: '', ty: '', tz: ''}\noffset: {fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, "
                "tz: 0}\n");
        } else {
            keys(node,
                 {"id", "type", "digital_in_channels", "digital_out_channels", "analog_in_channels",
                  "analog_out_channels", "inputs", "outputs"},
                 at);
            defaults = YAML::Load(
                "digital_in_channels: 0\ndigital_out_channels: 0\nanalog_in_channels: 0\nanalog_out_channels: "
                "0\ninputs: {digital_inputs: '', digital_outputs: '', analog_inputs: ''}\noutputs: {digital_outputs: "
                "'', analog_outputs: ''}\n");
        }
        // 字段表直接取现有消费者的缺省结构，不接受未实现的 PDO 字段。
        for (const auto& group : defaults) {
            const auto name = group.first.as<std::string>();
            if (group.second.IsMap() && node[name]) {
                require(node[name].IsMap(), at + "." + name + " must be a mapping");
                for (const auto& value : node[name])
                    require(group.second[value.first.as<std::string>()].IsDefined(),
                            at + "." + name + ": unknown field " + value.first.as<std::string>());
            }
        }
        node = merge(defaults, node);
        for (auto direction : {"inputs", "outputs"})
            if (node[direction]) {
                for (const auto& entry : node[direction])
                    require(entry.second.IsScalar(), at + "." + direction + " PDO names must be strings");
            }
        if (type == "driver") {
            choice(node, "torque_source", {"load_torque", "secondary_position"}, at);
            const auto limit = node["limit"], transform = node["transform"];
            require(number(limit, "lower", at) <= number(limit, "upper", at), at + ": lower exceeds upper");
            for (auto key : {"vel", "acc", "jerk", "effort"})
                require(number(limit, key, at) >= 0, at + ".limit." + key + " must be nonnegative");
            for (auto key : {"ratio", "cnt_per_unit", "torque_per_unit"})
                require(number(transform, key, at) != 0, at + ".transform." + key + " must not be zero");
            integer(transform, "offset_pos_cnt", INT32_MIN, INT32_MAX, at);
            string(transform, "user_unit_name", at);
        } else if (type == "ft_sensor") {
            for (const auto& value : node["offset"])
                require(std::isfinite(value.second.as<double>()), at + ".offset must be finite");
        } else
            for (auto key :
                 {"digital_in_channels", "digital_out_channels", "analog_in_channels", "analog_out_channels"})
                integer(node, key, 0, std::numeric_limits<int>::max(), at);
        result.push_back(node);
    }
    return result;
}
void calibrationValues(const Node& node, const std::string& kind, const std::string& where) {
    if (kind == "payload") {
        keys(node, {"mass", "com"}, where);
        require(number(node, "mass", where) >= 0, where + ".mass must be nonnegative");
        require(node["com"].IsSequence() && node["com"].size() == 3, where + ".com must have 3 elements");
        for (const auto& value : node["com"]) require(std::isfinite(value.as<double>()), where + ".com must be finite");
    } else if (kind == "tool_tcp") {
        poseKeys(node, where);
        ToolPackageFiles::pose(node);
    } else {
        require(node.IsMap(), where + " must be a joint mapping");
        for (const auto& joint : node) {
            keys(joint.second, {"offset_pos_cnt"}, where + "." + joint.first.as<std::string>());
            integer(joint.second, "offset_pos_cnt", INT32_MIN, INT32_MAX, where);
        }
    }
}
void calibrationContext(const Node& context, const Node& device, const Node& robot,
                        const std::optional<ToolPackage>& tool, const std::string& kind, const std::string& where) {
    keys(context, {"device_id", "robot", "tool", "tool_install"}, where + ".context");
    keys(context["robot"], {"name", "package_version"}, where + ".context.robot");
    require(context["device_id"].as<std::string>() == device["id"].as<std::string>(),
            where + ": calibration device mismatch");
    for (auto key : {"name", "package_version"})
        require(context["robot"][key].as<std::string>() == robot[key].as<std::string>(),
                where + ": calibration robot mismatch");
    if (kind == "joint_zero") {
        require(!context["tool"] && !context["tool_install"], where + ": joint_zero must not bind a tool");
        return;
    }
    require(tool.has_value(), where + ": tool calibration requires a selected tool");
    keys(context["tool"], {"name", "package_version", "digest"}, where + ".context.tool");
    require(context["tool"]["name"].as<std::string>() == tool->name &&
                context["tool"]["package_version"].as<std::string>() == tool->version,
            where + ": calibration tool mismatch");
    if (context["tool"]["digest"])
        require(context["tool"]["digest"].as<std::string>() == tool->digest,
                where + ": calibration tool digest mismatch");
    poseKeys(context["tool_install"], where + ".context.tool_install", true);
    require(context["tool_install"]["parent_link"].as<std::string>() ==
                    device["tool_install"]["parent_link"].as<std::string>() &&
                KDL::Equal(ToolPackageFiles::pose(context["tool_install"]),
                           ToolPackageFiles::pose(device["tool_install"]), 1e-9),
            where + ": calibration installation mismatch");
}
void atomicYaml(const std::string& filename, const Node& value, mode_t mode) {
    fs::create_directories(fs::path(filename).parent_path());
    auto temporary = filename + ".tmp.XXXXXX";
    const int fd = mkstemp(temporary.data());
    require(fd >= 0, "Cannot create output: " + filename);
    FILE* file = fdopen(fd, "w");
    if (!file) {
        close(fd);
        std::remove(temporary.c_str());
        throw std::runtime_error("Cannot open output: " + filename);
    }
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(17);
    emitter << value;
    bool ok =
        emitter.good() && fchmod(fd, mode) == 0 && fwrite(emitter.c_str(), 1, emitter.size(), file) == emitter.size();
    ok = fflush(file) == 0 && fsync(fd) == 0 && ok;
    ok = fclose(file) == 0 && ok;
    if (ok && std::rename(temporary.c_str(), filename.c_str()) == 0) return;
    std::remove(temporary.c_str());
    throw std::runtime_error("Cannot atomically write output: " + filename);
}
void protectSources(const std::string& output, const std::vector<std::string>& sources) {
    const auto target = fs::weakly_canonical(output);
    for (const auto& source : sources) {
        require(target != fs::weakly_canonical(source), "Output must not overwrite a configuration source: " + output);
    }
}
void protectInputs(const std::string& output, const AppConfig& config) {
    protectSources(output, config.source_files);
    protectSources(output, {config.robot.urdf});
    const auto eni = config.effective["robot"]["master"]["eni"];
    if (eni.IsDefined()) protectSources(output, {eni.as<std::string>()});
    if (config.robot.tool) {
        const auto relative =
            fs::weakly_canonical(output).lexically_relative(fs::weakly_canonical(config.robot.tool->directory));
        require(relative.empty() || *relative.begin() == "..", "Output must not modify the selected tool package");
    }
}
void overrideFields(const Node& node, const std::string& prefix, Node fields) {
    if (node.IsMap()) {
        for (const auto& item : node) {
            overrideFields(item.second,
                           prefix.empty() ? item.first.as<std::string>() : prefix + "." + item.first.as<std::string>(),
                           fields);
        }
    } else
        fields.push_back(prefix);
}
}  // namespace

AppConfig AppConfig::load(const std::string& filename) {
    AppConfig config;
    const auto entry = fs::absolute(filename).lexically_normal().string();
    config.entry_file = entry;
    try {
        auto app = readFile(entry, config.source_files);
        if (!app["schema_version"] && !app["robot_config"] && !app["device_config"]) return loadLegacy(filename);
        schema(app, entry);
        softwareKeys(app, entry, true);
        Node user(YAML::NodeType::Map);
        if (app["user_config"] && !app["user_config"].IsNull()) {
            path(app, "user_config", entry);
            const auto user_file = app["user_config"].as<std::string>();
            user = readFile(user_file, config.source_files);
            keys(user, {"software", "robot", "tool", "device"}, user_file);
            if (user["software"]) {
                softwareKeys(user["software"], user_file + ".software", false);
                softwarePaths(user["software"], user_file);
            }
            if (user["robot"]) {
                robotKeys(user["robot"], user_file + ".robot", false);
                robotPaths(user["robot"], user_file);
            }
            if (user["device"]) {
                deviceKeys(user["device"], user_file + ".device", false);
                devicePaths(user["device"], user_file);
            }
            if (user["tool"]) {
                keys(user["tool"], {"tcp"}, user_file + ".tool");
                if (user["tool"]["tcp"]) poseKeys(user["tool"]["tcp"], user_file + ".tool.tcp");
            }
        }
        auto defaults = YAML::Load(R"(
http_host: 0.0.0.0
http_port: 8080
http_control_ttl_seconds: 300
scripts: /var/lib/rocos/scripts
web_root: ../web
tool_library: /var/lib/rocos/tools
logging: {directory: /var/log/rocos, file_level: info, console_level: info, max_file_size_mib: 50, max_files: 20, flush_interval_ms: 1000}
diagnostics: {host: 0.0.0.0, port: 8082, max_sse_clients: 4, max_downloads: 1}
)");
        app.remove("schema_version");
        app.remove("user_config");
        auto software = merge(defaults, app);
        softwarePaths(software, entry);
        software = merge(software, child(user, "software"));
        const auto robot_file = string(software, "robot_config", "software");
        auto robot = readFile(robot_file, config.source_files);
        schema(robot, robot_file);
        robotKeys(robot, robot_file, true);
        robotPaths(robot, robot_file);
        robot = merge(robot, child(user, "robot"));
        const auto robot_name = string(robot, "name", robot_file);
        string(robot, "package_version", robot_file);
        require(FrameRegistry::validName(robot_name), robot_file + ": invalid robot name");
        for (auto key : {"urdf", "base_link", "tip_link", "hardware_config", "master_config"})
            string(robot, key, robot_file);
        integer(robot, "ecat_id", 0, std::numeric_limits<int>::max(), robot_file);
        choice(robot, "master_type", {"ecm", "igh"}, robot_file);
        const auto hardware_file = robot["hardware_config"].as<std::string>();
        const auto hardware = readFile(hardware_file, config.source_files);
        keys(hardware, {"hardware"}, hardware_file);
        // 数组直接替换，覆盖时也先验证最终选择的基础文件结构。
        auto hw = hardwareConfig(robot["hardware"] ? robot["hardware"] : hardware["hardware"],
                                 hardware_file + ": robot.hardware");
        const auto master_file = robot["master_config"].as<std::string>();
        auto master_document = readFile(master_file, config.source_files);
        const auto master_type = robot["master_type"].as<std::string>();
        if (master_type == "ecm")
            keys(master_document, {"master"}, master_file);
        else {
            keys(master_document, {"version", "master", "slaves"}, master_file);
            require(master_document["version"].as<int>() == 2,
                    master_file + ": migrate IgH version 1 to version 2 first");
        }
        Node master = master_document["master"] ? YAML::Clone(master_document["master"]) : Node(YAML::NodeType::Map);
        require(master.IsMap() && (master_type != "ecm" || master_document["master"]),
                master_file + ": master mapping required");
        path(master, "eni", master_file);
        path(master, "log", master_file, false, true);
        if (master_type == "igh") master["slaves"] = master_document["slaves"];
        master = masterConfig(merge(master, child(robot, "master")), master_type, master_file + ": robot.master");
        require(master["id"].as<int>() == robot["ecat_id"].as<int>(), "robot.ecat_id must equal robot.master.id");
        robot["master"] = master;

        const auto device_file = string(software, "device_config", "software");
        auto device = readFile(device_file, config.source_files);
        schema(device, device_file);
        deviceKeys(device, device_file, true);
        devicePaths(device, device_file);
        device = merge(YAML::Load("tool_config: null\ntool_install: null\ncalibration_files: {joint_zero: null, "
                                  "kinematics: null, dynamics: null, payload: null, tool_tcp: null}\n"),
                       device);
        device = merge(device, child(user, "device"));
        config.robot.id = string(device, "id", device_file);
        require(FrameRegistry::validName(config.robot.id) && config.robot.id != "." && config.robot.id != "..",
                device_file + ": invalid device id");
        for (auto key : {"name", "package_version"})
            require(device["expected_robot"][key].as<std::string>() == robot[key].as<std::string>(),
                    device_file + ": expected_robot does not match selected robot");
        Node tool(YAML::NodeType::Null);
        if (!device["tool_config"].IsNull()) {
            const auto tool_file = string(device, "tool_config", device_file);
            tool = readFile(tool_file, config.source_files);
            schema(tool, tool_file);
            keys(tool, {"schema_version", "name", "package_version", "model", "tcp", "mount"}, tool_file);
            if (tool["model"]) keys(tool["model"], {"urdf"}, tool_file + ".model");
            if (tool["mount"]) poseKeys(tool["mount"], tool_file + ".mount");
            poseKeys(tool["tcp"], tool_file + ".tcp");
            require(fs::path(tool_file).filename() == "tool.yaml", tool_file + ": package manifest must be tool.yaml");
            config.robot.tool = ToolPackageFiles::read(fs::path(tool_file).parent_path().string());
            path(tool["model"], "urdf", tool_file);
            tool = merge(tool, child(user, "tool"));
            require(device["tool_install"].IsMap(), device_file + ": selected tool requires tool_install");
            require(string(device["tool_install"], "parent_link", device_file) == robot["tip_link"].as<std::string>(),
                    device_file + ": current tool consumer requires tool_install.parent_link = robot.tip_link");
            config.robot.tool->mount = ToolPackageFiles::pose(device["tool_install"]);
            config.robot.tool->tcp = ToolPackageFiles::pose(tool["tcp"]);
        } else {
            require(device["tool_install"].IsNull() && device["calibration_files"]["payload"].IsNull() &&
                        device["calibration_files"]["tool_tcp"].IsNull(),
                    device_file + ": no-tool installation must not reference tool calibration");
            require(!user["tool"] || user["tool"].size() == 0, "user.tool requires a selected tool");
            // 明确无工具对应零附加载荷，不伪造一份辨识文件。
            config.robot.initial_payload = std::make_pair(0.0, Vector::Zero());
        }
        Node calibration(YAML::NodeType::Map), contexts(YAML::NodeType::Map);
        const auto overrides = child(device, "calibration");
        for (auto kind : {"joint_zero", "payload", "tool_tcp"}) {
            const auto file = device["calibration_files"][kind];
            if (file.IsNull()) {
                require(!child(overrides, kind).IsDefined(),
                        std::string("device.calibration.") + kind + " requires a referenced result file");
                continue;
            }
            const auto result_file = file.as<std::string>();
            auto record = readFile(result_file, config.source_files);
            schema(record, result_file);
            const std::string name(kind);
            if (name == "payload")
                keys(record, {"schema_version", "context", "mass", "com"}, result_file);
            else if (name == "tool_tcp")
                keys(record, {"schema_version", "context", "tcp"}, result_file);
            else
                keys(record, {"schema_version", "context", "joints"}, result_file);
            calibrationContext(record["context"], device, robot, config.robot.tool, kind, result_file);
            contexts[kind] = record["context"];
            Node parameters;
            if (name == "payload") {
                parameters["mass"] = record["mass"];
                parameters["com"] = record["com"];
                config.robot.payload_metadata["schema_version"] = 1;
                config.robot.payload_metadata["context"] = record["context"];
            } else
                parameters = YAML::Clone(record[name == "tool_tcp" ? "tcp" : "joints"]);
            calibrationValues(parameters, name, result_file);
            parameters = merge(parameters, child(overrides, kind));
            calibrationValues(parameters, name, result_file + ": final device.calibration." + kind);
            calibration[kind] = parameters;
            if (name == "payload")
                config.robot.initial_payload =
                    std::make_pair(parameters["mass"].as<double>(),
                                   Vector(parameters["com"][0].as<double>(), parameters["com"][1].as<double>(),
                                          parameters["com"][2].as<double>()));
            if (name == "tool_tcp") config.robot.tool->tcp = ToolPackageFiles::pose(parameters);
        }
        for (auto kind : {"kinematics", "dynamics"}) {
            const auto file = device["calibration_files"][kind];
            if (!file.IsNull()) {
                require(fs::is_regular_file(file.as<std::string>()), std::string(kind) + " resource not found");
                throw std::invalid_argument(std::string("Unsupported ") + kind +
                                            " NPZ consumer; no parameters have been applied");
            }
        }
        if (calibration["joint_zero"])
            for (const auto& joint : calibration["joint_zero"]) {
                bool found = false;
                for (auto entry : hw)
                    if (entry["type"].as<std::string>() == "driver" &&
                        entry["joint_name"].as<std::string>() == joint.first.as<std::string>()) {
                        entry["transform"]["offset_pos_cnt"] = joint.second["offset_pos_cnt"].as<int>();
                        found = true;
                    }
                require(found, "joint_zero references unknown hardware joint: " + joint.first.as<std::string>());
            }
        robot["hardware"] = hw;
        device["calibration"] = calibration;
        config.effective["software"] = software;
        config.effective["robot"] = robot;
        config.effective["device"] = device;
        config.effective["tool"] = tool;
        config.effective["snapshot"]["calibration_contexts"] = contexts;
        config.effective["snapshot"]["user_override_fields"] = Node(YAML::NodeType::Sequence);
        overrideFields(user, "", config.effective["snapshot"]["user_override_fields"]);
        config.effective["snapshot"]["master_parameters_applied"] = false;
        if (master_type == "igh")
            config.effective["snapshot"]["source_formats"][master_file] = 2;
        else
            config.effective["snapshot"]["internal_defaults"]["ecm"] = YAML::Load("auxclk: 0\nflash: disabled\n");

        config.robot.composed_configuration = true;
        config.robot.hardware_config = hardware_file;
        Node hardware_root;
        hardware_root["hardware"] = hw;
        config.robot.hardware = Hardware::LoadConfigFromNode(hardware_root);
        config.robot.ecat_id = robot["ecat_id"].as<int>();
        config.robot.urdf = robot["urdf"].as<std::string>();
        config.robot.base_link = robot["base_link"].as<std::string>();
        config.robot.tip_link = robot["tip_link"].as<std::string>();
        config.robot.joint_binding.clear();
        config.robot.master_cycle_us = master["cycle"].as<int>();
        config.robot.master_state = master["state"].as<std::string>();
        config.robot.frames = (fs::path(device_file).parent_path() / "frames.yaml").string();
        config.robot.payload = device["calibration_files"]["payload"].IsNull()
                                   ? (fs::path(device_file).parent_path() / "identification/payload.yaml").string()
                                   : device["calibration_files"]["payload"].as<std::string>();
        const auto user_device = child(user, "device");
        config.robot.payload_overridden = child(child(user_device, "calibration"), "payload").IsDefined() ||
                                          child(child(user_device, "calibration_files"), "payload").IsDefined();
        config.robot.tool_library = software["tool_library"].as<std::string>();
        config.http_host = string(software, "http_host", "software");
        config.http_port = software["http_port"].as<int>();
        config.http_control_ttl_seconds = software["http_control_ttl_seconds"].as<int>();
        config.scripts = string(software, "scripts", "software");
        config.web_root = string(software, "web_root", "software");
        const auto logging = software["logging"], diagnostics = software["diagnostics"];
        config.logging.directory = string(logging, "directory", "software.logging");
        config.logging.file_level = parseLogLevel(logging["file_level"].as<std::string>());
        config.logging.console_level = parseLogLevel(logging["console_level"].as<std::string>());
        config.logging.max_file_size = integer(logging, "max_file_size_mib", 1, 1024, "software.logging") * 1024 * 1024;
        config.logging.max_files =
            integer(logging, "max_files", 1, std::numeric_limits<int>::max(), "software.logging");
        config.logging.flush_interval_ms = logging["flush_interval_ms"].as<int>();
        config.diagnostics.host = string(diagnostics, "host", "software.diagnostics");
        config.diagnostics.port = diagnostics["port"].as<int>();
        config.diagnostics.max_sse_clients = integer(diagnostics, "max_sse_clients", 1, 16, "software.diagnostics");
        config.diagnostics.max_downloads = integer(diagnostics, "max_downloads", 1, 16, "software.diagnostics");
        config.validate();
        return config;
    } catch (const std::exception& e) {
        throw std::invalid_argument(entry + ": " + e.what());
    }
}

void AppConfig::validateResources() const {
    for (const auto& file : {robot.hardware_config, robot.urdf})
        require(fs::is_regular_file(file), "Configuration resource not found: " + file);
    require(fs::is_directory(scripts), "Scripts directory not found: " + scripts);
    require(fs::is_directory(web_root), "Web resource directory not found: " + web_root);
    if (!robot.composed_configuration) return;
    // URDF 由模型消费者校验，合并器与快照只保存路径。
    Model model(robot.urdf, robot.base_link, robot.tip_link);
    JointBinding binding;
    std::vector<int32_t> ids;
    for (const auto& drive : robot.hardware->drives) ids.push_back(drive.id);
    require(binding.Configure(model.GetJointNames(), ids) == Result::NoError,
            "Hardware/model binding configuration mismatch");
    for (const auto& drive : robot.hardware->drives)
        require(binding.Bind(drive.joint_name, drive.id) == Result::NoError,
                "Hardware joint_name not in model chain: " + drive.joint_name);
    require(binding.Validate() == Result::NoError,
            "Hardware joint_name must cover every movable model joint exactly once");
}

void AppConfig::writeSnapshot(const std::string& filename) const {
    require(robot.composed_configuration, "Effective snapshots require schema_version: 1 configuration");
    protectInputs(filename, *this);
    auto result = YAML::Clone(effective);
    result["software"]["http_host"] = http_host;
    result["software"]["http_port"] = http_port;
    result["software"]["http_control_ttl_seconds"] = http_control_ttl_seconds;
    auto meta = result["snapshot"];
    meta["format_version"] = 1;
    meta["app_version"] = ROCOS_APP_VERSION;
    meta["device_id"] = robot.id;
    meta["generated_at"] = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    meta["source_files"] = source_files;
    if (result["robot"]["master"]["license"]) result["robot"]["master"]["license"] = "<redacted>";
    atomicYaml(fs::absolute(filename).lexically_normal().string(), result, 0444);
}

void AppConfig::exportConfiguration(const std::string& directory) const {
    require(robot.composed_configuration, "Configuration export requires schema_version: 1 configuration");
    const auto output = fs::absolute(directory).lexically_normal();
    Node root;
    auto master = YAML::Clone(effective["robot"]["master"]);
    const auto type = effective["robot"]["master_type"].as<std::string>();
    for (const auto& name : {type + ".yaml", std::string("ecm-legacy.yaml"), std::string("effective.yaml")}) {
        protectInputs((output / name).string(), *this);
    }
    if (type == "igh") {
        root["version"] = 2;
        root["slaves"] = master["slaves"];
        master.remove("slaves");
    }
    root["master"] = master;
    atomicYaml((output / (type + ".yaml")).string(), root, 0600);
    if (type == "ecm") {
        // 旧格式省略禁用的可选端口，auxclk/flash 使用后端内部默认值。
        for (auto key : {"sp", "mbxsrv"})
            if (master[key].IsNull()) master.remove(key);
        for (const auto& names : std::vector<std::pair<std::string, std::vector<std::string>>>{
                 {"verbose", {"silent", "errors", "warnings", "diagnostic", "info", "verbose", "cyclic"}},
                 {"perf", {"off", "timing", "histogram"}},
                 {"mode", {"interrupt", "polling"}},
                 {"dcmmode",
                  {"off", "bus_shift", "master_shift", "link_layer_ref_clock", "master_ref_clock", "dcx"}}}) {
            const auto value = master[names.first].as<std::string>();
            master[names.first] =
                static_cast<int>(std::find(names.second.begin(), names.second.end(), value) - names.second.begin());
        }
        atomicYaml((output / "ecm-legacy.yaml").string(), master, 0600);
    }
    writeSnapshot((output / "effective.yaml").string());
}
}  // namespace rocos
