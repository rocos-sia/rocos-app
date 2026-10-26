#include "frame_registry.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <sys/stat.h>
#include <unistd.h>
#include <filesystem>
#include <stdexcept>
#include <algorithm>

namespace rocos {
namespace {
std::string calibrationYaml(const YAML::Node& value) {
    if (!value || value.IsNull()) return {};
    if (!value.IsMap()) throw std::invalid_argument("Calibration record must be a mapping");
    YAML::Emitter out; out.SetDoublePrecision(17); out<<value;
    if (!out.good()) throw std::invalid_argument("Cannot serialize calibration record");
    return out.c_str();
}
}
FrameRegistry::FrameRegistry(std::string path, std::string library, std::string robot_id)
    : path_(std::move(path)),
      library_(library.empty() ? (std::filesystem::absolute(path_).parent_path() / "tools").string() : std::move(library)),
      robot_id_(std::move(robot_id)), state_(std::make_shared<const FrameRegistryState>()) {
    if (!validName(robot_id_)) throw std::invalid_argument("Invalid robot_id");
    auto initial=std::make_shared<FrameRegistryState>(); initial->robot_id=robot_id_; state_=initial;
}

bool FrameRegistry::validName(const std::string& name) {
    if (name.empty() || name.size() > 64) return false;
    for (const unsigned char ch : name)
        if (!((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
              (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.')) return false;
    return true;
}
bool FrameRegistry::validFrame(const KDL::Frame& frame) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(frame.p[i])) return false;
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(frame.M(i, j))) return false;
            double dot = 0;
            for (int k = 0; k < 3; ++k) dot += frame.M(k, i) * frame.M(k, j);
            if (std::abs(dot - (i == j ? 1 : 0)) > 1e-6) return false;
        }
    }
    return KDL::dot(frame.M.UnitX() * frame.M.UnitY(), frame.M.UnitZ()) > 1 - 1e-6;
}
std::shared_ptr<const FrameRegistryState> FrameRegistry::snapshot() const {
    return std::atomic_load(&state_);
}
Result FrameRegistry::resolveTcp(const FrameRegistryState& state, const KDL::Frame& flange,
                                 KDL::Frame& tcp, const std::string& tool_name) {
    if (!validFrame(flange)) return Result::FkCalcFail;
    const auto& name = tool_name.empty() ? state.active_tool : tool_name;
    if (name.empty()) { tcp = flange; return Result::NoError; }
    if (!validName(name)) return Result::IllegalParameter;
    const auto found = state.tools.find(name);
    if (found == state.tools.end()) return Result::ResourceUnavailable;
    if (!validFrame(found->second)) return Result::IllegalParameter;
    const auto result = flange * found->second;
    if (!validFrame(result)) return Result::FkCalcFail;
    tcp = result;
    return Result::NoError;
}
std::vector<std::string> FrameRegistry::names(bool tool) const {
    auto state = snapshot();
    const auto& frames = tool ? state->tools : state->objects;
    std::vector<std::string> names;
    for (const auto& item : frames) names.push_back(item.first);
    return names;
}
Result FrameRegistry::get(const std::string& name, bool tool, KDL::Frame& frame) const {
    auto state = snapshot();
    const auto& frames = tool ? state->tools : state->objects;
    auto found = frames.find(name);
    if (found == frames.end()) return Result::ResourceUnavailable;
    frame = found->second;
    return Result::NoError;
}
Result FrameRegistry::publish(std::shared_ptr<FrameRegistryState> next, bool persist) {
    if (persist) {
        const auto result = write(*next, path_);
        if (result != Result::NoError) return result;
    }
    std::atomic_store(&state_, std::shared_ptr<const FrameRegistryState>(std::move(next)));
    return Result::NoError;
}
Result FrameRegistry::set(const std::string& name, const KDL::Frame& frame, bool tool,
                          bool add_only, bool persist, const std::string& previous_name) {
    if (!validName(name) || !validFrame(frame)) return Result::IllegalParameter;
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    // 包工具必须通过分量接口修改，旧接口不能把合成偏置写回局部 TCP。
    if (tool && (next->packages.count(name) || next->packages.count(previous_name))) return Result::CallingConflictError;
    auto& frames = tool ? next->tools : next->objects;
    if (add_only && frames.count(name)) return Result::CallingConflictError;
    if (!previous_name.empty()) {
        if (!validName(previous_name)) return Result::IllegalParameter;
        if (!frames.count(previous_name)) return Result::ResourceUnavailable;
        if (previous_name != name) {
            if (frames.count(name)) return Result::CallingConflictError;
            frames.erase(previous_name);
            auto& active = tool ? next->active_tool : next->active_object;
            if (active == previous_name) active = name;
        }
    }
    frames[name] = frame;
    return publish(std::move(next), persist);
}
Result FrameRegistry::remove(const std::string& name, bool tool, bool persist) {
    if (!validName(name)) return Result::IllegalParameter;
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    auto& frames = tool ? next->tools : next->objects;
    auto& active = tool ? next->active_tool : next->active_object;
    if (!frames.erase(name)) return Result::ResourceUnavailable;
    if (tool) next->packages.erase(name);
    if (active == name) active.clear();
    return publish(std::move(next), persist);
}
Result FrameRegistry::activate(const std::string& name, bool tool, bool persist) {
    if (!name.empty() && !validName(name)) return Result::IllegalParameter;
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    const auto& frames = tool ? next->tools : next->objects;
    if (!name.empty() && !frames.count(name)) return Result::ResourceUnavailable;
    (tool ? next->active_tool : next->active_object) = name;
    return publish(std::move(next), persist);
}
Result FrameRegistry::write(const FrameRegistryState& state, const std::string& path) {
    if (path.empty()) return Result::IllegalParameter;
    std::string temporary = path + ".tmp.XXXXXX";
    int fd = -1;
    FILE* file = nullptr;
    try {
        YAML::Node root;
        root["active_tool"] = state.active_tool;
        root["active_object"] = state.active_object;
        if (!state.packages.empty()) root["robot_id"] = state.robot_id;
        for (bool tool : {true, false}) {
            const auto key = tool ? "tools" : "objects";
            root[key] = YAML::Node(YAML::NodeType::Map);
            for (const auto& item : tool ? state.tools : state.objects) {
                if (tool && state.packages.count(item.first)) continue;
                auto node = root[key][item.first];
                for (int i = 0; i < 3; ++i) node["position"].push_back(item.second.p[i]);
                double x, y, z, w;
                item.second.M.GetQuaternion(x, y, z, w);
                for (double value : {x, y, z, w}) node["orientation"].push_back(value);
            }
        }
        for (const auto& item : state.packages) {
            const auto& tool = item.second;
            auto node = root["package_tools"][item.first];
            node["digest"] = tool.digest; node["package_version"] = tool.version;
            node["mount"] = ToolPackageFiles::pose(tool.mount); node["tcp"] = ToolPackageFiles::pose(tool.tcp);
            node["revision"] = tool.revision;
            if (!tool.calibration_yaml.empty()) node["calibration"] = YAML::Load(tool.calibration_yaml);
        }
        YAML::Emitter emitter;
        emitter.SetDoublePrecision(17);
        emitter << root;
        if (!emitter.good()) return Result::ResourceUnavailable;
        fd = mkstemp(temporary.data());
        if (fd < 0) return Result::ResourceUnavailable;
        // 原文件只有完整的新快照写好后才会被原子替换。
        struct stat existing{};
        if (::stat(path.c_str(), &existing) == 0) fchmod(fd, existing.st_mode & 0777);
        else fchmod(fd, 0640);
        file = fdopen(fd, "w");
        if (!file) { ::close(fd); std::remove(temporary.c_str()); return Result::ResourceUnavailable; }
        bool ok = fwrite(emitter.c_str(), 1, emitter.size(), file) == emitter.size();
        ok = fflush(file) == 0 && ok;
        ok = fsync(fd) == 0 && ok;
        ok = fclose(file) == 0 && ok;
        file = nullptr;
        if (ok && std::rename(temporary.c_str(), path.c_str()) == 0) return Result::NoError;
    } catch (const std::exception&) {
        if (file) fclose(file);
    }
    std::remove(temporary.c_str());
    return Result::ResourceUnavailable;
}
Result FrameRegistry::save(const std::string& path) const {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    return write(*snapshot(), path.empty() ? path_ : path);
}
Result FrameRegistry::load(const std::string& path) {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    try {
        const auto root = YAML::LoadFile(path.empty() ? path_ : path);
        if (!root.IsMap()) return Result::IllegalParameter;
        auto next = std::make_shared<FrameRegistryState>();
        next->robot_id=robot_id_;
        for (bool tool : {true, false}) {
            const auto node = root[tool ? "tools" : "objects"];
            if (!node || node.IsNull()) continue;
            if (!node.IsMap()) return Result::IllegalParameter;
            auto& frames = tool ? next->tools : next->objects;
            for (const auto& item : node) {
                const auto name = item.first.as<std::string>();
                if (!validName(name)) return Result::IllegalParameter;
                const auto pos = item.second["position"], q = item.second["orientation"];
                if (!pos.IsSequence() || pos.size() != 3 || !q.IsSequence() || q.size() != 4) return Result::IllegalParameter;
                double x = q[0].as<double>(), y = q[1].as<double>(), z = q[2].as<double>(), w = q[3].as<double>();
                const double norm = std::sqrt(x*x + y*y + z*z + w*w);
                if (!std::isfinite(norm) || norm < 1e-12) return Result::IllegalParameter;
                KDL::Frame frame(KDL::Rotation::Quaternion(x/norm, y/norm, z/norm, w/norm),
                    KDL::Vector(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>()));
                if (!validFrame(frame) || frames.count(name)) return Result::IllegalParameter;
                frames.emplace(name, frame);
            }
        }
        if (root["package_tools"]) {
            if (!root["package_tools"].IsMap()) return Result::IllegalParameter;
            if (root["package_tools"].size() && root["robot_id"].as<std::string>("")!=robot_id_) return Result::IllegalParameter;
            for (const auto& item : root["package_tools"]) {
                const auto name = item.first.as<std::string>(), digest = item.second["digest"].as<std::string>();
                if (!validName(name) || name == "." || name == ".." || digest.size()!=64 || digest.find_first_not_of("0123456789abcdef")!=std::string::npos || next->tools.count(name)) return Result::IllegalParameter;
                auto tool = ToolPackageFiles::read((std::filesystem::path(library_)/".objects"/name/digest).string());
                if (tool.name!=name || tool.digest!=digest || tool.version!=item.second["package_version"].as<std::string>()) return Result::IllegalParameter;
                tool.mount=ToolPackageFiles::pose(item.second["mount"]); tool.tcp=ToolPackageFiles::pose(item.second["tcp"]);
                tool.revision=item.second["revision"].as<uint64_t>(1); tool.calibration_yaml=calibrationYaml(item.second["calibration"]);
                if (tool.revision<1 || tool.revision>=9007199254740991ULL || !validFrame(tool.mount*tool.tcp)) return Result::IllegalParameter;
                tool.revision=allocateRevision(tool.revision);
                next->tools[name]=tool.mount*tool.tcp; next->packages[name]=std::move(tool);
            }
        }
        next->active_tool = root["active_tool"].as<std::string>("");
        next->active_object = root["active_object"].as<std::string>("");
        if ((!next->active_tool.empty() && !next->tools.count(next->active_tool)) ||
            (!next->active_object.empty() && !next->objects.count(next->active_object))) return Result::IllegalParameter;
        return publish(std::move(next), false);
    } catch (const std::exception&) { return Result::ResourceUnavailable; }
}
void FrameRegistry::putPackage(FrameRegistryState& state, ToolPackage tool, const std::string& expected_digest) {
    const auto found=state.packages.find(tool.name);
    if (found!=state.packages.end()) {
        if (found->second.digest==tool.digest) return;
        if (expected_digest!=found->second.digest || tool.version==found->second.version)
            throw std::invalid_argument("Tool update requires matching expected_digest and a different package_version");
    } else if (state.tools.count(tool.name)) throw std::invalid_argument("Tool name already belongs to a plain frame");
    tool.revision=allocateRevision();
    state.tools[tool.name]=tool.mount*tool.tcp; state.packages[tool.name]=std::move(tool);
}
uint64_t FrameRegistry::allocateRevision(uint64_t minimum) {
    const auto revision=std::max(next_revision_,minimum);
    if (revision>=9007199254740990ULL) throw std::invalid_argument("Tool revision exhausted");
    next_revision_=revision+1; return revision;
}
void FrameRegistry::importPackage(const std::string& zip, const std::string& expected_digest) {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto tool=ToolPackageFiles::installZip(zip,library_);
    auto next=std::make_shared<FrameRegistryState>(*snapshot()); putPackage(*next,std::move(tool),expected_digest);
    if (publish(std::move(next),true)!=Result::NoError) throw std::runtime_error("Cannot persist imported tool; registry retained");
}
void FrameRegistry::reloadPackages(bool persist) {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next=std::make_shared<FrameRegistryState>(*snapshot());
    if (std::filesystem::exists(library_)) for (const auto& entry : std::filesystem::directory_iterator(library_)) {
        if (!entry.is_directory() || entry.path().filename().string()[0]=='.' || !std::filesystem::exists(entry.path()/"tool.yaml")) continue;
        // 启动优先恢复持久化的不可变版本；源目录变化不能偷偷升级或阻止旧装配恢复。
        if (!persist && next->packages.count(entry.path().filename().string())) continue;
        auto tool=ToolPackageFiles::freeze(entry.path().string(),library_);
        if (entry.path().filename()!=tool.name) throw std::invalid_argument("Tool folder and name differ");
        putPackage(*next,std::move(tool),"");
    }
    if (publish(std::move(next),persist)!=Result::NoError) throw std::runtime_error("Cannot persist tool library refresh");
}
void FrameRegistry::initializePackage(const ToolPackage& configured) {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto tool = ToolPackageFiles::freeze(configured.directory, library_);
    if (tool.digest != configured.digest) throw std::runtime_error("Tool package changed during startup");
    tool.mount = configured.mount;
    tool.tcp = configured.tcp;
    tool.calibration_yaml = configured.calibration_yaml;
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    putPackage(*next, std::move(tool), "");
    next->active_tool = configured.name;
    publish(std::move(next), false);
}
void FrameRegistry::updatePackage(const std::string& name, const KDL::Frame& mount, const KDL::Frame& tcp,
                                  const std::string& digest, uint64_t revision, const YAML::Node& calibration) {
    if (!validFrame(mount) || !validFrame(tcp) || !validFrame(mount*tcp)) throw std::invalid_argument("Invalid installation or TCP transform");
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next=std::make_shared<FrameRegistryState>(*snapshot()); auto found=next->packages.find(name);
    if (found==next->packages.end() || found->second.digest!=digest || found->second.revision!=revision)
        throw std::invalid_argument("Tool definition changed; reload before saving");
    auto& tool=found->second; tool.mount=mount; tool.tcp=tcp; tool.calibration_yaml=calibrationYaml(calibration); tool.revision=allocateRevision();
    next->tools[name]=mount*tcp;
    if (publish(std::move(next),true)!=Result::NoError) throw std::runtime_error("Cannot persist tool overrides; registry retained");
}
std::string FrameRegistry::exportInstallation(const std::string& name) const {
    auto state=snapshot(); auto found=state->packages.find(name);
    if (found==state->packages.end()) throw std::invalid_argument("Package tool not found");
    const auto& tool=found->second;
    YAML::Node root; root["schema_version"]=1; root["robot_id"]=robot_id_; root["name"]=name;
    root["package_version"]=tool.version; root["package_digest"]=tool.digest;
    root["mount"]=ToolPackageFiles::pose(tool.mount); root["tcp"]=ToolPackageFiles::pose(tool.tcp);
    if (!tool.calibration_yaml.empty()) root["calibration"]=YAML::Load(tool.calibration_yaml);
    YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter<<root; return emitter.c_str();
}
void FrameRegistry::importInstallation(const std::string& yaml) {
    const auto root=YAML::Load(yaml);
    if (root["schema_version"].as<int>(0)!=1 || root["robot_id"].as<std::string>("")!=robot_id_) throw std::invalid_argument("Installation belongs to a different robot_id");
    const auto name=root["name"].as<std::string>(); const auto state=snapshot(); const auto found=state->packages.find(name);
    if (found==state->packages.end() || found->second.digest!=root["package_digest"].as<std::string>() || found->second.version!=root["package_version"].as<std::string>()) throw std::invalid_argument("Installation requires the identical tool package");
    updatePackage(name,ToolPackageFiles::pose(root["mount"]),ToolPackageFiles::pose(root["tcp"]),found->second.digest,found->second.revision,root["calibration"]);
}
} // namespace rocos
