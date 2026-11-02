#include "tool_package.hpp"
#include "frame_registry.hpp"
#include <archive.h>
#include <archive_entry.h>
#include <openssl/evp.h>
#include <tinyxml2.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <set>
#include <unistd.h>
#include <fcntl.h>

namespace rocos {
namespace {
namespace fs = std::filesystem;
constexpr size_t limit = 64 * 1024 * 1024, max_files = 512;
void require(bool condition, const std::string& message) { if (!condition) throw std::invalid_argument(message); }
bool packageName(const std::string& name) { return FrameRegistry::validName(name) && name != "." && name != ".." && name[0] != '.'; }
bool version(const std::string& value) { return std::regex_match(value, std::regex("(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)(-[A-Za-z0-9.-]+)?")); }
std::string readFile(const fs::path& path) {
    require(fs::is_regular_file(path) && fs::file_size(path) <= limit, "Missing or oversized tool asset");
    std::ifstream file(path, std::ios::binary);
    require(file.good(), "Cannot read tool asset");
    return std::string(std::istreambuf_iterator<char>(file), {});
}
void writeFile(const fs::path& path, const std::string& value) {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary); file.write(value.data(), value.size()); file.close();
    require(!file.fail(), "Cannot write tool asset");
    const int fd=::open(path.c_str(),O_RDONLY);
    require(fd>=0,"Cannot flush tool asset"); const int result=::fsync(fd); ::close(fd);
    require(result==0,"Cannot flush tool asset");
}
fs::path safePath(const std::string& value) {
    fs::path path(value);
    require(!value.empty() && value.find('\\') == std::string::npos && value.find(':') == std::string::npos && value.find('\0') == std::string::npos && !path.is_absolute(), "Asset path must be relative");
    path = path.lexically_normal();
    for (const auto& part : path) require(part != ".." && part != ".", "Asset path escapes tool package");
    require(path != ".", "Invalid asset path");
    return path;
}
std::map<std::string, std::string> files(const fs::path& root) {
    require(fs::is_directory(root) && !fs::is_symlink(root), "Missing tool package directory");
    std::map<std::string, std::string> out; size_t total = 0;
    for (const auto& entry : fs::recursive_directory_iterator(root)) {
        require(!entry.is_symlink(), "Tool package symlinks are not supported");
        if (entry.is_directory()) continue;
        require(entry.is_regular_file(), "Unsupported tool asset type");
        total += entry.file_size();
        require(total <= limit && out.size() < max_files, "Tool package exceeds size or file limit");
        out.emplace(entry.path().lexically_relative(root).generic_string(), readFile(entry.path()));
    }
    return out;
}
std::string digest(const std::map<std::string, std::string>& entries) {
    auto* context = EVP_MD_CTX_new();
    if (!context) throw std::runtime_error("Cannot allocate digest context");
    EVP_DigestInit_ex(context, EVP_sha256(), nullptr);
    for (const auto& entry : entries) {
        const auto header = std::to_string(entry.first.size()) + ":" + entry.first + std::to_string(entry.second.size()) + ":";
        EVP_DigestUpdate(context, header.data(), header.size()); EVP_DigestUpdate(context, entry.second.data(), entry.second.size());
    }
    unsigned char hash[EVP_MAX_MD_SIZE]; unsigned int length = 0;
    EVP_DigestFinal_ex(context, hash, &length); EVP_MD_CTX_free(context);
    std::ostringstream result;
    for (unsigned int i = 0; i < length; ++i) result << std::hex << std::setw(2) << std::setfill('0') << int(hash[i]);
    return result.str();
}
struct TempDirectory {
    fs::path path;
    explicit TempDirectory(const fs::path& parent) {
        fs::create_directories(parent);
        auto pattern = (parent / ".import-XXXXXX").string();
        auto* made = mkdtemp(pattern.data()); require(made, "Cannot create import directory"); path = made;
    }
    ~TempDirectory() { std::error_code error; fs::remove_all(path, error); }
};
}
rocos::Frame ToolPackageFiles::pose(const YAML::Node& node) {
    require(node.IsMap(), "Expected pose mapping");
    const auto p = node["position"], q = node["orientation"];
    require(p.IsSequence() && p.size() == 3 && q.IsSequence() && q.size() == 4, "Expected position[3], orientation[4]");
    const double x=q[0].as<double>(), y=q[1].as<double>(), z=q[2].as<double>(), w=q[3].as<double>();
    const double norm = std::sqrt(x*x+y*y+z*z+w*w);
    require(std::isfinite(norm) && norm > 1e-12, "Invalid tool quaternion");
    rocos::Frame frame(rocos::Rotation::Quaternion(x/norm,y/norm,z/norm,w/norm), rocos::Vector(p[0].as<double>(),p[1].as<double>(),p[2].as<double>()));
    require(FrameRegistry::validFrame(frame), "Invalid tool transform"); return frame;
}
YAML::Node ToolPackageFiles::pose(const rocos::Frame& frame) {
    YAML::Node node;
    for (int i=0;i<3;++i) node["position"].push_back(frame.p[i]);
    double x,y,z,w; frame.M.GetQuaternion(x,y,z,w);
    for (double v : {x,y,z,w}) node["orientation"].push_back(v);
    return node;
}
std::string ToolPackageFiles::asset(const ToolPackage& tool, const std::string& value) {
    const auto relative = safePath(value);
    auto path = fs::path(tool.directory);
    require(!fs::is_symlink(path), "Tool package symlink is not supported");
    for (const auto& part : relative) { path /= part; require(!fs::is_symlink(path), "Tool asset symlink is not supported"); }
    return readFile(path);
}
ToolPackage ToolPackageFiles::read(const std::string& directory) {
    const auto entries = files(directory);
    require(entries.count("tool.yaml"), "Tool package requires tool.yaml");
    const auto root = YAML::Load(entries.at("tool.yaml"));
    require(root.IsMap() && root["schema_version"].as<int>(0) == 1, "Unsupported tool package schema");
    ToolPackage tool;
    tool.name = root["name"].as<std::string>(""); tool.version = root["package_version"].as<std::string>("");
    require(packageName(tool.name) && version(tool.version), "Invalid tool name or package version");
    tool.directory = fs::absolute(directory).lexically_normal().string(); tool.digest = digest(entries);
    tool.default_mount = root["mount"] ? pose(root["mount"]) : rocos::Frame();
    tool.default_tcp = pose(root["tcp"]); tool.mount = tool.default_mount; tool.tcp = tool.default_tcp;
    require(FrameRegistry::validFrame(tool.mount*tool.tcp), "Composed tool transform is invalid");
    if (root["model"]) {
        tool.urdf = safePath(root["model"]["urdf"].as<std::string>()).generic_string();
        auto extension=fs::path(tool.urdf).extension().string();
        std::transform(extension.begin(),extension.end(),extension.begin(),::tolower);
        require(extension==".urdf" || extension==".xml", "Tool description must use .urdf or .xml");
        const auto xml = asset(tool, tool.urdf);
        tinyxml2::XMLDocument document;
        require(document.Parse(xml.data(), xml.size()) == tinyxml2::XML_SUCCESS, "Invalid tool URDF XML");
        auto* robot = document.FirstChildElement("robot");
        require(robot && robot->FirstChildElement("link"), "Tool URDF requires a robot and link");
        std::function<void(tinyxml2::XMLElement*)> check = [&](auto* element) {
            require(std::string(element->Name())!="texture", "Tool textures are not supported; use URDF colors and STL meshes");
            if (std::string(element->Name()) == "mesh") {
                const auto* filename = element->Attribute("filename"); require(filename, "Missing mesh filename");
                std::string relative(filename);
                if (relative.rfind("package://",0) == 0) {
                    relative.erase(0,10); auto slash = relative.find('/');
                    require(slash != std::string::npos && relative.substr(0,slash) == tool.name, "Mesh package name must match tool name"); relative.erase(0,slash+1);
                } else relative = (fs::path(tool.urdf).parent_path()/relative).lexically_normal().generic_string();
                auto extension=fs::path(relative).extension().string();
                std::transform(extension.begin(),extension.end(),extension.begin(),::tolower);
                require(extension == ".stl", "Tool packages currently support STL meshes");
                asset(tool, relative);
            }
            for (auto* child=element->FirstChildElement();child;child=child->NextSiblingElement()) check(child);
        };
        check(robot);
    }
    return tool;
}
ToolPackage ToolPackageFiles::freeze(const std::string& directory, const std::string& library) {
    auto tool = read(directory);
    const auto target=fs::path(library)/".objects"/tool.name/tool.digest;
    if (!fs::exists(target)) {
        TempDirectory temporary{fs::path(library)};
        for (const auto& item : files(directory)) writeFile(temporary.path/item.first,item.second);
        fs::create_directories(target.parent_path()); fs::rename(temporary.path,target);
        const int fd=::open(target.parent_path().c_str(),O_RDONLY|O_DIRECTORY);
        require(fd>=0,"Cannot flush tool library"); const int result=::fsync(fd); ::close(fd);
        require(result==0,"Cannot flush tool library");
    }
    auto frozen=read(target.string()); require(frozen.digest == tool.digest, "Tool package digest mismatch"); return frozen;
}
ToolPackage ToolPackageFiles::installZip(const std::string& bytes, const std::string& library) {
    require(!bytes.empty() && bytes.size() <= limit, "ZIP is empty or too large");
    TempDirectory temporary{fs::path(library)};
    std::unique_ptr<archive, decltype(&archive_read_free)> reader(archive_read_new(),archive_read_free);
    archive_read_support_format_zip(reader.get());
    require(archive_read_open_memory(reader.get(),bytes.data(),bytes.size()) == ARCHIVE_OK,"Invalid ZIP archive");
    archive_entry* entry=nullptr; size_t total=0,count=0; std::string name;
    std::set<std::string> seen;
    int status;
    while ((status=archive_read_next_header(reader.get(),&entry)) == ARCHIVE_OK) {
        const auto path=safePath(archive_entry_pathname(entry));
        const auto first=path.begin()->string();
        require(packageName(first) && (name.empty() || name==first),"ZIP must contain one named tool directory"); name=first;
        require(!archive_entry_symlink(entry) && !archive_entry_hardlink(entry),"ZIP links are not supported");
        if (archive_entry_filetype(entry)==AE_IFDIR) continue;
        require(archive_entry_filetype(entry)==AE_IFREG && ++count<=max_files && seen.insert(path.generic_string()).second,"Invalid or duplicate ZIP entry");
        std::string content; char buffer[16384]; la_ssize_t size;
        while ((size=archive_read_data(reader.get(),buffer,sizeof buffer))>0) {
            total+=size; require(total<=limit,"Expanded tool package exceeds size limit"); content.append(buffer,size);
        }
        require(size==0,"Corrupt ZIP entry"); writeFile(temporary.path/path,content);
    }
    require(status==ARCHIVE_EOF,"Corrupt ZIP archive");
    auto tool=read((temporary.path/name).string()); require(tool.name==name,"Tool folder and name differ");
    return freeze(tool.directory,library);
}
std::string ToolPackageFiles::exportZip(const ToolPackage& tool,const std::string& next_version,bool calibrated) {
    auto entries=files(tool.directory);
    require(digest(entries)==tool.digest,"Tool package assets changed; reload package");
    if (calibrated) {
        require(version(next_version) && next_version!=tool.version,"Export calibrated TCP with a new package version");
        auto manifest=YAML::Load(entries.at("tool.yaml"));
        manifest["tcp"]=pose(tool.tcp); manifest["package_version"]=next_version;
        YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter<<manifest; entries["tool.yaml"]=emitter.c_str();
    }
    std::vector<char> buffer(limit+1024*1024); size_t used=0;
    std::unique_ptr<archive,decltype(&archive_write_free)> writer(archive_write_new(),archive_write_free);
    require(archive_write_set_format_zip(writer.get())==ARCHIVE_OK && archive_write_open_memory(writer.get(),buffer.data(),buffer.size(),&used)==ARCHIVE_OK,"Cannot create tool ZIP");
    for (const auto& item : entries) {
        std::unique_ptr<archive_entry,decltype(&archive_entry_free)> entry(archive_entry_new(),archive_entry_free);
        const auto path=tool.name+"/"+item.first;
        archive_entry_set_pathname(entry.get(),path.c_str()); archive_entry_set_size(entry.get(),item.second.size());
        archive_entry_set_filetype(entry.get(),AE_IFREG); archive_entry_set_perm(entry.get(),0644);
        require(archive_write_header(writer.get(),entry.get())==ARCHIVE_OK && archive_write_data(writer.get(),item.second.data(),item.second.size())==static_cast<la_ssize_t>(item.second.size()),"Cannot write tool ZIP");
    }
    require(archive_write_close(writer.get())==ARCHIVE_OK,"Cannot finalize tool ZIP"); return std::string(buffer.data(),used);
}
} // namespace rocos
