#include "model_bundle.hpp"

#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

#include "tinyxml2.h"

namespace rocos::dualarm {
namespace {
namespace fs = std::filesystem;
bool Within(const fs::path& path, const fs::path& root) {
  auto p = path.begin();
  for (auto r = root.begin(); r != root.end(); ++r, ++p)
    if (p == path.end() || *p != *r) return false;
  return true;
}
std::string Read(const fs::path& path) {
  if (!fs::is_regular_file(path) || fs::file_size(path) > 64 * 1024 * 1024)
    throw std::invalid_argument(
        "Model resource missing or larger than 64 MiB: " + path.string());
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::invalid_argument("Cannot read model resource: " + path.string());
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}
}  // namespace
ModelBundle::ModelBundle(const std::string& urdf,
                         const std::string& asset_root) {
  const auto root = fs::canonical(asset_root), file = fs::canonical(urdf);
  if (!Within(file, root))
    throw std::invalid_argument("URDF must be inside asset_root");
  xml_ = Read(file);
  tinyxml2::XMLDocument document;
  if (document.Parse(xml_.c_str()) != tinyxml2::XML_SUCCESS ||
      !document.FirstChildElement("robot"))
    throw std::invalid_argument("Invalid model URDF XML");
  auto* robot = document.FirstChildElement("robot");
  std::set<std::string> links, children;
  for (auto* link = robot->FirstChildElement("link"); link;
       link = link->NextSiblingElement("link"))
    if (link->Attribute("name")) links.insert(link->Attribute("name"));
  for (auto* joint = robot->FirstChildElement("joint"); joint;
       joint = joint->NextSiblingElement("joint")) {
    auto* child = joint->FirstChildElement("child");
    if (child && child->Attribute("link"))
      children.insert(child->Attribute("link"));
  }
  for (const auto& name : children) links.erase(name);
  if (links.size() != 1)
    throw std::invalid_argument("Model requires one root link");
  root_link_ = *links.begin();
  size_t total = xml_.size();
  std::function<void(tinyxml2::XMLElement*)> visit = [&](auto* element) {
    const std::string tag = element->Name();
    if ((tag == "mesh" || tag == "texture") && element->Attribute("filename")) {
      const std::string name = element->Attribute("filename");
      if (!assets_.count(name)) {
        fs::path path;
        if (name.rfind("package://", 0) == 0) {
          const auto slash = name.find('/', 10);
          if (slash == std::string::npos)
            throw std::invalid_argument("Invalid package URI");
          path = root / name.substr(slash + 1);
        } else {
          if (fs::path(name).is_absolute() ||
              name.find("://") != std::string::npos)
            throw std::invalid_argument(
                "Model assets must use local relative/package paths");
          path = file.parent_path() / name;
        }
        path = fs::canonical(path);
        if (!Within(path, root))
          throw std::invalid_argument("Model asset escapes asset_root");
        auto bytes = Read(path);
        total += bytes.size();
        if (total > 256 * 1024 * 1024)
          throw std::invalid_argument("Model bundle exceeds 256 MiB");
        assets_.emplace(name, std::move(bytes));
      }
    }
    for (auto* child = element->FirstChildElement(); child;
         child = child->NextSiblingElement())
      visit(child);
  };
  visit(robot);
  // Content checksum for cache invalidation, not a security signature.
  uint64_t hash = 14695981039346656037ULL;
  const auto feed = [&](const std::string& bytes) {
    for (unsigned char byte : bytes) {
      hash ^= byte;
      hash *= 1099511628211ULL;
    }
  };
  feed(xml_);
  for (const auto& asset : assets_) {
    feed(asset.first);
    feed(asset.second);
  }
  std::ostringstream version;
  version << "fnv1a64-" << std::hex << std::setw(16) << std::setfill('0')
          << hash;
  revision_ = version.str();
}
const std::string& ModelBundle::Asset(const std::string& name) const {
  const auto found = assets_.find(name);
  if (found == assets_.end())
    throw std::invalid_argument("Resource is not referenced by this model");
  return found->second;
}
nlohmann::json ModelBundle::Manifest() const {
  auto assets = nlohmann::json::array();
  for (const auto& asset : assets_)
    assets.push_back({{"path", asset.first}, {"bytes", asset.second.size()}});
  return {
      {"revision", revision_}, {"root_link", root_link_}, {"assets", assets}};
}
}  // namespace rocos::dualarm
