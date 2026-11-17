#pragma once
#include <filesystem>
#include <map>
#include <string>

#include "json.hpp"

namespace rocos::dualarm {
// Immutable startup snapshot. Only resources explicitly referenced by this URDF
// and contained in asset_root can be served; no filename fallback search.
class ModelBundle {
 public:
  ModelBundle(const std::string& urdf, const std::string& asset_root);
  const std::string& Xml() const { return xml_; }
  const std::string& Revision() const { return revision_; }
  const std::string& RootLink() const { return root_link_; }
  const std::string& Asset(const std::string& name) const;
  nlohmann::json Manifest() const;

 private:
  std::string xml_, revision_, root_link_;
  std::map<std::string, std::string> assets_;
};
}  // namespace rocos::dualarm
