#pragma once
#include <array>
#include <functional>
#include <map>
#include <optional>
#include <string>

#include "api_values.hpp"
#include "tool_calibration.hpp"

namespace rocos::dualarm {
// Access under the owning arm's command mutex; no shared sessions across arms.
class CalibrationSessions {
 public:
  using Capture = std::function<Result(ToolCalibrationSample&)>;
  using Lookup = std::function<std::optional<ToolPackage>(const std::string&)>;
  using Persist =
      std::function<Result(const std::string&, const ToolCalibrationSolution&,
                           const std::string&, uint64_t, uint64_t&)>;
  Json CapturePoint(const Json& body, const std::string& owner,
                    const Capture& capture, const Lookup& lookup = {});
  Json Calculate(const Json& body, const std::string& owner,
                 const Lookup& lookup = {});
  Json Read(const std::string& id) const;
  void Save(const Json& body, const std::string& owner, const Persist& persist,
            const Lookup& lookup = {});
  void Clear() { sessions_.clear(); }

 private:
  struct Session {
    std::string id, name, owner;
    uint64_t created = 0;
    std::string package_digest;
    uint64_t package_revision = 0;
    std::array<std::optional<ToolCalibrationSample>, 6> samples;
    std::optional<ToolCalibrationSolution> result;
  };
  Session& Owned(const std::string& id, const std::string& owner);
  static Json Serialize(const Session& session);
  static void CheckContext(Session& session, const Lookup& lookup);
  std::map<std::string, Session> sessions_;
  uint64_t counter_ = 0;
};
}  // namespace rocos::dualarm
