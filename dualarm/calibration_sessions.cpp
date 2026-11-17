#include "calibration_sessions.hpp"

#include <algorithm>
#include <iomanip>
#include <random>
#include <sstream>

namespace rocos::dualarm {
namespace {
std::string Id(const Json& body) {
  if (!body.contains("session_id") || !body.at("session_id").is_string() ||
      body.at("session_id").get_ref<const std::string&>().empty())
    throw ApiError(1001, "Missing session_id");
  return body.at("session_id").get<std::string>();
}
std::string NewId() {
  std::random_device random;
  std::ostringstream out;
  out << "cal-" << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i)
    out << std::setw(8) << static_cast<uint32_t>(random());
  return out.str();
}
}  // namespace

CalibrationSessions::Session& CalibrationSessions::Owned(
    const std::string& id, const std::string& owner) {
  const auto found = sessions_.find(id);
  if (owner.empty() || found == sessions_.end() || found->second.owner != owner)
    throw ApiError(4004, "Calibration session unavailable for this arm/owner");
  return found->second;
}

Json CalibrationSessions::CapturePoint(const Json& body,
                                       const std::string& owner,
                                       const Capture& capture,
                                       const Lookup& lookup) {
  if (!body.contains("name") || !body.at("name").is_string() ||
      !FrameRegistry::validName(body.at("name").get<std::string>()) ||
      !body.contains("index") || !body.at("index").is_number_integer() ||
      Number(body.at("index")) < 1 || Number(body.at("index")) > 6 ||
      (body.contains("session_id") && !body.at("session_id").is_string()))
    throw ApiError(1001,
                   "Expected tool name, index in [1,6], optional session_id");
  // Samples always come from Robot feedback, never from a client pose.
  if (body.contains("pose") || body.contains("flange") ||
      body.contains("joints"))
    throw ApiError(1001,
                   "Client-supplied calibration samples are not accepted");
  if (owner.empty()) throw ApiError(4004, "Missing calibration owner");
  const auto name = body.at("name").get<std::string>();
  const int index = body.at("index").get<int>();
  auto id = body.value("session_id", std::string());
  if (id.empty() && index != 1)
    throw ApiError(4001, "Start calibration with point 1");
  if (!id.empty() && Owned(id, owner).name != name)
    throw ApiError(1001, "Tool name does not match calibration session");
  if (!id.empty()) CheckContext(Owned(id, owner), lookup);
  const auto package = lookup ? lookup(name) : std::nullopt;
  ToolCalibrationSample sample;
  if (capture(sample) != Result::NoError ||
      !FrameRegistry::validFrame(sample.flange) || sample.joints.empty() ||
      !std::isfinite(sample.timestamp) || sample.timestamp <= 0 ||
      !std::all_of(sample.joints.begin(), sample.joints.end(),
                   [](double value) { return std::isfinite(value); }))
    throw ApiError(4005, "Cannot capture a valid stationary flange pose");
  if (id.empty()) {
    if (sessions_.size() >= 16) {
      const auto oldest = std::min_element(
          sessions_.begin(), sessions_.end(), [](const auto& a, const auto& b) {
            return a.second.created < b.second.created;
          });
      sessions_.erase(oldest);
    }
    do {
      id = NewId();
    } while (sessions_.count(id));
    Session session;
    session.id = id;
    session.name = name;
    session.owner = owner;
    session.created = ++counter_;
    if (package) {
      session.package_digest = package->digest;
      session.package_revision = package->revision;
    }
    sessions_.emplace(id, std::move(session));
  }
  auto& session = Owned(id, owner);
  CheckContext(session, lookup);
  session.samples[index - 1] = std::move(sample);
  session.result.reset();
  return Serialize(session);
}

Json CalibrationSessions::Calculate(const Json& body, const std::string& owner,
                                    const Lookup& lookup) {
  const auto id = Id(body);
  const double tolerance = Number(body, "max_error", 0.001);
  if (tolerance <= 0 || tolerance > 0.05)
    throw ApiError(1001, "max_error must be in (0, 0.05] metres");
  auto& session = Owned(id, owner);
  session.result.reset();
  CheckContext(session, lookup);
  std::array<Frame, 6> poses;
  for (size_t i = 0; i < poses.size(); ++i) {
    if (!session.samples[i])
      throw ApiError(4001, "Capture all six points first");
    poses[i] = session.samples[i]->flange;
  }
  ToolCalibrationSolution solution;
  std::string error;
  if (!solveSixPointTool(poses, tolerance, solution, error))
    throw ApiError(4002, error);
  session.result = solution;
  return Serialize(session);
}

Json CalibrationSessions::Read(const std::string& id) const {
  if (id.empty()) throw ApiError(1001, "Missing session_id");
  const auto found = sessions_.find(id);
  if (found == sessions_.end())
    throw ApiError(4004, "Calibration session not found");
  return Serialize(found->second);
}

void CalibrationSessions::Save(const Json& body, const std::string& owner,
                               const Persist& persist, const Lookup& lookup) {
  if (body.size() != 1)
    throw ApiError(1001,
                   "Save accepts only session_id, not a client pose or path");
  auto& session = Owned(Id(body), owner);
  CheckContext(session, lookup);
  if (!session.result)
    throw ApiError(4001, "Calculate calibration before saving");
  uint64_t saved_revision = session.package_revision;
  const auto result =
      persist(session.name, *session.result, session.package_digest,
              session.package_revision, saved_revision);
  if (result == Result::CallingConflictError) {
    session.result.reset();
    throw ApiError(4004, "Tool installation changed; restart calibration");
  }
  if (result == Result::ConflictTaskRunning)
    throw ApiError(4005, "Stop the selected arm before saving calibration");
  if (result == Result::IllegalParameter)
    throw ApiError(1001, "Invalid calibration or tool context");
  if (result != Result::NoError)
    throw ApiError(static_cast<int>(result), "Cannot persist tool calibration");
  session.package_revision = saved_revision;
}

void CalibrationSessions::CheckContext(Session& session, const Lookup& lookup) {
  const auto package = lookup ? lookup(session.name) : std::nullopt;
  const bool current = package
                           ? package->digest == session.package_digest &&
                                 package->revision == session.package_revision
                           : session.package_digest.empty();
  if (!current) {
    session.result.reset();
    throw ApiError(4004, "Tool installation changed; restart calibration");
  }
}

Json CalibrationSessions::Serialize(const Session& session) {
  Json samples = Json::array(), result = nullptr;
  size_t count = 0;
  for (size_t i = 0; i < session.samples.size(); ++i) {
    if (!session.samples[i]) {
      samples.push_back(nullptr);
      continue;
    }
    ++count;
    const auto& sample = *session.samples[i];
    samples.push_back({{"index", i + 1},
                       {"flange", PoseJson(sample.flange)},
                       {"joints", sample.joints},
                       {"timestamp", sample.timestamp}});
  }
  if (session.result) {
    const auto& value = *session.result;
    result = {{"pose", PoseJson(value.frame)},
              {"error_state", false},
              {"metrics",
               {{"rms_error", value.rms_error},
                {"max_error", value.max_error},
                {"condition_number", value.condition_number},
                {"axis_angle_degrees", value.axis_angle_degrees}}}};
  }
  return {{"session_id", session.id},
          {"name", session.name},
          {"samples", samples},
          {"captured", count},
          {"result", result},
          {"package_digest", session.package_digest},
          {"package_revision", session.package_revision}};
}
}  // namespace rocos::dualarm
