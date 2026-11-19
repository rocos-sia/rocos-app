#include "motion_commands.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace rocos::dualarm {
bool MotionCommands::Terminal(const std::string& status) {
  return status == "finished" || status == "stopped" || status == "failed" ||
         status == "rejected";
}

bool MotionCommands::Validate(const Json& body, const std::string& instance,
                              bool required, bool precise) const {
  const bool guarded = required || body.contains("instance_id") ||
                       body.contains("motion_epoch") ||
                       body.contains("command_id");
  if (!guarded) return false;  // Legacy clients retain the original protocol.
  if (!body.contains("instance_id") || !body["instance_id"].is_string() ||
      !body.contains("motion_epoch") ||
      !body["motion_epoch"].is_number_integer())
    throw ApiError(1001,
                   "Motion requires instance_id and integer motion_epoch");
  if (body["instance_id"] != instance)
    throw ApiError(4105, "App instance changed; verify control again");
  if (body["motion_epoch"] != epoch_)
    throw ApiError(4101, "Motion epoch expired; this request cannot execute");
  if (precise) {
    if (!body.contains("command_id") || !body["command_id"].is_string())
      throw ApiError(1001, "Precise motion requires command_id");
    const auto id = body["command_id"].get<std::string>();
    if (id.empty() || id.size() > 128 ||
        !std::all_of(id.begin(), id.end(), [](unsigned char c) {
          return std::isalnum(c) || c == '-' || c == '_' || c == '.' ||
                 c == ':';
        }))
      throw ApiError(1001, "Invalid command_id");
  }
  return true;
}

bool MotionCommands::Begin(const Json& body, const std::string& operation,
                           const std::string& owner) {
  const auto id = body.at("command_id").get<std::string>();
  const auto previous = records_.find(id);
  if (previous != records_.end()) {
    if (previous->second.owner != owner || previous->second.body != body ||
        previous->second.data["operation"] != operation)
      throw ApiError(4102, "command_id already belongs to another request");
    return false;
  }
  // Never evict IDs in the current epoch: doing so would allow replay. Stop
  // advances the fence before making space for another group of commands.
  for (auto it = records_.begin(); it != records_.end();) {
    if (it->first != latest_ && it->second.data["motion_epoch"] != epoch_ &&
        Terminal(it->second.data["status"]))
      it = records_.erase(it);
    else
      ++it;
  }
  if (records_.size() >= 256)
    throw ApiError(4104, "Command history full; stop before submitting again");
  Record record;
  record.body = body;
  record.owner = owner;
  record.data = {{"command_id", id},
                 {"motion_epoch", epoch_},
                 {"operation", operation},
                 {"status", "planning"},
                 {"success", false},
                 {"code", 0},
                 {"message", "Command is being prepared"}};
  records_.emplace(id, std::move(record));
  latest_ = id;
  return true;
}

void MotionCommands::Finish(const std::string& id, bool success, int code,
                            const std::string& message) {
  auto& record = records_.at(id);
  record.data["success"] = success;
  record.data["code"] = code;
  record.data["message"] = message;
  record.data["status"] = success ? "accepted" : "rejected";
  record.after_sequence = sequence_;
}

void MotionCommands::Stop() {
  ++epoch_;
  for (auto& [id, record] : records_) {
    if (Terminal(record.data["status"])) continue;
    record.stopping = true;
    record.quiet = 0;
    record.after_sequence = sequence_;
    record.data["status"] = "stopping";
  }
}

void MotionCommands::Observe(const Json& state) {
  ++sequence_;
  const auto fsm = state.at("robot_state").get<std::string>();
  const bool fault = fsm.find("ERROR") != std::string::npos ||
                     fsm.find("FAULT") != std::string::npos;
  bool stationary = true;
  for (const auto& joint : state.at("joint_states")) {
    const auto velocity = joint.find("velocity");
    stationary = stationary && velocity != joint.end() &&
                 velocity->is_number() &&
                 std::isfinite(velocity->get<double>()) &&
                 std::abs(velocity->get<double>()) < 0.0001;
  }
  for (auto& [id, record] : records_) {
    const auto status = record.data["status"].get<std::string>();
    if (Terminal(status) || status == "planning" ||
        sequence_ <= record.after_sequence)
      continue;
    if (fault) {
      record.data["status"] = "failed";
      record.data["message"] = "Robot entered " + fsm;
    } else if (state.value("motion_busy", true) ||
               (fsm != "STOPPED" && fsm != "IDLE")) {
      record.quiet = 0;
      record.data["status"] = record.stopping   ? "stopping"
                              : fsm == "PAUSED" ? "paused"
                                                : "running";
    } else {
      record.quiet = stationary ? record.quiet + 1 : 0;
      if (record.quiet >= 3)
        record.data["status"] = record.stopping ? "stopped" : "finished";
    }
  }
}

Json MotionCommands::Metadata(const std::string& instance) const {
  const auto last = records_.find(latest_);
  return {
      {"instance_id", instance},
      {"motion_epoch", epoch_},
      {"state_seq", sequence_},
      {"command", last == records_.end() ? Json(nullptr) : last->second.data}};
}

Json MotionCommands::Read(const std::string& id,
                          const std::string& owner) const {
  const auto found = records_.find(id);
  if (found == records_.end() || found->second.owner != owner)
    throw ApiError(4103, "Command outcome unavailable for this arm/owner");
  return found->second.data;
}
}  // namespace rocos::dualarm
