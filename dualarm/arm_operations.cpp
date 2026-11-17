#include "arm_operations.hpp"
#include "tool_package_json.hpp"

#include <filesystem>

namespace rocos::dualarm {
namespace {
void Check(Result result) {
  if (!Succeeded(result))
    throw ApiError(static_cast<int>(result), "Robot operation failed");
}
std::string Name(Robot& robot, const Json& body, bool allow_empty = false) {
  const auto name = body.at("name").get<std::string>();
  if (!(allow_empty && name.empty()) && !robot.IsValidFrameName(name))
    throw ApiError(1001, "Invalid frame name");
  return name;
}
void DefaultPathOnly(const Json& body, const std::string& configured) {
  const auto requested = body.value("path", std::string());
  if (!requested.empty() &&
      (configured.empty() || std::filesystem::weakly_canonical(requested) !=
                                 std::filesystem::weakly_canonical(configured)))
    throw ApiError(
        1001,
        "Dual-arm persistence is restricted to this arm's configured path");
}
}  // namespace

Result ExecuteMotion(Robot& robot, const std::string& action,
                     const Json& body) {
  if (action.rfind("jog/", 0) == 0) {
    if (action == "jog/stop")
      return robot.IsMotionBusy() ? robot.StopMotion() : Result::NoError;
    const double timeout = Positive(body, "timeout", 0.1);
    const double threshold = Number(body, "dir_threshold", 0.99);
    if (threshold < -1 || threshold > 1)
      throw ApiError(1001, "dir_threshold must be in [-1, 1]");
    if (action == "jog/svd") {
      const size_t dimensions = std::max(0, robot.getJointNum() - 6);
      if (!dimensions)
        throw ApiError(1001, "Robot has no redundant dimensions");
      return robot.MoveSvdJogging(Values(body, "dim_speeds", dimensions),
                                  timeout, threshold);
    }
    const double speed = Positive(body, "speed", 1);
    const auto direction = body.value("direction", std::string("POSITIVE"));
    if (direction != "POSITIVE" && direction != "NEGATIVE")
      throw ApiError(1001, "Invalid direction");
    const double sign = direction == "POSITIVE" ? 1 : -1;
    if (action == "jog/cartesian") {
      const auto values = Values(body, "twist", 6);
      const auto name = body.value("frame", std::string("BASE"));
      const std::map<std::string, Robot::JogFrame> frames{
          {"BASE", Robot::JogFrame::BASE},
          {"FLANGE", Robot::JogFrame::FLANGE},
          {"TOOL", Robot::JogFrame::TOOL},
          {"OBJECT", Robot::JogFrame::OBJECT}};
      if (!frames.count(name)) throw ApiError(1001, "Invalid jog frame");
      Twist twist(Vector(values[0], values[1], values[2]) * sign,
                  Vector(values[3], values[4], values[5]) * sign);
      return robot.MoveJogging(twist, frames.at(name), speed, timeout,
                               threshold);
    }
    auto joints = Joints(body, robot.getJointNum());
    for (unsigned int i = 0; i < joints.rows(); ++i) joints(i) *= sign;
    if (action == "jog/nullspace")
      return robot.MoveNullJogging(joints, speed, timeout, threshold);
    return robot.MoveJogging(joints, speed, timeout, threshold);
  }
  const double velocity = body.contains("velocity")
                              ? Positive(body, "velocity", 1)
                              : Positive(body, "speed", 1);
  const double acceleration = Positive(body, "acceleration", 2);
  const double jerk = Positive(body, "jerk", 10);
  if (action == "movej")
    return robot.MoveJ(Joints(body, robot.getJointNum()), velocity,
                       acceleration, jerk);
  if (action == "movel_fk")
    return robot.MoveL_FK(Joints(body, robot.getJointNum()),
                          body.value("tool_name", std::string()), velocity,
                          acceleration, jerk);
  const auto pose = ParsePose(body.at("pose"));
  if (action == "movej_ik")
    return robot.MoveJ_IK(pose, velocity, acceleration, jerk);
  return robot.MoveL(pose, body.value("tool_name", std::string()), velocity,
                     acceleration, jerk);
}

Json ReadFrames(Robot& robot, const std::string& action,
                const httplib::Request& req) {
  if (action == "enabled") return {{"enabled", robot.IsEnabled()}};
  if (action == "load_params") {
    const auto com = robot.GetLoadCom();
    return {{"valid", robot.HasLoadParameters()},
            {"mass", robot.GetLoadMass()},
            {"com", {com.x(), com.y(), com.z()}}};
  }
  const bool tool = action == "tool_frames" || action == "tool_frame";
  if (action == "tool_frames" || action == "object_frames") {
    Json data{{"names",
               tool ? robot.GetToolFrameNames() : robot.GetObjectFrameNames()},
              {"active", tool ? robot.GetActiveToolFrameName()
                              : robot.GetActiveObjectFrameName()}};
    if (tool) {
      data["file_path"] = robot.GetFramesPath();
      data["package_support"] =
          false;  // Package import/edit is not exposed here.
    }
    return data;
  }
  const auto name = req.get_param_value("name");
  if (!(action == "tcp" && name.empty()) && !robot.IsValidFrameName(name))
    throw ApiError(1001, "Invalid or missing frame name");
  Frame frame;
  Check(action == "tcp" ? robot.GetTcpPose(frame, name)
        : tool          ? robot.GetToolFrame(name, frame)
                        : robot.GetObjectFrame(name, frame));
  if (action == "tcp")
    return {{"pose", PoseJson(frame)}, {"reference_frame", "base"}};
  Json data{{"name", name}, {"frame", PoseJson(frame)}};
  if (tool) {
    const auto package = robot.GetToolPackage(name);
    if (package) data["package"] = toolPackageJson(*package);
  }
  return data;
}

Json WriteFrames(Robot& robot, const std::string& action,
                 const httplib::Request& req, const Json& body) {
  // All dual-arm writers use the same arm mutex; do not change frames
  // mid-motion.
  if (robot.GetStateString() != "IDLE" && robot.GetStateString() != "STOPPED")
    throw ApiError(4005, "Configuration requires IDLE or STOPPED");
  if (action == "load_params") {
    const double mass = Number(body.at("mass"));
    auto com = body.contains("com") ? Values(body, "com", 3)
                                    : std::vector<double>{0, 0, 0};
    Check(robot.SetLoadParameters(mass, Vector(com[0], com[1], com[2])));
    return {{"mass", mass}, {"com", com}};
  }
  if (action == "load_params/save") {
    DefaultPathOnly(body, robot.GetLoadParametersPath());
    Check(robot.SaveLoadParameters());
    return {{"path", robot.GetLoadParametersPath()}};
  }
  if (action == "frames/load" || action == "frames/save") {
    DefaultPathOnly(body, robot.GetFramesPath());
    Check(action == "frames/load" ? robot.LoadFrames() : robot.SaveFrames());
    return {{"path", robot.GetFramesPath()},
            {"tool_frames", robot.GetToolFrameNames()},
            {"object_frames", robot.GetObjectFrameNames()},
            {"active_tool_frame", robot.GetActiveToolFrameName()},
            {"active_object_frame", robot.GetActiveObjectFrameName()}};
  }
  const bool tool = action == "tool_frame" || action == "active_tool_frame";
  if (req.method == "DELETE") {
    const auto name = Name(robot, Json{{"name", req.get_param_value("name")}});
    const auto value = req.get_param_value("persist");
    if (!value.empty() && value != "true" && value != "false")
      throw ApiError(1001, "persist must be true or false");
    const bool persist = value == "true";
    Check(tool ? robot.RemoveToolFrame(name, persist)
               : robot.RemoveObjectFrame(name));
    Json result{{"name", name},
                {"active", tool ? robot.GetActiveToolFrameName()
                                : robot.GetActiveObjectFrameName()}};
    if (tool) result["persisted"] = persist;
    return result;
  }
  const bool active = action.rfind("active_", 0) == 0;
  const auto name = Name(robot, body, active);
  const bool persist = body.value("persist", false);
  if (active) {
    Check(tool ? robot.SetActiveToolFrame(name, persist)
               : robot.SetActiveObjectFrame(name));
    Json result{{"active", name}};
    if (tool) result["persisted"] = persist;
    return result;
  }
  if (body.contains("previous_name") &&
      body.at("previous_name").get<std::string>() != name)
    throw ApiError(1001,
                   "Renaming tools is not supported by the dual-arm adapter");
  const auto frame =
      ParsePose(body.contains("frame") ? body.at("frame") : body.at("pose"));
  Check(tool ? robot.SetToolFrame(name, frame, persist)
             : robot.SetObjectFrame(name, frame));
  Json result{{"name", name}, {"frame", PoseJson(frame)}};
  if (tool) result["persisted"] = persist;
  return result;
}
}  // namespace rocos::dualarm
