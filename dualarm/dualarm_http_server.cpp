#include "dualarm_http_server.hpp"

#include <openssl/rand.h>

#include <array>
#include <cmath>
#include <set>
#include <stdexcept>

#include "arm_operations.hpp"
#include "robot_info_json.hpp"
#include "tool_package_json.hpp"

namespace rocos::dualarm {
namespace {
using Json = nlohmann::json;
std::string InstanceId() {
  std::array<unsigned char, 16> bytes{};
  if (RAND_bytes(bytes.data(), bytes.size()) != 1)
    throw std::runtime_error("Cannot create App instance identifier");
  const char* hex = "0123456789abcdef";
  std::string id;
  for (const auto byte : bytes) {
    id += hex[byte >> 4];
    id += hex[byte & 15];
  }
  return id;
}
void Reply(httplib::Response& res, bool success, int code,
           const std::string& message, const Json& data = nullptr) {
  res.set_content(Json{{"success", success},
                       {"code", code},
                       {"message", message},
                       {"data", data}}
                      .dump(),
                  "application/json");
}
void NotFound(httplib::Response& res) {
  res.status = 404;
  Reply(res, false, 1001, "Unknown dual-arm API path");
}
Json Body(const httplib::Request& req) {
  Json body =
      req.body.empty() ? Json::object() : Json::parse(req.body, nullptr, false);
  if (!body.is_object()) throw std::invalid_argument("Expected a JSON object");
  return body;
}
std::string String(const Json& body, const char* key) {
  if (!body.contains(key)) return "";
  if (!body[key].is_string() ||
      body[key].get_ref<const std::string&>().size() > 256)
    throw std::invalid_argument(std::string("Invalid ") + key);
  return body[key].get<std::string>();
}
Json Pose(const Frame& frame) {
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  return {{"position",
           {{"x", frame.p.x()}, {"y", frame.p.y()}, {"z", frame.p.z()}}},
          {"orientation", {{"x", x}, {"y", y}, {"z", z}, {"w", w}}}};
}
Json State(Robot& robot) {
  const auto snap = robot.GetRobotStateSnapshot();
  Json joints = Json::array();
  for (const auto& j : snap.joints)
    joints.push_back({{"id", j.id},
                      {"name", j.name},
                      {"status", j.status},
                      {"position", j.position},
                      {"velocity", j.velocity},
                      {"torque", j.torque},
                      {"load_torque", j.load_torque}});
  return {{"robot_state", snap.state_string},
          {"rci", snap.rci},
          {"is_enabled", snap.is_enabled},
          {"is_running", snap.is_running},
          {"control_active", snap.control_active},
          {"motion_busy", snap.motion_busy},
          {"work_mode", snap.work_mode},
          {"timestamp", snap.timestamp},
          {"joint_states", joints},
          {"flange", Pose(snap.flange)},
          {"tcp", snap.tcp_valid ? Pose(snap.tcp) : Json(nullptr)},
          {"active_tool_frame_name", snap.active_tool_frame_name},
          {"active_tool_frame", Pose(snap.active_tool_frame)},
          {"active_tool_package",
           snap.active_tool_package
               ? toolPackageJson(*snap.active_tool_package, false)
               : Json(nullptr)},
          {"active_object_frame_name", snap.active_object_frame_name},
          {"active_object_frame", Pose(snap.active_object_frame)},
          {"hw_state",
           {{"joint_num", snap.joint_num}, {"state", snap.hardware_state}}}};
}
}  // namespace

DualArmHttpServer::DualArmHttpServer(DualArmController& controller)
    : controller_(controller), instance_id_(InstanceId()) {
  const auto& config = controller.Config();
  if (!config.model_urdf.empty())
    model_ = std::make_unique<ModelBundle>(config.model_urdf,
                                           config.model_asset_root);
  server_.set_payload_max_length(64 * 1024);
  server_.set_read_timeout(5, 0);
  server_.set_write_timeout(5, 0);
  const auto handler = [this](const auto& req, auto& res) {
    const auto started = std::chrono::steady_clock::now();
    LogContext context;
    if (req.matches.size() == 4) {
      const std::string side = req.matches[1];
      if (side == "left" || side == "right") {
        const size_t index = side == "left" ? 0 : 1;
        context = {index == 0 ? LogScope::Left : LogScope::Right,
                   controller_.Config().arms[index].robot.ecat_id};
      }
    }
    ScopedLogContext log_scope(context);
    const auto request_id = std::to_string(logNowMicros()) + "-" +
                            std::to_string(++request_sequence_);
    res.set_header("X-Rocos-Request-Id", request_id);
    try {
      Handle(req, res);
    } catch (const ApiError& error) {
      Reply(res, false, error.code, error.what());
    } catch (const Json::exception&) {
      Reply(res, false, 1001, "Invalid JSON field type");
    } catch (const std::invalid_argument& error) {
      Reply(res, false, 1001, error.what());
    } catch (const std::exception& error) {
      // Response details and request bodies may contain credentials; audit only
      // normalized operation names, status and result codes below.
      (void)error;
      res.status = 500;
      Reply(res, false, -3001, "Dual-arm request failed");
    }
    const auto body = Json::parse(res.body, nullptr, false);
    const int http_status = res.status < 0 ? 200 : res.status;
    const bool success =
        http_status < 400 && (!body.is_object() || body.value("success", true));
    const int code = body.is_object() ? body.value("code", 0) : 0;
    const bool heartbeat = req.matches.size() == 4 &&
                           req.matches[2] == "control" &&
                           req.matches[3] == "heartbeat";
    // Successful polling/heartbeat is intentionally not an operation event.
    if (!success || (req.method != "GET" && !heartbeat)) {
      std::string operation = req.method + " unknown";
      if (res.status != 404 && req.matches.size() == 4)
        operation = req.method + " " + req.matches[2].str() + "/" +
                    req.matches[3].str();
      const std::string outcome = success ? "accepted" : "rejected";
      LogManager::instance().submit(
          success ? LogLevel::Info
                  : (http_status >= 500 ? LogLevel::Error : LogLevel::Warn),
          "DualArm.Http", operation + " " + outcome, code, 0,
          {operation, request_id, outcome, http_status,
           std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - started)
               .count()});
    }
  };
  server_.Get("/api/arms", handler);
  server_.Get(R"(/api/workcell/(model|urdf|urdf/mesh))", handler);
  const std::string route =
      R"(/api/arms/([^/]+)/(robot|control|calibration)/(.+))";
  server_.Get(route, handler);
  server_.Post(route, handler);
  server_.Delete(route, handler);
  server_.set_error_handler([this](const auto&, auto& res) {
    if (!res.has_header("X-Rocos-Request-Id")) {
      ScopedLogContext context(LogContext{});
      const auto id = std::to_string(logNowMicros()) + "-" +
                      std::to_string(++request_sequence_);
      res.set_header("X-Rocos-Request-Id", id);
      LogManager::instance().submit(LogLevel::Warn, "DualArm.Http",
                                    "Unknown or invalid HTTP request rejected",
                                    1001, 0,
                                    {"unknown", id, "rejected", res.status, 0});
    }
    if (res.body.empty())
      Reply(res, false, 1001, "Unknown or invalid dual-arm HTTP request");
  });
}
DualArmHttpServer::~DualArmHttpServer() { Stop(); }
void DualArmHttpServer::Start() {
  const auto& config = controller_.Config();
  if (!server_.bind_to_port(config.http_host, config.http_port))
    throw std::runtime_error("Cannot bind dual-arm HTTP port");
  running_ = true;
  try {
    thread_ = std::thread([this] {
      server_.listen_after_bind();
      running_ = false;
    });
    server_.wait_until_ready();
  } catch (...) {
    server_.stop();
    running_ = false;
    throw;
  }
}
void DualArmHttpServer::Stop() {
  server_.stop();
  if (thread_.joinable()) thread_.join();
  running_ = false;
}
void DualArmHttpServer::Handle(const httplib::Request& req,
                               httplib::Response& res) {
  if (req.path.rfind("/api/workcell/", 0) == 0) {
    HandleModel(req, res, req.matches[1]);
    return;
  }
  if (req.path == "/api/arms") {
    Json arms = Json::array();
    for (size_t i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      const auto& config = controller_.Config().arms[i].robot;
      arms.push_back({{"arm", side},
                      {"device_id", config.id},
                      {"ecat_id", config.ecat_id},
                      {"joint_count", controller_.Arm(i).getJointNum()},
                      {"api_prefix", "/api/arms/" + side},
                      {"base_link", config.base_link},
                      {"tip_link", config.tip_link},
                      {"joint_names", [&] {
                         Json names = Json::array();
                         for (const auto& joint :
                              controller_.Arm(i).GetJointInfo())
                           names.push_back(joint.name);
                         return names;
                       }()}});
    }
    Reply(res, true, 0, "ok",
          {{"execution_mode", "independent"},
           {"instance_id", instance_id_},
           {"arms", arms},
           {"model_url", model_ ? Json("/api/workcell/model") : Json(nullptr)},
           {"capabilities",
            {{"jog", {"joint", "cartesian", "nullspace", "svd"}},
             {"motion", {"movej", "movej_ik", "movel", "movel_fk"}},
             {"frames", true},
             {"six_point_calibration", true},
             {"tool_package_calibration", true},
             {"tool_packages", false},
             {"command_protocol", 1},
             {"synchronized_motion", false}}}});
    return;
  }
  const std::string side = req.matches[1], group = req.matches[2],
                    action = req.matches[3];
  const std::set<std::string> robot_get{
      "state",          "info",         "move_status", "enabled",
      "command_status", "tcp",          "tool_frames", "object_frames",
      "tool_frame",     "object_frame", "load_params", "urdf",
      "urdf/mesh"};
  const std::set<std::string> robot_post{"enable",
                                         "disable",
                                         "reset",
                                         "movej",
                                         "pause",
                                         "resume",
                                         "stop",
                                         "movej_ik",
                                         "movel",
                                         "movel_fk",
                                         "jog/joint",
                                         "jog/cartesian",
                                         "jog/nullspace",
                                         "jog/svd",
                                         "jog/stop",
                                         "tool_frame",
                                         "object_frame",
                                         "active_tool_frame",
                                         "active_object_frame",
                                         "frames/load",
                                         "frames/save",
                                         "load_params",
                                         "load_params/save"};
  const std::set<std::string> robot_delete{"tool_frame", "object_frame"};
  const std::set<std::string> control_post{"acquire", "release", "heartbeat"};
  const bool known =
      group == "calibration"
          ? ((req.method == "GET" && action == "result") ||
             (req.method == "POST" &&
              (action == "pose" || action == "run" || action == "save")))
      : group == "robot"
          ? (req.method == "GET"    ? robot_get.count(action)
             : req.method == "POST" ? robot_post.count(action)
                                    : robot_delete.count(action))
          : (req.method == "GET"
                 ? action == "status"
                 : req.method == "POST" && control_post.count(action));
  if (!known) {
    NotFound(res);
    return;
  }
  if (side != "left" && side != "right") {
    Reply(res, false, 1001, "arm must be left or right");
    return;
  }
  const size_t index = side == "left" ? 0 : 1;
  auto& robot = controller_.Arm(index);
  auto& context = controller_.Context(index);
  if (group == "robot" && req.method == "GET" && action == "command_status") {
    std::lock_guard<std::mutex> command_lock(context.command_mutex);
    const auto owner = req.get_header_value("X-Rocos-Control-Token");
    const auto auth = context.lease.Authorize(
        owner, req.get_header_value("X-Rocos-Client-Id"));
    if (auth) {
      Reply(res, false, auth,
            "Invalid or missing control credentials for selected arm");
      return;
    }
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    context.commands.Observe(State(robot));
    auto data = context.commands.Metadata(instance_id_);
    data["command"] =
        context.commands.Read(req.get_param_value("command_id"), owner);
    Reply(res, true, 0, "ok", data);
    return;
  }
  if (group == "robot" && req.method == "GET") {
    if (action == "urdf" || action == "urdf/mesh") {
      HandleModel(req, res, action);
      return;
    }
    if (action == "state") {
      // Serialize publication with acceptance/stop metadata, independently of
      // slow arm command dispatch. A planning record cannot finish on idle
      // data.
      std::lock_guard<std::mutex> lock(context.status_mutex);
      auto data = State(robot);
      context.commands.Observe(data);
      data.update(context.commands.Metadata(instance_id_));
      Reply(res, true, 0, "ok", data);
    } else if (action == "info")
      Reply(res, true, 0, "ok", robotInfoJson(robot.GetJointInfo()));
    else if (action == "move_status") {
      const auto snap = robot.GetRobotStateSnapshot();
      Json data{{"robot_state", snap.state_string},
                {"is_running", snap.is_running},
                {"control_active", snap.control_active}};
      if (!req.get_param_value("task_id").empty()) {
        data["task_id"] = req.get_param_value("task_id");
        data["task"] = nullptr;  // MoveJ does not create async task IDs in the
                                 // single-arm API.
      }
      Reply(res, true, 0, "ok", data);
    } else {
      std::lock_guard<std::mutex> lock(context.command_mutex);
      Reply(res, true, 0, "ok", ReadFrames(robot, action, req));
    }
    return;
  }
  // Lease mutation and command acceptance share one arm-local lock. Another arm
  // remains independently dispatchable even during a slow enable/disable call.
  std::lock_guard<std::mutex> lock(context.command_mutex);
  if (group == "calibration" && req.method == "GET") {
    Reply(res, true, 0, "ok",
          context.calibration.Read(req.get_param_value("session_id")));
    return;
  }
  if (group == "control" && action == "status") {
    auto data = context.lease.Status();
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    data.update(context.commands.Metadata(instance_id_));
    Reply(res, true, 0, "ok", data);
    return;
  }
  if (group == "control" && action == "acquire") {
    const auto body = Body(req);
    if (body.contains("command_protocol") &&
        !body["command_protocol"].is_number_integer())
      throw std::invalid_argument("command_protocol must be an integer");
    auto client = req.get_header_value("X-Rocos-Client-Id");
    if (client.empty()) client = String(body, "client_id");
    if (client.size() > 256) throw std::invalid_argument("Invalid client_id");
    Json data;
    const bool acquired = context.lease.Acquire(
        client, String(body, "client_name"), req.remote_addr, req.remote_port,
        req.get_header_value("User-Agent"), data, ControlLease::Clock::now(),
        body.value("command_protocol", 0));
    {
      std::lock_guard<std::mutex> status_lock(context.status_mutex);
      data.update(context.commands.Metadata(instance_id_));
    }
    Reply(res, acquired, acquired ? 0 : 3008,
          acquired ? "control acquired"
                   : "control already held by another client",
          data);
    return;
  }
  const int auth =
      context.lease.Authorize(req.get_header_value("X-Rocos-Control-Token"),
                              req.get_header_value("X-Rocos-Client-Id"));
  if (auth) {
    Reply(res, false, auth,
          "Invalid or missing control credentials for selected arm");
    return;
  }
  if (group == "control") {
    if (action == "heartbeat") {
      const auto body = Body(req);
      if (body.contains("command_protocol")) {
        if (!body["command_protocol"].is_number_integer() ||
            body["command_protocol"] != 1)
          throw std::invalid_argument(
              "Heartbeat can only upgrade command_protocol to 1");
        context.lease.EnableCommandProtocol();
      }
    }
    if (action == "release") context.lease.Release();
    auto data = context.lease.Status();
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    data.update(context.commands.Metadata(instance_id_));
    Reply(res, true, 0, "ok", data);
    return;
  }
  if (group == "calibration") {
    const auto state = robot.GetStateString();
    if (robot.IsRciEnabled() || (state != "IDLE" && state != "STOPPED"))
      throw ApiError(4005, "Stop the selected arm before calibration writes");
    const auto body = Body(req);
    const auto owner = req.get_header_value("X-Rocos-Control-Token");
    const auto lookup = [&](const std::string& name) {
      return robot.GetToolPackage(name);
    };
    Json data;
    if (action == "pose") {
      data = context.calibration.CapturePoint(
          body, owner,
          [&](auto& sample) {
            return robot.CaptureCalibrationPose(sample.flange, sample.joints,
                                                sample.timestamp);
          },
          lookup);
    } else if (action == "run") {
      try {
        data = context.calibration.Calculate(body, owner, lookup);
      } catch (const ApiError& error) {
        if (error.code != 4002) throw;
        Reply(
            res, false, error.code, error.what(),
            context.calibration.Read(body.at("session_id").get<std::string>()));
        return;
      }
    } else {
      context.calibration.Save(
          body, owner,
          [&](const auto& name, const auto& solution, const auto& digest,
              uint64_t revision, uint64_t& saved_revision) {
            return robot.SaveToolCalibration(name, solution, digest, revision,
                                             saved_revision);
          },
          lookup);
    }
    Reply(
        res, true, 0,
        action == "save" ? "Calibration saved; activate tool explicitly" : "ok",
        data);
    return;
  }
  const std::set<std::string> configuration{
      "tool_frame",  "object_frame", "active_tool_frame", "active_object_frame",
      "frames/load", "frames/save",  "load_params",       "load_params/save"};
  if (configuration.count(action)) {
    const auto data = WriteFrames(robot, action, req, Body(req));
    if (action == "frames/load") context.calibration.Clear();
    Reply(res, true, 0, "ok", data);
    return;
  }
  Result result = Result::NoError;
  const auto body = Body(req);
  const bool precise = action == "movej" || action == "movej_ik" ||
                       action == "movel" || action == "movel_fk";
  const bool stop = action == "stop" || action == "jog/stop";
  std::string command_id;
  if (stop) {
    if ((context.lease.CommandProtocol() == 1 ||
         body.contains("instance_id")) &&
        body.value("instance_id", std::string()) != instance_id_)
      throw ApiError(4105, "Stop requires the current App instance_id");
    // Invalidate queued and delayed old requests even when the arm is idle.
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    context.commands.Stop();
  } else if (precise || action.rfind("jog/", 0) == 0) {
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    const bool guarded = context.commands.Validate(
        body, instance_id_, context.lease.CommandProtocol() == 1, precise);
    if (guarded && precise) {
      command_id = body.at("command_id").get<std::string>();
      if (!context.commands.Begin(
              body, action, req.get_header_value("X-Rocos-Control-Token"))) {
        const auto previous = context.commands.Read(
            command_id, req.get_header_value("X-Rocos-Control-Token"));
        auto data = context.commands.Metadata(instance_id_);
        data["command"] = previous;
        Reply(res, previous.at("success"), previous.at("code"),
              previous.at("message"), data);
        return;
      }
    }
  }
  const auto finish = [&](bool success, int code, const std::string& message) {
    if (command_id.empty()) return;
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    context.commands.Finish(command_id, success, code, message);
  };
  try {
    if (stop) {
      // An idle stop is a successful cancellation fence, not an unsupported FSM
      // transition. It also covers precise requests that have not arrived yet.
      result = robot.IsMotionBusy() ? robot.StopMotion() : Result::NoError;
    } else if (precise || action.rfind("jog/", 0) == 0) {
      result = ExecuteMotion(robot, action, body);
    } else if (action == "enable")
      result = robot.SetEnabled();
    else if (action == "disable")
      result = robot.SetDisabled();
    else if (action == "reset")
      result = robot.ResetFault();
    else if (action == "pause")
      result = robot.PauseMotion();
    else if (action == "resume")
      result = robot.ResumeMotion();
  } catch (const ApiError& error) {
    finish(false, error.code, error.what());
    throw;
  } catch (const Json::exception&) {
    finish(false, 1001, "Invalid JSON field type");
    throw;
  } catch (const std::invalid_argument& error) {
    finish(false, 1001, error.what());
    throw;
  } catch (...) {
    finish(false, -3001, "Dual-arm request failed");
    throw;
  }
  const bool success =
      result == Result::NoError || result == Result::PlanFinished;
  finish(success, static_cast<int>(result),
         success ? action + " accepted" : action + " failed");
  Json data{{"robot_state", robot.GetStateString()}};
  if (action == "enable" || action == "disable" || action == "reset")
    data["enabled"] = robot.IsEnabled();
  else
    data["control_active"] = robot.IsControlActive();
  {
    std::lock_guard<std::mutex> status_lock(context.status_mutex);
    data.update(context.commands.Metadata(instance_id_));
  }
  Reply(res, success, static_cast<int>(result),
        success ? action + " accepted" : action + " failed", data);
}
void DualArmHttpServer::HandleModel(const httplib::Request& req,
                                    httplib::Response& res,
                                    const std::string& action) {
  if (!model_) {
    res.status = 404;
    Reply(res, false, -3001, "No workcell model configured");
    return;
  }
  if (action == "model") {
    auto data = model_->Manifest();
    data["urdf_url"] = "/api/workcell/urdf";
    data["mesh_url"] = "/api/workcell/urdf/mesh";
    data["arms"] = Json::array();
    for (size_t i = 0; i < 2; ++i) {
      const auto& config = controller_.Config().arms[i].robot;
      Json joints = Json::array();
      for (const auto& joint : controller_.Arm(i).GetJointInfo())
        joints.push_back(joint.name);
      data["arms"].push_back({{"arm", i == 0 ? "left" : "right"},
                              {"base_link", config.base_link},
                              {"tip_link", config.tip_link},
                              {"joint_names", joints}});
    }
    Reply(res, true, 0, "ok", data);
    return;
  }
  const std::string* bytes = &model_->Xml();
  if (action == "urdf/mesh") {
    try {
      bytes = &model_->Asset(req.get_param_value("path"));
    } catch (const std::invalid_argument& error) {
      res.status = 404;
      Reply(res, false, 1001, error.what());
      return;
    }
  }
  const auto etag = "\"" + model_->Revision() + "\"";
  res.set_header("ETag", etag);
  res.set_header("Cache-Control", "no-cache");
  if (req.get_header_value("If-None-Match") == etag) {
    res.status = 304;
    return;
  }
  res.set_content(*bytes,
                  action == "urdf" ? "text/xml" : "application/octet-stream");
}
}  // namespace rocos::dualarm
