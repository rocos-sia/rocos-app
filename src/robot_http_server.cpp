
#include "robot_http_server.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <kdl_parser/kdl_parser.hpp>
#include <random>
#include <sstream>
#include <stdexcept>

#include "lua_script_engine.hpp"
#include "robot.hpp"
#include "robot_info_json.hpp"
#include "tool_package_json.hpp"

namespace rocos {

namespace {

int resultCode(Result result) {
    return static_cast<int>(result);
}

bool resultSucceeded(Result result) {
    return result == Result::NoError || result == Result::PlanFinished;
}

const char* scriptResultMessage(Result result) {
    switch (result) {
        case Result::LuaStateConflict:
            return "script state conflict";
        case Result::LuaExecutionError:
            return "Lua script compilation or execution failed";
        case Result::LuaInvalidBreakpoint:
            return "invalid script breakpoint";
        case Result::LuaFileError:
            return "script file unavailable";
        case Result::LuaStopped:
            return "script stopped";
        default:
            return "script operation failed";
    }
}

double jsonNumberOr(const nlohmann::json& body,
                    const char* primary_key,
                    const char* fallback_key,
                    double default_value) {
    if (body.contains(primary_key)) {
        return body[primary_key].get<double>();
    }
    if (fallback_key != nullptr && body.contains(fallback_key)) {
        return body[fallback_key].get<double>();
    }
    return default_value;
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool directionSign(const std::string& direction, double& sign) {
    if (direction == "POSITIVE") {
        sign = 1.0;
        return true;
    }
    if (direction == "NEGATIVE") {
        sign = -1.0;
        return true;
    }
    if (direction == "NONE") {
        sign = 0.0;
        return true;
    }
    return false;
}

bool parseJogFrame(const std::string& frame, Robot::JogFrame& jog_frame) {
    if (frame == "BASE") {
        jog_frame = Robot::JogFrame::BASE;
        return true;
    }
    if (frame == "FLANGE") {
        jog_frame = Robot::JogFrame::FLANGE;
        return true;
    }
    if (frame == "TOOL") {
        jog_frame = Robot::JogFrame::TOOL;
        return true;
    }
    if (frame == "OBJECT") {
        jog_frame = Robot::JogFrame::OBJECT;
        return true;
    }
    return false;
}


bool parseFrameJson(const nlohmann::json& j, KDL::Frame& frame) {
    if (!j.is_object()) return false;

    if (j.contains("position") && j.contains("orientation") &&
        j["position"].is_object() && j["orientation"].is_object()) {
        const auto& pos = j["position"];
        const auto& rot = j["orientation"];
        const double x = pos.value("x", 0.0);
        const double y = pos.value("y", 0.0);
        const double z = pos.value("z", 0.0);
        const double qx = rot.value("x", 0.0);
        const double qy = rot.value("y", 0.0);
        const double qz = rot.value("z", 0.0);
        const double qw = rot.value("w", 1.0);
        if (!isFinite(x) || !isFinite(y) || !isFinite(z) ||
            !isFinite(qx) || !isFinite(qy) || !isFinite(qz) || !isFinite(qw)) {
            return false;
        }
        frame = KDL::Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw),
                           KDL::Vector(x, y, z));
        return true;
    }

    const double x = j.value("x", 0.0);
    const double y = j.value("y", 0.0);
    const double z = j.value("z", 0.0);
    const double qx = j.value("qx", 0.0);
    const double qy = j.value("qy", 0.0);
    const double qz = j.value("qz", 0.0);
    const double qw = j.value("qw", 1.0);
    if (!isFinite(x) || !isFinite(y) || !isFinite(z) ||
        !isFinite(qx) || !isFinite(qy) || !isFinite(qz) || !isFinite(qw)) {
        return false;
    }
    frame = KDL::Frame(KDL::Rotation::Quaternion(qx, qy, qz, qw),
                       KDL::Vector(x, y, z));
    return true;
}

bool parseNamedFrameBody(const nlohmann::json& body,
                         std::string& name,
                         KDL::Frame& frame) {
    if (body.is_discarded() || !body.contains("name") || !body["name"].is_string()) {
        return false;
    }
    const nlohmann::json* frame_json = nullptr;
    if (body.contains("frame")) {
        frame_json = &body["frame"];
    } else if (body.contains("pose")) {
        frame_json = &body["pose"];
    }
    if (frame_json == nullptr) return false;

    name = body["name"].get<std::string>();
    return parseFrameJson(*frame_json, frame);
}

}  // namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

RobotHttpServer::RobotHttpServer(Robot* robot, LuaScriptEngine* script_engine, int control_ttl_seconds,
                                 const std::string& web_root)
    : robot_(robot),
      script_engine_(script_engine),
      server_(new httplib::Server()),
      taskCounter_(0),
      poolShutdown_(false),
      activeWorkers_(0),
      control_ttl_seconds_(control_ttl_seconds),
      web_root_(web_root) {
    if (control_ttl_seconds_ <= 0) {
        throw std::invalid_argument("control_ttl_seconds must be positive");
    }

    frame_registry_ = robot ? &robot->frames_ : nullptr;
    if (robot) state_snapshot_ = [robot] { return robot->GetRobotStateSnapshot(); };
    tcp_pose_ = [robot](Frame& tcp, const std::string& name) {
        return robot ? robot->GetTcpPose(tcp, name) : Result::ResourceUnavailable;
    };
    tool_motion_busy_ = [robot] { return !robot || robot->IsMotionBusy(); };
    calibration_capture_ = [robot](ToolCalibrationSample& sample) {
        if (!robot) return Result::ResourceUnavailable;
        return robot->CaptureCalibrationPose(sample.flange, sample.joints, sample.timestamp);
    };
    log_ptr_ = Logger::getInstance("HttpServer");

    // Start worker thread pool
    for (int i = 0; i < MAX_WORKERS; ++i) {
        workerThreads_.emplace_back([this]() {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(poolMutex_);
                    poolCv_.wait(lock, [this]() {
                        return poolShutdown_ || !taskQueue_.empty();
                    });
                    if (poolShutdown_ && taskQueue_.empty()) return;
                    if (taskQueue_.empty()) continue;
                    task = std::move(taskQueue_.front());
                    taskQueue_.pop();
                }
                ++activeWorkers_;
                try {
                    task();
                } catch (...) {
                    log_ptr_->error("Async task threw exception");
                }
                --activeWorkers_;
            }
        });
    }
    registerRoutes();
}

RobotHttpServer::~RobotHttpServer() {
    {
        std::lock_guard<std::mutex> lock(poolMutex_);
        poolShutdown_ = true;
    }
    poolCv_.notify_all();
    for (auto& t : workerThreads_) {
        if (t.joinable()) t.join();
    }
    stop();
}

void RobotHttpServer::submitTask(std::function<void()> func) {
    {
        std::lock_guard<std::mutex> lock(poolMutex_);
        taskQueue_.push(std::move(func));
    }
    poolCv_.notify_one();
}

// ============================================================================
// Server Lifecycle
// ============================================================================

void RobotHttpServer::run(const std::string& host, int port) {
    log_ptr_->info("HTTP Server listening on {}:{}", host, port);
    server_->listen(host, port);
}

void RobotHttpServer::runAsync(const std::string& host, int port) {
    thread_.reset(new std::thread([this, host, port]() {
        this->run(host, port);
    }));
}

void RobotHttpServer::stop() {
    if (server_) server_->stop();
    if (thread_ && thread_->joinable()) thread_->join();
}

// ============================================================================
// Route Registration
// ============================================================================

void RobotHttpServer::registerRoutes() {
    if (!web_root_.empty()) {
        if (!server_->set_mount_point("/", web_root_)) {
            log_ptr_->error("Failed to mount web UI directory: {}", web_root_);
        }
    } else if (!server_->set_mount_point("/", "web") && !server_->set_mount_point("/", "../web") &&
               !server_->set_mount_point("/", "../../web")) {
        log_ptr_->error("Failed to mount web UI directory. Tried: web, ../web, ../../web");
    }

    // CORS preflight
    server_->Options(".*", [this](const httplib::Request&, httplib::Response& res) {
        setCorsHeaders(res);
    });

    // Robot state
    server_->Get("/api/robot/state", [this](auto& req, auto& res) { handleGetRobotState(req, res); });
    server_->Get("/api/robot/tcp", [this](auto& req, auto& res) { handleGetTcpPose(req, res); });
    server_->Get("/api/robot/info", [this](auto& req, auto& res) { handleGetRobotInfo(req, res); });
    server_->Get("/api/robot/impedance", [this](auto& req, auto& res) { handleGetImpedance(req, res); });
    server_->Get("/api/robot/urdf", [this](auto& req, auto& res) { handleGetRobotModel(req, res); });
    server_->Get("/api/robot/urdf/mesh", [this](auto& req, auto& res) { handleGetLinkMesh(req, res); });
    for (const auto* path : {"/api/robot/tool_packages", "/api/robot/tool_package/asset", "/api/robot/tool_package/export", "/api/robot/tool_package/installation"})
        server_->Get(path, [this](auto& req, auto& res) { handleToolPackages(req, res); });
    for (const auto* path : {"/api/robot/tool_packages/import", "/api/robot/tool_packages/reload", "/api/robot/tool_package/overrides", "/api/robot/tool_package/installation", "/api/calibration/save"})
        server_->Post(path, [this](auto& req, auto& res) { handleToolPackages(req, res); });
    server_->Get("/api/robot/enabled", [this](auto& req, auto& res) { handleIsEnabled(req, res); });
    server_->Post("/api/robot/enable", [this](auto& req, auto& res) { handleEnable(req, res); });
    server_->Post("/api/robot/disable", [this](auto& req, auto& res) { handleDisable(req, res); });
    server_->Post("/api/robot/reset", [this](auto& req, auto& res) { handleReset(req, res); });
    server_->Post("/api/robot/workmode", [this](auto& req, auto& res) { handleSetWorkMode(req, res); });
    server_->Post("/api/robot/load_params", [this](auto& req, auto& res) { handleSetLoadParameters(req, res); });
    server_->Get("/api/robot/load_params", [this](auto& req, auto& res) { handleGetLoadParameters(req, res); });
    server_->Post("/api/robot/load_params/save", [this](auto& req, auto& res) { handleSaveLoadParameters(req, res); });

    // Robot servo control
    server_->Post("/api/robot/servo/start", [this](auto& req, auto& res) { handleServoStart(req, res); });
    server_->Post("/api/robot/servo/stop", [this](auto& req, auto& res) { handleServoStop(req, res); });
    server_->Post("/api/robot/servo/mode", [this](auto& req, auto& res) { handleServoMode(req, res); });

    // Robot motion control
    server_->Post("/api/robot/movej", [this](auto& req, auto& res) { handleMoveJ(req, res); });
    server_->Post("/api/robot/movej_ik", [this](auto& req, auto& res) { handleMoveJ_IK(req, res); });
    server_->Post("/api/robot/movel", [this](auto& req, auto& res) { handleMoveL(req, res); });
    server_->Post("/api/robot/movel_fk", [this](auto& req, auto& res) { handleMoveL_FK(req, res); });
    server_->Post("/api/robot/movec", [this](auto& req, auto& res) { handleMoveC(req, res); });
    server_->Post("/api/robot/pause", [this](auto& req, auto& res) { handlePause(req, res); });
    server_->Post("/api/robot/resume", [this](auto& req, auto& res) { handleResume(req, res); });
    server_->Post("/api/robot/stop", [this](auto& req, auto& res) { handleStop(req, res); });
    server_->Post("/api/robot/wait_move", [this](auto& req, auto& res) { handleWaitMove(req, res); });
    server_->Get("/api/robot/move_status", [this](auto& req, auto& res) { handleMoveStatus(req, res); });

    // Robot jogging — 全部向量传递
    server_->Post("/api/robot/jog/joint", [this](auto& req, auto& res) { handleJogJoint(req, res); });
    server_->Post("/api/robot/jog/cartesian", [this](auto& req, auto& res) { handleJogCartesian(req, res); });
    server_->Post("/api/robot/jog/nullspace", [this](auto& req, auto& res) { handleJogNullspace(req, res); });
    server_->Post("/api/robot/jog/svd", [this](auto& req, auto& res) { handleJogSvd(req, res); });
    server_->Post("/api/robot/jog/stop", [this](auto& req, auto& res) { handleJogStop(req, res); });

    // Frame management
    server_->Get("/api/robot/tool_frames", [this](auto& req, auto& res) { handleGetToolFrameNames(req, res); });
    server_->Get("/api/robot/object_frames", [this](auto& req, auto& res) { handleGetObjectFrameNames(req, res); });
    server_->Get("/api/robot/tool_frame", [this](auto& req, auto& res) { handleGetToolFrame(req, res); });
    server_->Get("/api/robot/object_frame", [this](auto& req, auto& res) { handleGetObjectFrame(req, res); });
    server_->Post("/api/robot/tool_frame", [this](auto& req, auto& res) { handleSetToolFrame(req, res); });
    server_->Post("/api/robot/object_frame", [this](auto& req, auto& res) { handleSetObjectFrame(req, res); });
    server_->Delete("/api/robot/tool_frame", [this](auto& req, auto& res) { handleRemoveToolFrame(req, res); });
    server_->Delete("/api/robot/object_frame", [this](auto& req, auto& res) { handleRemoveObjectFrame(req, res); });
    server_->Post("/api/robot/active_tool_frame", [this](auto& req, auto& res) { handleSetActiveToolFrame(req, res); });
    server_->Post("/api/robot/active_object_frame", [this](auto& req, auto& res) { handleSetActiveObjectFrame(req, res); });
    server_->Post("/api/robot/frames/load", [this](auto& req, auto& res) { handleLoadFrames(req, res); });
    server_->Post("/api/robot/frames/save", [this](auto& req, auto& res) { handleSaveFrames(req, res); });

    // Calibration
    server_->Post("/api/calibration/pose", [this](auto& req, auto& res) { handleSetPoseFrame(req, res); });
    server_->Post("/api/calibration/tool", [this](auto& req, auto& res) { handleSetToolFrame(req, res); });
    server_->Post("/api/calibration/object", [this](auto& req, auto& res) { handleSetObjectFrame(req, res); });
    server_->Post("/api/calibration/run", [this](auto& req, auto& res) { handleCalibrationRun(req, res); });
    server_->Get("/api/calibration/result", [this](auto& req, auto& res) { handleCalibrationResult(req, res); });

    // Lua script
    server_->Post("/api/script/upload", [this](auto& req, auto& res) { handleScriptUpload(req, res); });
    server_->Post("/api/script/run", [this](auto& req, auto& res) { handleScriptRun(req, res); });
    server_->Post("/api/script/pause", [this](auto& req, auto& res) { handleScriptPause(req, res); });
    server_->Post("/api/script/resume", [this](auto& req, auto& res) { handleScriptResume(req, res); });
    server_->Post("/api/script/stop", [this](auto& req, auto& res) { handleScriptStop(req, res); });
    server_->Post("/api/script/step", [this](auto& req, auto& res) { handleScriptStep(req, res); });
    server_->Get("/api/script/status", [this](auto& req, auto& res) { handleScriptStatus(req, res); });
    server_->Get("/api/script/list", [this](auto& req, auto& res) { handleScriptList(req, res); });
    server_->Get("/api/script/download", [this](auto& req, auto& res) { handleScriptDownload(req, res); });
    server_->Post("/api/script/breakpoint/add", [this](auto& req, auto& res) { handleScriptBreakpointAdd(req, res); });
    server_->Post("/api/script/breakpoint/remove", [this](auto& req, auto& res) { handleScriptBreakpointRemove(req, res); });
    server_->Post("/api/script/breakpoint/clear", [this](auto& req, auto& res) { handleScriptBreakpointClear(req, res); });

    // Control rights (single-holder lock)
    server_->Post("/api/control/heartbeat", [this](auto& req, auto& res) { handleControlHeartbeat(req, res); });
    server_->Post("/api/control/acquire", [this](auto& req, auto& res) { handleControlAcquire(req, res); });
    server_->Post("/api/control/release", [this](auto& req, auto& res) { handleControlRelease(req, res); });
    server_->Post("/api/control/takeover", [this](auto& req, auto& res) { handleControlTakeover(req, res); });
    server_->Get("/api/control/status", [this](auto& req, auto& res) { handleControlStatus(req, res); });

    
	    // 修改后：去掉 ".*",
	    server_->set_post_routing_handler([](const httplib::Request&, httplib::Response& res) {
	    if (res.body.empty() && (res.status == 0 || res.status == 404)) {
	        res.status = 404;
        // 按照设计文档，业务错误码使用 4 位编码；这里统一返回通用参数错误码 1001
        res.set_content(R"({"success":false,"code":1001,"message":"endpoint not found","data":null})", "application/json");
    }
});
}

// ============================================================================
// Utility: CORS
// ============================================================================

void RobotHttpServer::setCorsHeaders(httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
    res.set_header("Access-Control-Allow-Headers",
                   "Content-Type, X-Rocos-Control-Token, X-Rocos-Client-Id");
    res.set_header("Access-Control-Expose-Headers",
                   "X-Rocos-Control-Token, X-Rocos-Client-Id");
}

// ============================================================================
// Utility: JSON Response
// ============================================================================

void RobotHttpServer::sendJson(httplib::Response& res, bool success,
                               int businessCode, const std::string& message,
                               const nlohmann::json& data) {
    setCorsHeaders(res);
    res.set_header("Content-Type", "application/json");

    nlohmann::json body;
    body["success"] = success;
    body["code"] = businessCode;
    body["message"] = message;
    body["data"] = data;

    res.set_content(body.dump(), "application/json");
}

void RobotHttpServer::sendScriptResult(httplib::Response& res, Result result,
                                       const std::string& success_message) {
    const bool success = result == Result::NoError;
    sendJson(res, success, resultCode(result),
             success ? success_message : scriptResultMessage(result),
             scriptStatusToJson());
}

nlohmann::json RobotHttpServer::scriptStatusToJson() const {
    if (script_engine_ == nullptr) {
        return nlohmann::json();
    }

    const LuaScriptEngine::Status status = script_engine_->GetStatus();
    nlohmann::json breakpoints = nlohmann::json::array();
    for (const auto& breakpoint : status.breakpoints) {
        breakpoints.push_back({{"filename", breakpoint.filename},
                               {"line", breakpoint.line}});
    }

    return {{"state", LuaScriptEngine::ToString(status.state)},
            {"script_id", status.script_id},
            {"filename", status.location.filename},
            {"line", status.location.line},
            {"error", status.error},
            {"motion_active", status.motion_active},
            {"breakpoints", std::move(breakpoints)}};
}

// ============================================================================
// Utility: Control Rights (single-holder lock)
// ============================================================================

std::string RobotHttpServer::makeConnectionKey(const std::string& remote_addr, int remote_port) {
    if (remote_addr.empty()) {
        return "unknown:" + std::to_string(remote_port);
    }
    return remote_addr + ":" + std::to_string(remote_port);
}

std::string RobotHttpServer::makeConnectionKey(const httplib::Request& req) {
    return makeConnectionKey(req.remote_addr, req.remote_port);
}

std::string RobotHttpServer::extractClientId(const httplib::Request& req) {
    if (req.has_header(CONTROL_CLIENT_ID_HEADER)) {
        return req.get_header_value(CONTROL_CLIENT_ID_HEADER);
    }
    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_discarded() && body.contains("client_id") && body["client_id"].is_string()) {
        return body["client_id"].get<std::string>();
    }
    return std::string{};
}

std::string RobotHttpServer::generateControlToken() {
    // 生成 UUID v4 字符串。使用 random_device + mt19937_64 提供足够的熵。
    static thread_local std::mt19937_64 rng(
        std::random_device{}() ^
        static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<uint64_t> dist;
    const uint64_t a = dist(rng);
    const uint64_t b = dist(rng);

    unsigned char bytes[16];
    for (int i = 0; i < 8; ++i) bytes[i]     = static_cast<unsigned char>((a >> (8 * i)) & 0xFF);
    for (int i = 0; i < 8; ++i) bytes[i + 8] = static_cast<unsigned char>((b >> (8 * i)) & 0xFF);
    bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0F) | 0x40);  // version 4
    bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3F) | 0x80);  // variant RFC4122

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (int i = 0; i < 16; ++i) {
        oss << std::setw(2) << static_cast<int>(bytes[i]);
        if (i == 3 || i == 5 || i == 7 || i == 9) oss << '-';
    }
    return oss.str();
}

RobotHttpServer::AuthResult RobotHttpServer::checkAndRenewToken_locked(const httplib::Request& req, bool allow_debug) {
    if (!req.has_header(CONTROL_TOKEN_HEADER)) return AuthResult::MissingToken;
    const std::string token = req.get_header_value(CONTROL_TOKEN_HEADER);

    // 调试旁路：使用预设 DEBUG_TOKEN + DEBUG_CLIENT_ID 可直接控制机器人，无需 acquire。
    // 仅用于 curl/脚本调试，生产环境可注释掉此段以禁用。
    if (allow_debug && token == DEBUG_TOKEN) {
        const std::string req_client_id = req.get_header_value(CONTROL_CLIENT_ID_HEADER);
        if (req_client_id == DEBUG_CLIENT_ID) {
            log_ptr_->info("Debug bypass: token accepted (client_id={})", req_client_id);
            return AuthResult::OK;
        }
    }

    if (current_token_.empty()) return AuthResult::InvalidToken;

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count() >= control_ttl_seconds_) {
        clearControl_locked("ttl expired");
        return AuthResult::Expired;
    }
    if (token != current_token_) return AuthResult::InvalidToken;

    // 验证 client_id：若 acquire 时记录了 client_id，后续请求必须携带相同的 client_id。
    // 这防止同一浏览器不同标签页通过共享 localStorage 窃取 token 绕过控制权检查。
    if (!current_owner_client_id_.empty()) {
        const std::string req_client_id = req.get_header_value(CONTROL_CLIENT_ID_HEADER);
        if (req_client_id.empty()) {
            log_ptr_->warn(
                "Control token valid but request missing X-Rocos-Client-Id "
                "(owner client_id={})", current_owner_client_id_);
            return AuthResult::InvalidToken;
        }
        if (req_client_id != current_owner_client_id_) {
            log_ptr_->warn(
                "Control token valid but client_id mismatch "
                "(expected={}, got={})",
                current_owner_client_id_, req_client_id);
            return AuthResult::InvalidToken;
        }
    }

    last_seen_at_ = now;
    return AuthResult::OK;
}

bool RobotHttpServer::ensureControl(const httplib::Request& req, httplib::Response& res, bool allow_debug,
                                    nlohmann::json* owner) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    switch (checkAndRenewToken_locked(req, allow_debug)) {
        case AuthResult::OK:
            if (owner != nullptr) *owner = controlOwnerJson_locked();
            return true;
        case AuthResult::MissingToken:
            sendJson(res, false, 3006,
                     "no control right, call POST /api/control/acquire first");
            return false;
        case AuthResult::InvalidToken:
            sendJson(res, false, 3007, "control token invalid");
            return false;
        case AuthResult::Expired:
            sendJson(res, false, 3007, "control token expired");
            return false;
    }
    return false;
}

void RobotHttpServer::clearControl_locked(const char* reason) {
    if (!current_token_.empty()) {
        log_ptr_->info("Control released (owner={}@{}:{}, reason={})",
                       current_owner_name_, current_owner_ip_, current_owner_port_, reason);
    }
    current_token_.clear();
    current_owner_ip_.clear();
    current_owner_port_ = -1;
    current_owner_connection_.clear();
    current_owner_client_id_.clear();
    current_owner_name_.clear();
    current_owner_agent_.clear();
    acquired_at_ = {};
    last_seen_at_ = {};
}

nlohmann::json RobotHttpServer::controlOwnerJson_locked() const {
    nlohmann::json data;
    const bool has_owner = !current_token_.empty();
    data["has_owner"] = has_owner;
    data["owner_ip"] = current_owner_ip_;
    data["owner_port"] = current_owner_port_;
    data["owner_connection"] = current_owner_connection_;
    data["owner_client_id"] = current_owner_client_id_;
    data["owner_name"] = current_owner_name_;
    data["owner_agent"] = current_owner_agent_;

    if (has_owner) {
        const auto now = std::chrono::steady_clock::now();
        const auto age = std::chrono::duration_cast<std::chrono::seconds>(now - acquired_at_).count();
        const auto idle = std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count();
        const long long expires_in = control_ttl_seconds_ - idle;
        data["held_for_seconds"] = age;
        data["idle_seconds"] = idle;
        data["expires_in_seconds"] = expires_in > 0 ? expires_in : 0;
    } else {
        data["held_for_seconds"] = 0;
        data["idle_seconds"] = 0;
        data["expires_in_seconds"] = 0;
    }
    data["ttl_seconds"] = control_ttl_seconds_;
    data["heartbeat_interval_seconds"] = std::min(5.0, control_ttl_seconds_ / 3.0);
    return data;
}

void RobotHttpServer::handleControlAcquire(const httplib::Request& req,
                                           httplib::Response& res) {
    log_ptr_->info("POST /api/control/acquire from {}", req.remote_addr);

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string client_name;
    if (!body.is_discarded() && body.contains("client_name") &&
        body["client_name"].is_string()) {
        client_name = body["client_name"].get<std::string>();
    }
    std::string client_id = extractClientId(req);
    if (client_id.empty() && !body.is_discarded() && body.contains("client_id") &&
        body["client_id"].is_string()) {
        client_id = body["client_id"].get<std::string>();
    }
    // 若客户端未提供 client_id，服务端自动生成一个，
    // 确保每个 token 都有绑定的 client_id，防止同源标签页 token 窃取。
    const bool client_id_auto_generated = client_id.empty();
    if (client_id_auto_generated) {
        client_id = generateControlToken();  // UUID v4 作为 client_id
    }
    const std::string user_agent = req.has_header("User-Agent")
        ? req.get_header_value("User-Agent") : std::string{};

    std::lock_guard<std::mutex> lock(control_mutex_);

    // 若已有持有者但已过期则先清空
    if (!current_token_.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count() >= control_ttl_seconds_) {
            clearControl_locked("ttl expired on acquire");
        }
    }

    if (!current_token_.empty()) {
        nlohmann::json data = controlOwnerJson_locked();
        sendJson(res, false, 3008, "control already held by another client", data);
        return;
    }

    current_token_ = generateControlToken();
    current_owner_ip_ = req.remote_addr;
    current_owner_port_ = req.remote_port;
    current_owner_connection_ = makeConnectionKey(req);
    current_owner_client_id_ = client_id;
    current_owner_name_ = client_name;
    current_owner_agent_ = user_agent;
    const auto now = std::chrono::steady_clock::now();
    acquired_at_ = now;
    last_seen_at_ = now;

    log_ptr_->info("Control acquired by {}@{}:{} ({}) client_id={} auto={}",
                   current_owner_name_, current_owner_ip_, current_owner_port_,
                   current_owner_agent_, current_owner_client_id_, client_id_auto_generated);

    nlohmann::json data = controlOwnerJson_locked();
    data["token"] = current_token_;
    // 返回 client_id，前端必须存储并在后续请求中通过 X-Rocos-Client-Id 头回传
    data["client_id"] = current_owner_client_id_;
    data["client_id_auto"] = client_id_auto_generated;
    sendJson(res, true, 0, "control acquired", data);
}

void RobotHttpServer::handleControlRelease(const httplib::Request& req,
                                           httplib::Response& res) {
    log_ptr_->info("POST /api/control/release from {}", req.remote_addr);

    if (!req.has_header(CONTROL_TOKEN_HEADER)) {
        sendJson(res, false, 3006,
                 "no control right, cannot release without token");
        return;
    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    const AuthResult auth = checkAndRenewToken_locked(req);
    if (auth != AuthResult::OK) {
        sendJson(res, false, 3007, auth == AuthResult::Expired ? "control token expired" : "control token invalid");
        return;
    }
    clearControl_locked("released by owner");
    sendJson(res, true, 0, "control released", controlOwnerJson_locked());
}

void RobotHttpServer::handleControlTakeover(const httplib::Request& req,
                                            httplib::Response& res) {
    log_ptr_->info("POST /api/control/takeover from {}", req.remote_addr);

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string client_name;
    if (!body.is_discarded() && body.contains("client_name") &&
        body["client_name"].is_string()) {
        client_name = body["client_name"].get<std::string>();
    }
    std::string client_id = extractClientId(req);
    if (client_id.empty() && !body.is_discarded() && body.contains("client_id") &&
        body["client_id"].is_string()) {
        client_id = body["client_id"].get<std::string>();
    }
    const bool client_id_auto_generated = client_id.empty();
    if (client_id_auto_generated) {
        client_id = generateControlToken();
    }
    const std::string user_agent = req.has_header("User-Agent")
        ? req.get_header_value("User-Agent") : std::string{};

    std::lock_guard<std::mutex> lock(control_mutex_);
    if (!current_token_.empty()) {
        log_ptr_->warn("Control taken over: previous owner {}@{}:{} forcibly evicted by {}@{}:{}",
                       current_owner_name_, current_owner_ip_, current_owner_port_,
                       client_name, req.remote_addr, req.remote_port);
        clearControl_locked("takeover");
    }

    current_token_ = generateControlToken();
    current_owner_ip_ = req.remote_addr;
    current_owner_port_ = req.remote_port;
    current_owner_connection_ = makeConnectionKey(req);
    current_owner_client_id_ = client_id;
    current_owner_name_ = client_name;
    current_owner_agent_ = user_agent;
    const auto now = std::chrono::steady_clock::now();
    acquired_at_ = now;
    last_seen_at_ = now;

    nlohmann::json data = controlOwnerJson_locked();
    data["token"] = current_token_;
    data["client_id"] = current_owner_client_id_;
    data["client_id_auto"] = client_id_auto_generated;
    sendJson(res, true, 0, "control taken over", data);
}

void RobotHttpServer::handleControlStatus(const httplib::Request&,
                                          httplib::Response& res) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    // 顺手做一次懒清理：读接口发现已过期则视为无持有者
    if (!current_token_.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_at_).count() >= control_ttl_seconds_) {
            clearControl_locked("ttl expired on status query");
        }
    }
    sendJson(res, true, 0, "ok", controlOwnerJson_locked());
}

void RobotHttpServer::handleControlHeartbeat(const httplib::Request& req, httplib::Response& res) {
    nlohmann::json owner;
    if (!ensureControl(req, res, false, &owner)) return;
    sendJson(res, true, 0, "ok", owner);
}

// ============================================================================
// Utility: Frame <-> JSON Conversion
// ============================================================================

KDL::Frame RobotHttpServer::jsonToFrame(const nlohmann::json& j) {
    double px = j.value("x", 0.0);
    double py = j.value("y", 0.0);
    double pz = j.value("z", 0.0);
    double qx = j.value("qx", 0.0);
    double qy = j.value("qy", 0.0);
    double qz = j.value("qz", 0.0);
    double qw = j.value("qw", 1.0);

    return KDL::Frame(
        KDL::Rotation::Quaternion(qx, qy, qz, qw),
        KDL::Vector(px, py, pz));
}

KDL::Frame RobotHttpServer::jsonToFrame(const nlohmann::json& pos,
                                        const nlohmann::json& rot) {
    KDL::Vector p(pos.value("x", 0.0),
                  pos.value("y", 0.0),
                  pos.value("z", 0.0));

    KDL::Rotation r = KDL::Rotation::Quaternion(
        rot.value("x", 0.0),
        rot.value("y", 0.0),
        rot.value("z", 0.0),
        rot.value("w", 1.0));

    return KDL::Frame(r, p);
}

std::string RobotHttpServer::getUrdfPath() { return robot_ != nullptr ? robot_->GetUrdfPath() : "robot.urdf"; }

nlohmann::json RobotHttpServer::frameToJson(const KDL::Frame& frame) {
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);

    nlohmann::json position;
    position["x"] = frame.p.x();
    position["y"] = frame.p.y();
    position["z"] = frame.p.z();

    nlohmann::json orientation;
    orientation["x"] = x;
    orientation["y"] = y;
    orientation["z"] = z;
    orientation["w"] = w;

    nlohmann::json j;
    j["position"] = position;
    j["orientation"] = orientation;

    return j;
}

// ============================================================================
// Utility: Async Task Management
// ============================================================================

std::string RobotHttpServer::generateTaskId() {
    std::lock_guard<std::mutex> lock(taskMutex_);
    ++taskCounter_;
    return "task_" + std::to_string(taskCounter_);
}

void RobotHttpServer::registerTask(const std::string& taskId, const std::string& type) {
    std::lock_guard<std::mutex> lock(taskMutex_);
    nlohmann::json info;
    info["task_id"] = taskId;
    info["type"] = type;
    info["status"] = "RUNNING";
    info["result"] = 0;
    info["message"] = "";
    taskMap_[taskId] = info;
}

void RobotHttpServer::updateTaskStatus(const std::string& taskId, const std::string& status,
                                       int result, const std::string& message) {
    std::lock_guard<std::mutex> lock(taskMutex_);
    auto it = taskMap_.find(taskId);
    if (it != taskMap_.end()) {
        it->second["status"] = status;
        it->second["result"] = result;
        it->second["message"] = message;
    }
}

nlohmann::json RobotHttpServer::getTaskInfo(const std::string& taskId) {
    std::lock_guard<std::mutex> lock(taskMutex_);
    auto it = taskMap_.find(taskId);
    if (it != taskMap_.end()) {
        return it->second;
    }
    return nlohmann::json();
}

void RobotHttpServer::cleanExpiredTasks() {
    std::lock_guard<std::mutex> lock(taskMutex_);
    auto now = std::chrono::steady_clock::now();
    for (auto it = taskMap_.begin(); it != taskMap_.end(); ) {
        auto& info = it->second;
        std::string status = info.value("status", "");
        if (status == "COMPLETED" || status == "FAILED" || status == "STOPPED") {
            if (info.contains("_finish_time")) {
                auto finish = std::chrono::steady_clock::time_point(
                    std::chrono::milliseconds(info["_finish_time_ms"].get<long long>()));
                if (std::chrono::duration_cast<std::chrono::seconds>(now - finish).count() > TASK_TTL_SECONDS) {
                    it = taskMap_.erase(it);
                    continue;
                }
            }
        }
        ++it;
    }
}

// ============================================================================
// Robot State Handlers
// ============================================================================

void RobotHttpServer::handleGetRobotState(const httplib::Request& req, httplib::Response& res) {
    if (!state_snapshot_) { sendJson(res, false, -3001, "Robot state is unavailable"); return; }
    const auto snap = state_snapshot_();

    nlohmann::json data;
    data["robot_state"] = snap.state_string;
    data["is_enabled"] = snap.is_enabled;
    data["is_running"] = snap.is_running;
    data["control_active"] = snap.control_active;
    data["motion_busy"] = snap.motion_busy;
    data["work_mode"] = snap.work_mode;
    data["timestamp"] = snap.timestamp;

    // 关节状态
    nlohmann::json joint_states = nlohmann::json::array();
    for (const auto& js : snap.joints) {
        nlohmann::json j;
        j["id"] = js.id;
        j["name"] = js.name;
        j["status"] = js.status;
        j["position"] = js.position;
        j["velocity"] = js.velocity;
        j["torque"] = js.torque;
        j["load_torque"] = js.load_torque;
        joint_states.push_back(j);
    }
    data["joint_states"] = joint_states;

    // 法兰末端位姿
    data["flange"] = frameToJson(snap.flange);
    data["tcp"] = snap.tcp_valid ? frameToJson(snap.tcp) : nlohmann::json(nullptr);

    // 激活的坐标系
    data["active_tool_frame_name"] = snap.active_tool_frame_name;
    data["active_tool_frame"] = frameToJson(snap.active_tool_frame);
    data["active_tool_package"] = snap.active_tool_package ? toolPackageJson(*snap.active_tool_package, false) : nlohmann::json(nullptr);
    data["active_object_frame_name"] = snap.active_object_frame_name;
    data["active_object_frame"] = frameToJson(snap.active_object_frame);

    // 硬件摘要
    nlohmann::json hw;
    hw["joint_num"] = snap.joint_num;
    hw["state"] = snap.hardware_state;
    data["hw_state"] = hw;

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetTcpPose(const httplib::Request& req, httplib::Response& res) {
    const auto name = req.get_param_value("name");
    if (!name.empty() && !FrameRegistry::validName(name)) {
        sendJson(res, false, 1001, "Invalid tool name"); return;
    }
    Frame tcp;
    const auto result = tcp_pose_(tcp, name);
    if (result != Result::NoError) {
        sendJson(res, false, resultCode(result), "TCP query failed: check tool name and flange feedback"); return;
    }
    sendJson(res, true, 0, "ok", {{"pose", frameToJson(tcp)}, {"reference_frame", "base"}});
}

void RobotHttpServer::handleGetRobotInfo(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/info");

    sendJson(res, true, 0, "ok",
             robotInfoJson(robot_ ? robot_->GetJointInfo() : std::vector<Robot::JointInfo>{}));
}

void RobotHttpServer::handleGetRobotModel(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/urdf");

    const std::string urdf_path = getUrdfPath();
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        log_ptr_->error("Failed to open URDF file: {}", urdf_path);
        sendJson(res, false, 1001, "URDF file not found: " + urdf_path);
        return;
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    setCorsHeaders(res);
    res.set_content(content, "text/xml");
}

void RobotHttpServer::handleGetImpedance(const httplib::Request& /*req*/,
                                          httplib::Response& res) {
    log_ptr_->info("GET /api/robot/impedance");
    const nlohmann::json data = robot_->GetImpedanceParams();
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetLinkMesh(const httplib::Request& req, httplib::Response& res) {
    namespace fs = std::filesystem;

    std::string path = req.get_param_value("path");
    log_ptr_->info("GET /api/robot/urdf/mesh path={}", path);

    if (path.empty()) {
        sendJson(res, false, 1001, "Missing 'path' query parameter");
        return;
    }

    // 解析 package:// ROS 路径格式：package://pkg_name/relative/path.stl
    // 剥离 scheme + package 名，保留相对路径部分（如 meshes/link1.stl）
    std::string rel_path = path;
    if (path.rfind("package://", 0) == 0) {
        const auto after_pkg = path.find('/', 10); // 跳过 "package://"
        if (after_pkg == std::string::npos) {
            sendJson(res, false, 1001, "Invalid package:// path: " + path);
            return;
        }
        rel_path = path.substr(after_pkg + 1); // e.g., "meshes/link1.stl"
    }

    // 禁止路径穿越攻击
    if (rel_path.find("..") != std::string::npos) {
        sendJson(res, false, 1001, "Path traversal not allowed");
        return;
    }

    // 提取文件名，用于回退搜索
    const auto sep = rel_path.find_last_of("/\\");
    const std::string filename = (sep != std::string::npos)
        ? rel_path.substr(sep + 1)
        : rel_path;

    // 从当前 URDF 所在目录查找，避免命中 bin 中的旧副本。
    const fs::path urdf_dir = fs::absolute(getUrdfPath()).parent_path();
    const std::vector<fs::path> search_bases = {urdf_dir};

    std::string found_path;

    // 1. 先尝试 base/rel_path 精确匹配
    for (const auto& base : search_bases) {
        fs::path candidate = base / rel_path;
        std::error_code ec;
        if (fs::is_regular_file(candidate, ec)) {
            found_path = candidate.string();
            break;
        }
    }

    // 2. 精确匹配失败时，按文件名在各目录树中递归搜索
    if (found_path.empty()) {
        for (const auto& base : search_bases) {
            std::error_code ec;
            if (!fs::is_directory(base, ec)) continue;
            for (const auto& entry :
                 fs::recursive_directory_iterator(base,
                     fs::directory_options::skip_permission_denied, ec)) {
                if (entry.is_regular_file() &&
                    entry.path().filename().string() == filename) {
                    found_path = entry.path().string();
                    break;
                }
            }
            if (!found_path.empty()) break;
        }
    }

    if (found_path.empty()) {
        log_ptr_->error("Mesh file not found: {}", path);
        sendJson(res, false, 1001, "Mesh file not found: " + filename);
        return;
    }

    std::ifstream file(found_path, std::ios::binary);
    if (!file.is_open()) {
        log_ptr_->error("Failed to open mesh file: {}", found_path);
        sendJson(res, false, 1001, "Failed to open mesh file");
        return;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    const std::string content = ss.str();

    log_ptr_->info("Serving mesh: {}", found_path);
    setCorsHeaders(res);
    res.set_header("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    res.set_content(content, "application/octet-stream");
}

// ============================================================================
// Robot Control Handlers
// ============================================================================

void RobotHttpServer::handleEnable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/enable");
    if (!ensureControl(req, res)) return;
    const Result result = robot_->SetEnabled();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["enabled"] = robot_->IsEnabled();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Robot enabled" : "Robot enable failed",
             data);
}

void RobotHttpServer::handleDisable(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/disable");
    if (!ensureControl(req, res)) return;
    const Result result = robot_->SetDisabled();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["enabled"] = robot_->IsEnabled();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Robot disabled" : "Robot disable failed",
             data);
}

void RobotHttpServer::handleReset(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/reset");
    if (!ensureControl(req, res)) return;
    const Result result = robot_->ResetFault();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["enabled"] = robot_->IsEnabled();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Robot reset to STOPPED" : "Robot reset failed",
             data);
}

void RobotHttpServer::handleIsEnabled(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/enabled");
    nlohmann::json data;
    data["enabled"] = robot_->IsEnabled();
    data["disabled"] = robot_->IsDisabled();
    data["robot_state"] = robot_->GetStateString();
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleSetWorkMode(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/workmode");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("mode") || !body["mode"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'mode' field");
        return;
    }

    const std::string mode_str = body["mode"].get<std::string>();
    const Result result = robot_->SetWorkMode(mode_str);

    nlohmann::json data;
    data["mode"] = mode_str;
    data["robot_state"] = robot_->GetStateString();

    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result)
                 ? "Work mode set to " + mode_str
                 : "Failed to set work mode",
             data);
}

void RobotHttpServer::handleSetLoadParameters(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/load_params");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("mass") || !body["mass"].is_number()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'mass' field");
        return;
    }

    const double mass = body["mass"].get<double>();

    // com 可选，默认质心在法兰原点 [0,0,0]
    KDL::Vector com(0.0, 0.0, 0.0);
    if (body.contains("com")) {
        const auto& com_json = body["com"];
        if (!com_json.is_array() || com_json.size() != 3) {
            sendJson(res, false, 1001, "'com' must be a 3-element array [x, y, z]");
            return;
        }
        for (const auto& v : com_json) {
            if (!v.is_number()) {
                sendJson(res, false, 1001, "'com' elements must be numbers");
                return;
            }
        }
        com = KDL::Vector(com_json[0].get<double>(),
                          com_json[1].get<double>(),
                          com_json[2].get<double>());
    }

    const Result result = robot_->SetLoadParameters(mass, com);

    nlohmann::json data;
    data["mass"] = mass;
    data["com"] = {com.x(), com.y(), com.z()};

    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result)
                 ? "Load parameters set"
                 : "Failed to set load parameters",
             data);
}

void RobotHttpServer::handleGetLoadParameters(const httplib::Request& req, httplib::Response& res) {
    (void)req;
    log_ptr_->info("GET /api/robot/load_params");

    nlohmann::json data;
    data["valid"] = robot_->HasLoadParameters();
    data["mass"] = robot_->GetLoadMass();
    const KDL::Vector com = robot_->GetLoadCom();
    data["com"] = {com.x(), com.y(), com.z()};

    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleSaveLoadParameters(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/load_params/save");
    if (!ensureControl(req, res)) return;

    // 可选 path 参数；缺省用默认 payload.yaml
    std::string path;
    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_discarded() && body.contains("path") && body["path"].is_string()) {
        path = body["path"].get<std::string>();
    }

    const Result result = robot_->SaveLoadParameters(path);

    nlohmann::json data;
    data["path"] = path.empty() ? robot_->GetLoadParametersPath() : path;

    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result)
                 ? "Load parameters saved"
                 : "Failed to save load parameters",
             data);
}

// ============================================================================
// Servo Control Handlers
// ============================================================================

void RobotHttpServer::handleServoStart(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/servo/start");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    uint16_t port = 8081;
    if (!body.is_discarded() && body.contains("port") && body["port"].is_number()) {
        int p = body["port"].get<int>();
        if (p > 0 && p <= 65535) {
            port = static_cast<uint16_t>(p);
        }
    }

    const Result result = robot_->MoveServoing(port);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    data["port"] = port;

    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Servo mode started"
                                     : "Servo start failed",
             data);
}

void RobotHttpServer::handleServoStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/servo/stop");
    if (!ensureControl(req, res)) return;

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();

    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Servo mode stopped"
                                     : "Servo stop failed",
             data);
}

void RobotHttpServer::handleServoMode(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/servo/mode");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("mode") || !body["mode"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'mode' field");
        return;
    }

    const std::string mode_str = body["mode"].get<std::string>();
    const Result result = robot_->SetServoMode(mode_str);

    nlohmann::json data;
    data["mode"] = mode_str;
    data["robot_state"] = robot_->GetStateString();

    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result)
                 ? "Servo mode set to " + mode_str
                 : "Failed to set servo mode",
             data);
}

// ============================================================================
// Motion Control Handlers
// ============================================================================

void RobotHttpServer::handleMoveJ(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movej");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints") ||
        !body["joints"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }

    const int joint_count = robot_->getJointNum();
    const auto& joints = body["joints"];
    if (static_cast<int>(joints.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray q_goal(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!joints[i].is_number()) {
            sendJson(res, false, 1001, "Joint values must be numbers");
            return;
        }
        const double value = joints[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Joint values must be finite");
            return;
        }
        q_goal(static_cast<unsigned int>(i)) = value;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const Result result = robot_->MoveJ(q_goal, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveJ accepted" : "MoveJ failed",
             data);
}

void RobotHttpServer::handleMoveJ_IK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movej_ik");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose") ||
        !body["pose"].is_object()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }
    const auto& pose_json = body["pose"];
    Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const Result result = robot_->MoveJ_IK(pose, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveJ_IK accepted" : "MoveJ_IK failed",
             data);
}

void RobotHttpServer::handleMoveL(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movel");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("pose") ||
        !body["pose"].is_object()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'pose' object");
        return;
    }

    const auto& pose_json = body["pose"];
    Frame pose;
    if (pose_json.contains("position") && pose_json.contains("orientation")) {
        pose = jsonToFrame(pose_json["position"], pose_json["orientation"]);
    } else {
        pose = jsonToFrame(pose_json);
    }

    const std::string tool_name = body.value("tool_name", std::string{});
    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const Result result = robot_->MoveL(pose, tool_name, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveL accepted" : "MoveL failed",
             data);
}

void RobotHttpServer::handleMoveL_FK(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movel_fk");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints") ||
        !body["joints"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }
    const int joint_count = robot_->getJointNum();
    const auto& joints = body["joints"];
    if (static_cast<int>(joints.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray q_goal(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!joints[i].is_number()) {
            sendJson(res, false, 1001, "Joint values must be numbers");
            return;
        }
        const double value = joints[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Joint values must be finite");
            return;
        }
        q_goal(static_cast<unsigned int>(i)) = value;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    const std::string tool_name = body.value("tool_name", std::string{});
    const Result result = robot_->MoveL_FK(q_goal, tool_name, velocity, acceleration, jerk);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveL_FK accepted" : "MoveL_FK failed",
             data);
}

void RobotHttpServer::handleMoveC(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/movec");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }

    Frame pose_start = robot_->getFlange();
    if (body.contains("pose_start") && !parseFrameJson(body["pose_start"], pose_start)) {
        sendJson(res, false, 1001, "Invalid 'pose_start' frame");
        return;
    }

    const double velocity = jsonNumberOr(body, "velocity", "speed", 1.0);
    const double acceleration = jsonNumberOr(body, "acceleration", nullptr, 2.0);
    const double jerk = jsonNumberOr(body, "jerk", nullptr, 10.0);
    if (!isFinite(velocity) || !isFinite(acceleration) || !isFinite(jerk) ||
        velocity <= 0.0 || acceleration <= 0.0 || jerk <= 0.0) {
        sendJson(res, false, 1001, "velocity, acceleration and jerk must be positive finite numbers");
        return;
    }

    Result result = Result::IllegalParameter;
    if (body.contains("center_frame") && body.contains("theta")) {
        Frame center_frame;
        if (!parseFrameJson(body["center_frame"], center_frame) || !body["theta"].is_number()) {
            sendJson(res, false, 1001, "Invalid 'center_frame' or 'theta'");
            return;
        }
        const double theta = body["theta"].get<double>();
        if (!isFinite(theta)) {
            sendJson(res, false, 1001, "theta must be finite");
            return;
        }
        result = robot_->MoveC(pose_start, center_frame, theta, velocity, acceleration, jerk);
    } else if (body.contains("pose_via") && (body.contains("pose_goal") || body.contains("pose_to"))) {
        Frame pose_via;
        Frame pose_goal;
        const auto& goal_json = body.contains("pose_goal") ? body["pose_goal"] : body["pose_to"];
        if (!parseFrameJson(body["pose_via"], pose_via) || !parseFrameJson(goal_json, pose_goal)) {
            sendJson(res, false, 1001, "Invalid 'pose_via' or 'pose_goal' frame");
            return;
        }
        result = robot_->MoveC(pose_start, pose_via, pose_goal, velocity, acceleration, jerk);
    } else {
        sendJson(res, false, 1001,
                 "MoveC requires either 'center_frame' + 'theta' or 'pose_via' + 'pose_goal'");
        return;
    }

    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "MoveC accepted" : "MoveC failed",
             data);
}

void RobotHttpServer::handleStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/stop");
    if (!ensureControl(req, res)) return;

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion stop requested" : "Motion stop failed",
             data);
}

void RobotHttpServer::handlePause(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/pause");
    if (!ensureControl(req, res)) return;

    const Result result = robot_->PauseMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion pause requested" : "Motion pause failed",
             data);
}

void RobotHttpServer::handleResume(const httplib::Request& req, httplib::Response& res) {
  log_ptr_->info("POST /api/robot/resume");
    if (!ensureControl(req, res)) return;

    const Result result = robot_->ResumeMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion resume requested" : "Motion resume failed",
             data);
}

void RobotHttpServer::handleWaitMove(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/wait_move");
    if (!ensureControl(req, res)) return;

    const Result result = robot_->WaitMove();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    data["motion_busy"] = robot_->IsMotionBusy();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Motion finished" : "WaitMove failed",
             data);
}

// ============================================================================
// Async Task Query
// ============================================================================

void RobotHttpServer::handleMoveStatus(const httplib::Request& req, httplib::Response& res) {
    std::string task_id = req.get_param_value("task_id");
    log_ptr_->info("GET /api/robot/move_status task_id={}", task_id);

    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["is_running"] = robot_->IsRunning();
    data["control_active"] = robot_->IsControlActive();
    if (!task_id.empty()) {
        data["task_id"] = task_id;
        data["task"] = getTaskInfo(task_id);
    }
    sendJson(res, true, 0, "ok", data);
}

// ============================================================================
// Lua Script Handlers
// ============================================================================

void RobotHttpServer::handleScriptUpload(const httplib::Request& req,
                                         httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("filename") ||
        !body["filename"].is_string() || !body.contains("source") ||
        !body["source"].is_string()) {
        sendJson(res, false, 1001,
                 "Invalid JSON or missing string 'filename'/'source' fields");
        return;
    }

    const Result result = script_engine_->LoadSource(
        body["source"].get<std::string>(), body["filename"].get<std::string>());
    sendScriptResult(res, result, "script uploaded");
}

void RobotHttpServer::handleScriptRun(const httplib::Request& req,
                                      httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Run(), "script started");
}

void RobotHttpServer::handleScriptPause(const httplib::Request& req,
                                        httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Pause(), "script pause requested");
}

void RobotHttpServer::handleScriptResume(const httplib::Request& req,
                                         httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Resume(), "script resumed");
}

void RobotHttpServer::handleScriptStop(const httplib::Request& req,
                                       httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Stop(), "script stop requested");
}

void RobotHttpServer::handleScriptStep(const httplib::Request& req,
                                       httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->Step(), "script step requested");
}

void RobotHttpServer::handleScriptStatus(const httplib::Request&,
                                         httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendJson(res, true, 0, "ok", scriptStatusToJson());
}

void RobotHttpServer::handleScriptList(const httplib::Request&,
                                       httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    const std::vector<std::string> scripts = script_engine_->ListScripts();
    sendJson(res, true, 0, "ok", nlohmann::json{{"scripts", scripts}});
}

void RobotHttpServer::handleScriptDownload(const httplib::Request& req,
                                           httplib::Response& res) {
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const std::string filename = req.get_param_value("filename");
    if (filename.empty()) {
        sendJson(res, false, 1001, "Missing 'filename' query parameter");
        return;
    }

    std::string source;
    const Result result = script_engine_->ReadScriptFile(filename, source);
    if (result != Result::NoError) {
        sendJson(res, false, resultCode(result),
                 "script file unavailable: " + filename);
        return;
    }

    setCorsHeaders(res);
    res.set_content(source, "text/plain");
}

void RobotHttpServer::handleScriptBreakpointAdd(const httplib::Request& req,
                                                httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("filename") ||
        !body["filename"].is_string() || !body.contains("line") ||
        !body["line"].is_number_integer()) {
        sendJson(res, false, 1001,
                 "Invalid JSON or missing 'filename'/'line' fields");
        return;
    }

    const Result result = script_engine_->AddBreakpoint(
        body["filename"].get<std::string>(), body["line"].get<int>());
    sendScriptResult(res, result, "breakpoint added");
}

void RobotHttpServer::handleScriptBreakpointRemove(const httplib::Request& req,
                                                   httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }

    const nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("filename") ||
        !body["filename"].is_string() || !body.contains("line") ||
        !body["line"].is_number_integer()) {
        sendJson(res, false, 1001,
                 "Invalid JSON or missing 'filename'/'line' fields");
        return;
    }

    const Result result = script_engine_->RemoveBreakpoint(
        body["filename"].get<std::string>(), body["line"].get<int>());
    sendScriptResult(res, result, "breakpoint removed");
}

void RobotHttpServer::handleScriptBreakpointClear(const httplib::Request& req,
                                                  httplib::Response& res) {
    if (!ensureControl(req, res)) return;
    if (script_engine_ == nullptr) {
        sendJson(res, false, resultCode(Result::FunctionNotSupported),
                 "Lua script engine is unavailable");
        return;
    }
    sendScriptResult(res, script_engine_->ClearBreakpoints(),
                     "breakpoints cleared");
}


// ============================================================================
// Jogging Compatibility Handler — 旧 flag 格式转换
// ============================================================================

void RobotHttpServer::handleJogCompat(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog (compat flag mode)");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("flag") || !body["flag"].is_string() ||
        !body.contains("direction") || !body["direction"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'flag'/'direction' fields");
        return;
    }

    const std::string flag = body["flag"].get<std::string>();
    const std::string direction = body["direction"].get<std::string>();
    const double speed = jsonNumberOr(body, "max_speed", "speed", 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);

    double sign = 0.0;
    if (!directionSign(direction, sign)) {
        sendJson(res, false, 1001, "Invalid direction");
        return;
    }

    Result result = Result::IllegalParameter;
    const int joint_count = robot_->getJointNum();

    if (sign == 0.0 || flag == "NONE") {
        result = robot_->StopMotion();
    } else if (flag.size() == 2 && flag[0] == 'J' && flag[1] >= '0' && flag[1] <= '9') {
        // 关节点动 flag → 向量
        const int joint_index = flag[1] - '0';
        if (joint_index >= joint_count) {
            sendJson(res, false, 1003,
                     "Joint index out of range: expected 0.." + std::to_string(joint_count - 1));
            return;
        }
        JntArray joint_direction(static_cast<unsigned int>(joint_count));
        joint_direction(static_cast<unsigned int>(joint_index)) = sign;
        result = robot_->MoveJogging(joint_direction, speed, timeout, dir_threshold);
    } else if (flag == "NULLSPACE") {
        // 零空间点动 flag → 向量
        const auto vector_json = body.contains("joints") ? body["joints"]
            : body.value("direction_vector", nlohmann::json::array());
        if (!vector_json.is_array() || static_cast<int>(vector_json.size()) != joint_count) {
            sendJson(res, false, 1001,
                     "NULLSPACE requires 'joints' array with " + std::to_string(joint_count) + " elements");
            return;
        }
        JntArray intent_direction(static_cast<unsigned int>(joint_count));
        for (int i = 0; i < joint_count; ++i) {
            if (!vector_json[i].is_number() || !isFinite(vector_json[i].get<double>())) {
                sendJson(res, false, 1001, "Invalid NULLSPACE direction values");
                return;
            }
            intent_direction(static_cast<unsigned int>(i)) = sign * vector_json[i].get<double>();
        }
        result = robot_->MoveNullJogging(intent_direction, speed, timeout, dir_threshold);
    } else {
        // 笛卡尔点动 flag → Twist
        const auto separator = flag.rfind('_');
        if (separator == std::string::npos) {
            sendJson(res, false, 1001, "Invalid jogging flag: " + flag);
            return;
        }
        const std::string frame = flag.substr(0, separator);
        const std::string axis = flag.substr(separator + 1);
        if (frame != "TOOL" && frame != "FLANGE" && frame != "OBJECT" && frame != "BASE") {
            sendJson(res, false, 1001, "Invalid frame in flag: " + flag);
            return;
        }

        Twist twist;
        if (axis == "X") twist.vel.x(sign);
        else if (axis == "Y") twist.vel.y(sign);
        else if (axis == "Z") twist.vel.z(sign);
        else if (axis == "ROLL") twist.rot.x(sign);
        else if (axis == "PITCH") twist.rot.y(sign);
        else if (axis == "YAW") twist.rot.z(sign);
        else {
            sendJson(res, false, 1001, "Invalid axis in flag: " + flag);
            return;
        }

        result = robot_->MoveJogging(twist, speed, timeout, dir_threshold);
    }

    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Dragging started" : "Dragging start failed",
             data);
}

// ============================================================================
// Jogging Handlers
// ============================================================================

void RobotHttpServer::handleJogJoint(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/joint");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("joints") || !body["joints"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'joints' array");
        return;
    }

    const int joint_count = robot_->getJointNum();
    const auto& joints_json = body["joints"];
    if (static_cast<int>(joints_json.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray joint_direction(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!joints_json[i].is_number()) {
            sendJson(res, false, 1001, "Joint direction values must be numbers");
            return;
        }
        const double value = joints_json[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Joint direction values must be finite");
            return;
        }
        joint_direction(static_cast<unsigned int>(i)) = value;
    }

    double sign = 1.0;
    if (body.contains("direction") && body["direction"].is_string()) {
        if (!directionSign(body["direction"].get<std::string>(), sign) || sign == 0.0) {
            sendJson(res, false, 1001, "Invalid direction");
            return;
        }
    }
    if (sign != 1.0) {
        for (unsigned int i = 0; i < joint_direction.rows(); ++i) {
            joint_direction(i) *= sign;
        }
    }

    const double speed = jsonNumberOr(body, "speed", nullptr, 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid speed, timeout or direction threshold");
        return;
    }

    const Result result = robot_->MoveJogging(joint_direction, speed, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Joint jogging started" : "Joint jogging failed",
             data);
}

void RobotHttpServer::handleJogCartesian(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/cartesian");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("twist") || !body["twist"].is_array()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'twist' array");
        return;
    }

    const auto& twist_json = body["twist"];
    if (twist_json.size() != 6) {
        sendJson(res, false, 1001, "Twist must have exactly 6 elements");
        return;
    }

    Twist twist;
    for (size_t i = 0; i < 6; ++i) {
        if (!twist_json[i].is_number()) {
            sendJson(res, false, 1001, "Twist values must be numbers");
            return;
        }
        const double value = twist_json[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "Twist values must be finite");
            return;
        }
        if (i == 0) twist.vel.x(value);
        else if (i == 1) twist.vel.y(value);
        else if (i == 2) twist.vel.z(value);
        else if (i == 3) twist.rot.x(value);
        else if (i == 4) twist.rot.y(value);
        else twist.rot.z(value);
    }

    double sign = 1.0;
    if (body.contains("direction") && body["direction"].is_string()) {
        if (!directionSign(body["direction"].get<std::string>(), sign) || sign == 0.0) {
            sendJson(res, false, 1001, "Invalid direction");
            return;
        }
    }
	    if (sign != 1.0) {
	        twist.vel = twist.vel * sign;
	        twist.rot = twist.rot * sign;
	    }

        Robot::JogFrame jog_frame = Robot::JogFrame::BASE;
        if (body.contains("frame")) {
            if (!body["frame"].is_string() ||
                !parseJogFrame(body["frame"].get<std::string>(), jog_frame)) {
                sendJson(res, false, 1001, "Invalid jog frame: expected BASE, FLANGE, TOOL or OBJECT");
                return;
            }
        }

    const double speed = jsonNumberOr(body, "speed", nullptr, 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
	        sendJson(res, false, 1001, "Invalid speed, timeout or direction threshold");
	        return;
    }

    const Result result = robot_->MoveJogging(twist, jog_frame, speed, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Cartesian jogging started" : "Cartesian jogging failed",
             data);
}

void RobotHttpServer::handleJogNullspace(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/nullspace");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }
    const auto vector_json = body.contains("joints") ? body["joints"] : body.value("direction_vector", nlohmann::json::array());
    if (!vector_json.is_array()) {
        sendJson(res, false, 1001, "NULLSPACE jogging requires 'joints' array");
        return;
    }

    double sign = 1.0;
    if (body.contains("direction")) {
        if (!body["direction"].is_string() ||
            !directionSign(body["direction"].get<std::string>(), sign) || sign == 0.0) {
            sendJson(res, false, 1001, "Invalid direction");
            return;
        }
    }

    const double speed = jsonNumberOr(body, "max_speed", "speed", 1.0);
    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(speed) || !isFinite(timeout) || !isFinite(dir_threshold) ||
        speed <= 0.0 || timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid speed, timeout or direction threshold");
        return;
    }

    const int joint_count = robot_->getJointNum();
    if (static_cast<int>(vector_json.size()) != joint_count) {
        sendJson(res, false, 1003,
                 "Joint count mismatch: expected " + std::to_string(joint_count));
        return;
    }

    JntArray intent_direction(static_cast<unsigned int>(joint_count));
    for (int i = 0; i < joint_count; ++i) {
        if (!vector_json[i].is_number()) {
            sendJson(res, false, 1001, "NULLSPACE direction values must be numbers");
            return;
        }
        const double value = vector_json[i].get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "NULLSPACE direction values must be finite");
            return;
        }
        intent_direction(static_cast<unsigned int>(i)) = sign * value;
    }

    const Result result = robot_->MoveNullJogging(intent_direction, speed, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Nullspace jogging started" : "Nullspace jogging failed",
             data);
}

void RobotHttpServer::handleJogSvd(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/svd");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded()) {
        sendJson(res, false, 1001, "Invalid JSON body");
        return;
    }
    const auto speeds_json = body.contains("dim_speeds") ? body["dim_speeds"] : body.value("speeds", nlohmann::json::array());
    if (!speeds_json.is_array() || speeds_json.empty()) {
        sendJson(res, false, 1001, "SVD jogging requires 'dim_speeds' array");
        return;
    }

    std::vector<double> dim_speeds;
    dim_speeds.reserve(speeds_json.size());
    for (const auto& value_json : speeds_json) {
        if (!value_json.is_number()) {
            sendJson(res, false, 1001, "SVD jogging speeds must be numbers");
            return;
        }
        const double value = value_json.get<double>();
        if (!isFinite(value)) {
            sendJson(res, false, 1001, "SVD jogging speeds must be finite");
            return;
        }
        dim_speeds.push_back(value);
    }

    const double timeout = jsonNumberOr(body, "timeout", nullptr, 0.1);
    const double dir_threshold = jsonNumberOr(body, "dir_threshold", nullptr, 0.99);
    if (!isFinite(timeout) || !isFinite(dir_threshold) ||
        timeout <= 0.0 || dir_threshold < -1.0 || dir_threshold > 1.0) {
        sendJson(res, false, 1001, "Invalid timeout or direction threshold");
        return;
    }

    const Result result = robot_->MoveSvdJogging(dim_speeds, timeout, dir_threshold);
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "SVD jogging started" : "SVD jogging failed",
             data);
}

void RobotHttpServer::handleJogStop(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/jog/stop");
    if (!ensureControl(req, res)) return;

    if (!robot_->IsMotionBusy()) {
        nlohmann::json data;
        data["robot_state"] = robot_->GetStateString();
        data["control_active"] = robot_->IsControlActive();
        sendJson(res, true, 0, "Jogging already stopped", data);
        return;
    }

    const Result result = robot_->StopMotion();
    nlohmann::json data;
    data["robot_state"] = robot_->GetStateString();
    data["control_active"] = robot_->IsControlActive();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Jogging stopped" : "Jogging stop failed",
             data);
}



// ============================================================================
// Frame Management Handlers
// ============================================================================

void RobotHttpServer::handleGetToolFrameNames(const httplib::Request&, httplib::Response& res) {
    if (!frame_registry_) { sendJson(res, false, -3001, "Tool registry is unavailable"); return; }
    const auto state = frame_registry_->snapshot();
    std::vector<std::string> names;
    for (const auto& tool : state->tools) names.push_back(tool.first);
    sendJson(res, true, 0, "ok", {{"names", names}, {"active", state->active_tool}, {"file_path", frame_registry_->path()},
        {"package_support", true}, {"robot_id", frame_registry_->robotId()}});
}

void RobotHttpServer::handleGetObjectFrameNames(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("GET /api/robot/object_frames");

    nlohmann::json data;
    data["names"] = robot_->GetObjectFrameNames();
    data["active"] = robot_->GetActiveObjectFrameName();
    sendJson(res, true, 0, "ok", data);
}

void RobotHttpServer::handleGetToolFrame(const httplib::Request& req, httplib::Response& res) {
    const auto name = req.get_param_value("name");
    if (name.empty()) { sendJson(res, false, 1001, "Missing 'name' query parameter"); return; }
    if (!frame_registry_) { sendJson(res, false, -3001, "Tool registry is unavailable"); return; }
    const auto state=frame_registry_->snapshot(); const auto found=state->tools.find(name);
    if (found==state->tools.end()) { sendJson(res,false,-3001,"Tool frame not found"); return; }
    nlohmann::json data{{"name",name},{"frame",frameToJson(found->second)}};
    if (state->packages.count(name)) data["package"]=toolPackageJson(state->packages.at(name));
    sendJson(res,true,0,"ok",data);
}

void RobotHttpServer::handleGetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    const std::string name = req.get_param_value("name");
    log_ptr_->info("GET /api/robot/object_frame name={}", name);
    if (name.empty()) {
        sendJson(res, false, 1001, "Missing 'name' query parameter");
        return;
    }

    Frame frame;
    const Result result = robot_->GetObjectFrame(name, frame);
    nlohmann::json data;
    data["name"] = name;
    if (resultSucceeded(result)) data["frame"] = frameToJson(frame);
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "ok" : "Object frame not found",
             data);
}

void RobotHttpServer::handleSetPoseFrame(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("index") || !body["index"].is_number_integer() ||
        body["index"].get<int64_t>() < 1 || body["index"].get<int64_t>() > 6 ||
        !body.contains("name") || !body["name"].is_string() ||
        !FrameRegistry::validName(body["name"].get<std::string>()) ||
        (body.contains("session_id") && !body["session_id"].is_string())) {
        sendJson(res, false, 1001, "Expected a valid tool name, index in [1,6] and optional session_id"); return;
    }
    const auto name = body["name"].get<std::string>();
    const auto id = body.value("session_id", std::string());
    const int index = body["index"].get<int>();
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    auto found = calibration_sessions_.find(id);
    if (!id.empty() && (found == calibration_sessions_.end() || found->second.owner_token != req.get_header_value(CONTROL_TOKEN_HEADER))) {
        sendJson(res, false, 4004, "Calibration session is unavailable for this control session; start again"); return;
    }
    if (id.empty() && index != 1) { sendJson(res, false, 4001, "Start a new calibration by capturing point 1"); return; }
    if (!id.empty() && found->second.name != name) { sendJson(res, false, 1001, "Tool name does not match calibration session"); return; }
    if (!id.empty() && !calibrationContextCurrent(found->second)) { sendJson(res,false,4004,"Tool installation changed; restart calibration"); return; }
    ToolCalibrationSample sample;
    const auto result = calibration_capture_(sample);
    if (result != Result::NoError || !FrameRegistry::validFrame(sample.flange) || sample.joints.empty() ||
        !std::isfinite(sample.timestamp) || sample.timestamp <= 0 ||
        !std::all_of(sample.joints.begin(), sample.joints.end(), [](double value) { return std::isfinite(value); })) {
        sendJson(res, false, 4005, "Cannot capture a valid stationary flange pose; stop motion and check controller state"); return;
    }
    if (id.empty()) {
        if (calibration_sessions_.size() >= 16) {
            auto oldest = std::min_element(calibration_sessions_.begin(), calibration_sessions_.end(),
                [](const auto& a, const auto& b) { return a.second.created < b.second.created; });
            calibration_sessions_.erase(oldest);
        }
        CalibrationSession session;
        session.id = "cal-" + generateControlToken(); session.name = name;
        session.owner_token = req.get_header_value(CONTROL_TOKEN_HEADER);
        session.created = ++calibration_counter_;
        const auto frames=frame_registry_->snapshot();
        const auto tool=frames->packages.find(name);
        if (tool!=frames->packages.end()) { session.package_digest=tool->second.digest; session.package_revision=tool->second.revision; }
        found = calibration_sessions_.emplace(session.id, std::move(session)).first;
    }
    found->second.samples[index - 1] = std::move(sample);
    found->second.result.reset();
    sendJson(res, true, 0, "Calibration point captured", calibrationSessionJson(found->second));
}

void RobotHttpServer::handleSetToolFrame(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    std::string name;
    Frame frame;
    try {
        if (!body.is_object() || !parseNamedFrameBody(body, name, frame) ||
            (body.contains("persist") && !body["persist"].is_boolean()) ||
            (body.contains("previous_name") && !body["previous_name"].is_string())) {
            sendJson(res, false, 1001, "Expected name, a valid frame, and optional boolean persist"); return;
        }
    } catch (const nlohmann::json::exception&) {
        sendJson(res, false, 1001, "Invalid tool pose values"); return;
    }
    const bool persist = body.value("persist", false);
    const auto result = frame_registry_->set(name, frame, true, false, persist, body.value("previous_name", std::string()));
    sendJson(res, resultSucceeded(result), resultCode(result), resultSucceeded(result) ? "Tool frame saved" : "Tool save failed; previous registry retained",
             {{"name", name}, {"frame", frameToJson(frame)}, {"persisted", resultSucceeded(result) && persist}});
}

void RobotHttpServer::handleSetObjectFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/object_frame");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    std::string name;
    Frame frame;
    if (!parseNamedFrameBody(body, name, frame)) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'name' and 'frame' fields");
        return;
    }

    const Result result = robot_->SetObjectFrame(name, frame);
    nlohmann::json data;
    data["name"] = name;
    data["frame"] = frameToJson(frame);
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Object frame set" : "Object frame set failed",
             data);
}

void RobotHttpServer::handleRemoveToolFrame(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto name = req.get_param_value("name"), persist_text = req.get_param_value("persist");
    if (name.empty() || (!persist_text.empty() && persist_text != "true" && persist_text != "false")) {
        sendJson(res, false, 1001, "Expected name and optional true/false persist query"); return;
    }
    const bool persist = persist_text == "true";
    const auto result = frame_registry_->remove(name, true, persist);
    sendJson(res, resultSucceeded(result), resultCode(result), resultSucceeded(result) ? "Tool frame removed" : "Tool removal failed; previous registry retained",
             {{"name", name}, {"active", frame_registry_->snapshot()->active_tool}, {"persisted", resultSucceeded(result) && persist}});
}

void RobotHttpServer::handleRemoveObjectFrame(const httplib::Request& req, httplib::Response& res) {
    const std::string name = req.get_param_value("name");
    log_ptr_->info("DELETE /api/robot/object_frame name={}", name);
    if (!ensureControl(req, res)) return;
    if (name.empty()) {
        sendJson(res, false, 1001, "Missing 'name' query parameter");
        return;
    }

    const Result result = robot_->RemoveObjectFrame(name);
    nlohmann::json data;
    data["name"] = name;
    data["active"] = robot_->GetActiveObjectFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Object frame removed" : "Object frame remove failed",
             data);
}

void RobotHttpServer::handleSetActiveToolFrame(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("name") || !body["name"].is_string() ||
        (body.contains("persist") && !body["persist"].is_boolean())) {
        sendJson(res, false, 1001, "Expected name and optional boolean persist"); return;
    }
    const bool persist = body.value("persist", false);
    const auto result = frame_registry_->activate(body["name"].get<std::string>(), true, persist);
    sendJson(res, resultSucceeded(result), resultCode(result), resultSucceeded(result) ? "Active tool frame set" : "Tool activation failed; previous registry retained",
             {{"active", frame_registry_->snapshot()->active_tool}, {"persisted", resultSucceeded(result) && persist}});
}

void RobotHttpServer::handleSetActiveObjectFrame(const httplib::Request& req, httplib::Response& res) {
    log_ptr_->info("POST /api/robot/active_object_frame");
    if (!ensureControl(req, res)) return;

    nlohmann::json body = nlohmann::json::parse(req.body, nullptr, false);
    if (body.is_discarded() || !body.contains("name") || !body["name"].is_string()) {
        sendJson(res, false, 1001, "Invalid JSON or missing 'name' field");
        return;
    }

    const std::string name = body["name"].get<std::string>();
    const Result result = robot_->SetActiveObjectFrame(name);
    nlohmann::json data;
    data["active"] = robot_->GetActiveObjectFrameName();
    sendJson(res, resultSucceeded(result), resultCode(result),
             resultSucceeded(result) ? "Active object frame set" : "Active object frame set failed",
             data);
}

void RobotHttpServer::handleLoadFrames(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto body = nlohmann::json::parse(req.body.empty() ? "{}" : req.body, nullptr, false);
    if (!body.is_object() || (body.contains("path") && !body["path"].is_string())) {
        sendJson(res, false, 1001, "Expected an optional string path"); return;
    }
    const auto path = body.value("path", std::string());
    const auto result = frame_registry_->load(path);
    const auto state = frame_registry_->snapshot();
    sendJson(res, resultSucceeded(result), resultCode(result), resultSucceeded(result) ? "Frames loaded" : "Frames load failed",
             {{"path", path.empty() ? frame_registry_->path() : path}, {"tool_frames", frame_registry_->names(true)},
               {"object_frames", frame_registry_->names(false)}, {"active_tool_frame", state->active_tool}, {"active_object_frame", state->active_object}});
}

void RobotHttpServer::handleSaveFrames(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto body = nlohmann::json::parse(req.body.empty() ? "{}" : req.body, nullptr, false);
    if (!body.is_object() || (body.contains("path") && !body["path"].is_string())) {
        sendJson(res, false, 1001, "Expected an optional string path"); return;
    }
    const auto path = body.value("path", std::string());
    const auto result = frame_registry_->save(path);
    const auto state = frame_registry_->snapshot();
    sendJson(res, resultSucceeded(result), resultCode(result), resultSucceeded(result) ? "Frames saved" : "Frames save failed",
             {{"path", path.empty() ? frame_registry_->path() : path}, {"tool_frames", frame_registry_->names(true)},
               {"object_frames", frame_registry_->names(false)}, {"active_tool_frame", state->active_tool}, {"active_object_frame", state->active_object}});
}

void RobotHttpServer::handleCalibrationRun(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (!prepareToolMutation(req, res, state_lock)) return;
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    if (!body.is_object() || !body.contains("session_id") || !body["session_id"].is_string() ||
        (body.contains("max_error") && !body["max_error"].is_number())) {
        sendJson(res, false, 1001, "Expected session_id and optional max_error in metres"); return;
    }
    const double tolerance = body.value("max_error", 0.001);
    if (!std::isfinite(tolerance) || tolerance <= 0 || tolerance > 0.05) {
        sendJson(res, false, 1001, "max_error must be in (0, 0.05] m"); return;
    }
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    auto found = calibration_sessions_.find(body["session_id"].get<std::string>());
    if (found == calibration_sessions_.end() || found->second.owner_token != req.get_header_value(CONTROL_TOKEN_HEADER)) {
        sendJson(res, false, 4004, "Calibration session is unavailable for this control session; start again"); return;
    }
    auto& session = found->second;
    session.result.reset();
    if (!calibrationContextCurrent(session)) { sendJson(res,false,4004,"Tool installation changed; restart calibration"); return; }
    std::array<KDL::Frame, 6> poses;
    for (size_t i = 0; i < poses.size(); ++i) {
        if (!session.samples[i]) { sendJson(res, false, 4001, "Capture all six points before calculating"); return; }
        poses[i] = session.samples[i]->flange;
    }
    ToolCalibrationSolution solution;
    std::string error;
    if (!solveSixPointTool(poses, tolerance, solution, error)) {
        sendJson(res, false, 4002, error, calibrationSessionJson(session)); return;
    }
    session.result = solution;
    sendJson(res, true, 0, "Tool calibration calculated; save the result to a tool explicitly", calibrationSessionJson(session));
}

void RobotHttpServer::handleCalibrationResult(const httplib::Request& req, httplib::Response& res) {
    const auto id = req.get_param_value("session_id");
    if (id.empty()) { sendJson(res, false, 1001, "Missing session_id query parameter"); return; }
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    const auto found = calibration_sessions_.find(id);
    if (found == calibration_sessions_.end()) { sendJson(res, false, 4004, "Calibration session not found"); return; }
    sendJson(res, true, 0, "ok", calibrationSessionJson(found->second));
}

bool RobotHttpServer::prepareToolMutation(const httplib::Request& req, httplib::Response& res, std::unique_lock<std::recursive_mutex>& state_lock) {
    if (!ensureControl(req, res)) return false;
    if (!frame_registry_) { sendJson(res, false, -3001, "Tool registry is unavailable"); return false; }
    if (robot_) state_lock = robot_->LockToolState();
    if ((robot_ && !robot_->CanChangeToolState()) || (tool_motion_busy_ && tool_motion_busy_())) {
        sendJson(res, false, 4005, "Stop robot motion before changing tools or capturing calibration points"); return false;
    }
    return true;
}

nlohmann::json RobotHttpServer::calibrationSessionJson(const CalibrationSession& session) {
    nlohmann::json samples = nlohmann::json::array();
    size_t captured = 0;
    for (size_t i = 0; i < session.samples.size(); ++i) {
        if (!session.samples[i]) { samples.push_back(nullptr); continue; }
        ++captured;
        const auto& sample = *session.samples[i];
        samples.push_back({{"index", i + 1}, {"flange", frameToJson(sample.flange)},
                           {"joints", sample.joints}, {"timestamp", sample.timestamp}});
    }
    nlohmann::json result;
    if (session.result) {
        const auto& value = *session.result;
        result = {{"pose", frameToJson(value.frame)}, {"error_state", false},
                  {"metrics", {{"rms_error", value.rms_error}, {"max_error", value.max_error},
                               {"condition_number", value.condition_number}, {"axis_angle_degrees", value.axis_angle_degrees}}}};
    }
    return {{"session_id", session.id}, {"name", session.name}, {"samples", samples}, {"package_digest",session.package_digest}, {"package_revision",session.package_revision},
            {"captured", captured}, {"result", result}};
}

bool RobotHttpServer::calibrationContextCurrent(const CalibrationSession& session) const {
    const auto state=frame_registry_->snapshot(); const auto found=state->packages.find(session.name);
    if (session.package_digest.empty()) return found==state->packages.end();
    return found!=state->packages.end() && found->second.digest==session.package_digest && found->second.revision==session.package_revision;
}

}  // namespace rocos
