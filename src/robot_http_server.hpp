// Copyright 2021, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// HTTP REST API Server for ROCOS-App Robot Controller
// Replaces gRPC with HTTP/JSON using cpp-httplib

#pragma once

#include <httplib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <json.hpp>
#include "types.hpp"
#include <unordered_map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "logger.hpp"
#include "robot.hpp"
#include "tool_calibration.hpp"
#include <optional>

namespace rocos {

class LuaScriptEngine;

class RobotHttpServer {
public:
    explicit RobotHttpServer(Robot* robot, LuaScriptEngine* script_engine = nullptr, int control_ttl_seconds = 300,
                             const std::string& web_root = "");
    ~RobotHttpServer();

    /// Start server (blocking)
    void run(const std::string& host = "0.0.0.0", int port = 8080);

    /// Start server in a separate thread (non-blocking)
    void runAsync(const std::string& host = "0.0.0.0", int port = 8080);

    /// Stop the server
    void stop();

private:
    friend struct RobotHttpServerTestAccess;
    void registerRoutes();

    // ---- Robot State ----
    void handleGetRobotState(const httplib::Request& req, httplib::Response& res);
    void handleGetTcpPose(const httplib::Request& req, httplib::Response& res);
    void handleGetRobotInfo(const httplib::Request& req, httplib::Response& res);
    void handleGetRobotModel(const httplib::Request& req, httplib::Response& res);
    void handleGetLinkMesh(const httplib::Request& req, httplib::Response& res);
    void handleGetImpedance(const httplib::Request& req, httplib::Response& res);

    // ---- Robot Control ----
    void handleEnable(const httplib::Request& req, httplib::Response& res);
    void handleDisable(const httplib::Request& req, httplib::Response& res);
    void handleReset(const httplib::Request& req, httplib::Response& res);
    void handleIsEnabled(const httplib::Request& req, httplib::Response& res);
    void handleSetWorkMode(const httplib::Request& req, httplib::Response& res);
    void handleSetLoadParameters(const httplib::Request& req, httplib::Response& res);
    void handleGetLoadParameters(const httplib::Request& req, httplib::Response& res);
    void handleSaveLoadParameters(const httplib::Request& req, httplib::Response& res);

    // ---- Servo Control ----
    void handleServoStart(const httplib::Request& req, httplib::Response& res);
    void handleServoStop(const httplib::Request& req, httplib::Response& res);
    void handleServoMode(const httplib::Request& req, httplib::Response& res);

    // ---- Motion Control ----
    void handleMoveJ(const httplib::Request& req, httplib::Response& res);
    void handleMoveJ_IK(const httplib::Request& req, httplib::Response& res);
    void handleMoveL(const httplib::Request& req, httplib::Response& res);
    void handleMoveL_FK(const httplib::Request& req, httplib::Response& res);
    void handleMoveC(const httplib::Request& req, httplib::Response& res);

    void handlePause(const httplib::Request& req, httplib::Response& res);
    void handleResume(const httplib::Request& req, httplib::Response& res);
    void handleStop(const httplib::Request& req, httplib::Response& res);
    void handleWaitMove(const httplib::Request& req, httplib::Response& res);

    // ---- Jogging ----
    void handleJogJoint(const httplib::Request& req, httplib::Response& res);
    void handleJogCartesian(const httplib::Request& req, httplib::Response& res);
    void handleJogNullspace(const httplib::Request& req, httplib::Response& res);
    void handleJogSvd(const httplib::Request& req, httplib::Response& res);
    void handleJogStop(const httplib::Request& req, httplib::Response& res);
    void handleJogCompat(const httplib::Request& req, httplib::Response& res);  // 旧 flag 兼容

    // ---- Async Task Query ----
    void handleMoveStatus(const httplib::Request& req, httplib::Response& res);

    // ---- Frame Management ----
    void handleGetToolFrameNames(const httplib::Request& req, httplib::Response& res);
    void handleGetObjectFrameNames(const httplib::Request& req, httplib::Response& res);
    void handleGetToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleGetObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetPoseFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleRemoveToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleRemoveObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetActiveToolFrame(const httplib::Request& req, httplib::Response& res);
    void handleSetActiveObjectFrame(const httplib::Request& req, httplib::Response& res);
    void handleLoadFrames(const httplib::Request& req, httplib::Response& res);
    void handleSaveFrames(const httplib::Request& req, httplib::Response& res);
    void handleToolPackages(const httplib::Request& req, httplib::Response& res);

    // ---- Calibration ----
    void handleCalibrationRun(const httplib::Request& req, httplib::Response& res);
    void handleCalibrationResult(const httplib::Request& req, httplib::Response& res);
    struct CalibrationSession {
        std::string id, name, owner_token;
        std::array<std::optional<ToolCalibrationSample>, 6> samples;
        std::optional<ToolCalibrationSolution> result;
        uint64_t created{0};
        std::string package_digest;
        uint64_t package_revision{0};
    };
    nlohmann::json calibrationSessionJson(const CalibrationSession& session);
    bool calibrationContextCurrent(const CalibrationSession& session) const;
    bool prepareToolMutation(const httplib::Request& req, httplib::Response& res, std::unique_lock<std::recursive_mutex>& state_lock);

    // ---- Lua Script ----
    void handleScriptUpload(const httplib::Request& req, httplib::Response& res);
    void handleScriptRun(const httplib::Request& req, httplib::Response& res);
    void handleScriptPause(const httplib::Request& req, httplib::Response& res);
    void handleScriptResume(const httplib::Request& req, httplib::Response& res);
    void handleScriptStop(const httplib::Request& req, httplib::Response& res);
    void handleScriptStep(const httplib::Request& req, httplib::Response& res);
    void handleScriptStatus(const httplib::Request& req, httplib::Response& res);
    void handleScriptList(const httplib::Request& req, httplib::Response& res);
    void handleScriptDownload(const httplib::Request& req, httplib::Response& res);
    void handleScriptBreakpointAdd(const httplib::Request& req, httplib::Response& res);
    void handleScriptBreakpointRemove(const httplib::Request& req, httplib::Response& res);
    void handleScriptBreakpointClear(const httplib::Request& req, httplib::Response& res);

    // ---- Control Rights (single-holder lock) ----
    void handleControlAcquire(const httplib::Request& req, httplib::Response& res);
    void handleControlRelease(const httplib::Request& req, httplib::Response& res);
    void handleControlTakeover(const httplib::Request& req, httplib::Response& res);
    void handleControlStatus(const httplib::Request& req, httplib::Response& res);
    void handleControlHeartbeat(const httplib::Request& req, httplib::Response& res);

    // ---- Utility ----
    void sendJson(httplib::Response& res, bool success,
                  int businessCode, const std::string& message,
                  const nlohmann::json& data = nlohmann::json());
    void sendScriptResult(httplib::Response& res, Result result,
                          const std::string& success_message);
    nlohmann::json scriptStatusToJson() const;

    rocos::Frame jsonToFrame(const nlohmann::json& j);

    rocos::Frame jsonToFrame(const nlohmann::json& pos, const nlohmann::json& rot);
    nlohmann::json frameToJson(const rocos::Frame& frame);
    std::string getUrdfPath();

    // ---- Async Task Management ----
    std::string generateTaskId();
    void registerTask(const std::string& taskId, const std::string& type);
    void updateTaskStatus(const std::string& taskId, const std::string& status,
                          int result = 0, const std::string& message = "ok");
    nlohmann::json getTaskInfo(const std::string& taskId);
    void cleanExpiredTasks();

    // ---- CORS & Preflight ----
    void setCorsHeaders(httplib::Response& res);

    // ---- Control Rights internals ----
    enum class AuthResult { OK, MissingToken, InvalidToken, Expired };
    static std::string makeConnectionKey(const std::string& remote_addr, int remote_port);
    static std::string makeConnectionKey(const httplib::Request& req);
    static std::string extractClientId(const httplib::Request& req);
    // 调用方必须持有 control_mutex_；校验身份和有效期后续期。
    AuthResult checkAndRenewToken_locked(const httplib::Request& req, bool allow_debug = false);
    // 写接口入口守卫：失败时已经写好响应，调用方应直接 return。
    bool ensureControl(const httplib::Request& req, httplib::Response& res, bool allow_debug = true,
                       nlohmann::json* owner = nullptr);
    std::string generateControlToken();
    nlohmann::json controlOwnerJson_locked() const;
    void clearControl_locked(const char* reason);

    // ---- Thread Pool ----
    void submitTask(std::function<void()> func);

    Robot* robot_;
    LuaScriptEngine* script_engine_;
    std::unique_ptr<httplib::Server> server_;
    std::unique_ptr<std::thread> thread_;
    std::mutex taskMutex_;
    std::unordered_map<std::string, nlohmann::json> taskMap_;
    long long taskCounter_;

    // Thread pool for async motion tasks
    std::vector<std::thread> workerThreads_;
    std::queue<std::function<void()>> taskQueue_;
    std::mutex poolMutex_;
    std::condition_variable poolCv_;
    bool poolShutdown_;
    std::atomic<int> activeWorkers_;
    static const int MAX_WORKERS = 8;

    Logger::logger_ptr log_ptr_ = nullptr;

    // Task TTL
    static const int TASK_TTL_SECONDS = 3600; // 1 hour

    // ---- Control Rights state ----
    mutable std::mutex control_mutex_;
    std::string current_token_;        // 空串 = 无持有者
    std::string current_owner_ip_;
    int current_owner_port_ = -1;
    std::string current_owner_connection_;
    std::string current_owner_client_id_;
    std::string current_owner_name_;
    std::string current_owner_agent_;
    std::chrono::steady_clock::time_point acquired_at_{};
    std::chrono::steady_clock::time_point last_seen_at_{};
    const int control_ttl_seconds_;
    const std::string web_root_;
    FrameRegistry* frame_registry_{nullptr};
    std::function<Robot::RobotStateSnapshot()> state_snapshot_;
    std::function<Result(Frame&, const std::string&)> tcp_pose_;
    std::function<bool()> tool_motion_busy_;
    std::function<Result(ToolCalibrationSample&)> calibration_capture_;
    std::mutex calibration_mutex_;
    std::unordered_map<std::string, CalibrationSession> calibration_sessions_;
    uint64_t calibration_counter_{0};
    static constexpr const char* CONTROL_TOKEN_HEADER = "X-Rocos-Control-Token";
    static constexpr const char* CONTROL_CLIENT_ID_HEADER = "X-Rocos-Client-Id";

    // 调试旁路：使用预设 token + client_id 可绕过 acquire 流程直接控制机器人
    // curl 示例: curl -X POST ... -H "X-Rocos-Control-Token: debug-token" -H "X-Rocos-Client-Id: debug-client"
    static constexpr const char* DEBUG_TOKEN = "debug-token";
    static constexpr const char* DEBUG_CLIENT_ID = "debug-client";
};

} // namespace rocos
