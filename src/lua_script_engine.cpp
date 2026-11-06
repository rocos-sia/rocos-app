#include "lua_script_engine.hpp"

#include <sol/sol.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>

#include "logger.hpp"
#include "robot.hpp"

namespace rocos {
namespace {

constexpr const char* SCRIPT_STOP_MARKER = "__ROCOS_SCRIPT_STOPPED__";

std::string normalizeFilename(const std::string& filename) {
    return std::filesystem::path(filename).lexically_normal().generic_string();
}

bool pathStartsWith(const std::filesystem::path& path,
                    const std::filesystem::path& root) {
    auto path_it_ = path.begin();
    auto root_it_ = root.begin();
    for (; root_it_ != root.end(); ++root_it_, ++path_it_) {
        if (path_it_ == path.end() || *path_it_ != *root_it_) {
            return false;
        }
    }
    return true;
}

}  // namespace

struct LuaScriptEngine::Impl {
    struct Breakpoint {
        std::string filename;
        int line{0};

        bool operator<(const Breakpoint& rhs) const noexcept {
            return std::tie(filename, line) <
                   std::tie(rhs.filename, rhs.line);
        }
    };

    struct Pose {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double qx{0.0};
        double qy{0.0};
        double qz{0.0};
        double qw{1.0};
    };

    Impl(Robot& robot, std::filesystem::path scripts_root)
        : robot_(robot),
          scripts_root_(std::filesystem::absolute(
              std::move(scripts_root)).lexically_normal()),
          log_ptr_(Logger::getInstance("LuaScriptEngine")) {}

    ~Impl() {
        Stop();
        joinWorker();
    }

    Result LoadSource(const std::string& source, const std::string& filename) {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        if (source.empty() || filename.empty()) {
            return Result::IllegalParameter;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ == State::Running ||
                state_ == State::Pausing ||
                state_ == State::Paused ||
                state_ == State::Stopping) {
                return Result::LuaStateConflict;
            }
        }
        joinWorker();

        std::lock_guard<std::mutex> lock(mutex_);
        initializeLua();
        filename_ = normalizeFilename(filename);
        source_ = source;
        location_ = SourceLocation{filename_, 0};
        error_.clear();
        stop_requested_ = false;
        pause_requested_ = false;
        step_mode_ = false;
        step_started_ = false;
        motion_active_ = false;

        sol::load_result load_result_ =
            lua_->load(source_, "@" + filename_);
        if (!load_result_.valid()) {
            const sol::error error_ = load_result_;
            state_ = State::Failed;
            this->error_ = error_.what();
            Logger::logCode(LogLevel::Error, "LuaScriptEngine", static_cast<int>(Result::LuaExecutionError), "Lua script compilation failed: {}", this->error_);
            return Result::LuaExecutionError;
        }

        function_ = load_result_;
        ++script_counter_;
        script_id_ = "script_" + std::to_string(script_counter_);
        state_ = State::Loaded;
        return Result::NoError;
    }

    // 将相对路径解析为 scripts_root_ 下的规范化绝对路径，并做穿越防护。
    // 成功返回 true 并填充 resolved_path / display_path。
    bool resolveScriptPath(const std::filesystem::path& relative_path,
                           std::filesystem::path& resolved_path,
                           std::filesystem::path& display_path) const {
        if (relative_path.empty() || relative_path.is_absolute()) {
            return false;
        }
        std::error_code error_code_;
        const auto root_ =
            std::filesystem::weakly_canonical(scripts_root_, error_code_);
        if (error_code_) {
            return false;
        }
        resolved_path = std::filesystem::canonical(root_ / relative_path, error_code_);
        if (error_code_ || !pathStartsWith(resolved_path, root_)) {
            return false;
        }
        display_path = std::filesystem::relative(resolved_path, root_, error_code_);
        return !static_cast<bool>(error_code_);
    }

    Result LoadFile(const std::filesystem::path& relative_path) {
        std::filesystem::path script_path_;
        std::filesystem::path display_path_;
        if (!resolveScriptPath(relative_path, script_path_, display_path_)) {
            return Result::LuaFileError;
        }

        std::ifstream stream_(script_path_);
        if (!stream_.is_open()) {
            Logger::logCode(LogLevel::Error, "LuaScriptEngine", static_cast<int>(Result::LuaFileError), "Cannot open Lua script file: {}", script_path_.string());
            return Result::LuaFileError;
        }

        std::ostringstream buffer_;
        buffer_ << stream_.rdbuf();
        if (stream_.bad()) {
            Logger::logCode(LogLevel::Error, "LuaScriptEngine", static_cast<int>(Result::LuaFileError), "Lua script file read failed: {}", script_path_.string());
            return Result::LuaFileError;
        }
        return LoadSource(buffer_.str(), display_path_.generic_string());
    }

    // 只读脚本文件内容（不加载进 Lua VM），供脚本下载接口使用
    Result ReadScriptFile(const std::filesystem::path& relative_path,
                          std::string& source) const {
        std::filesystem::path script_path_;
        std::filesystem::path display_path_;
        if (!resolveScriptPath(relative_path, script_path_, display_path_)) {
            return Result::LuaFileError;
        }

        std::ifstream stream_(script_path_);
        if (!stream_.is_open()) {
            return Result::LuaFileError;
        }

        std::ostringstream buffer_;
        buffer_ << stream_.rdbuf();
        if (stream_.bad()) {
            return Result::LuaFileError;
        }
        source = buffer_.str();
        return Result::NoError;
    }

    // 列出 scripts_root_ 下所有 .lua 脚本（相对路径，按字典序）
    std::vector<std::string> ListScripts() const {
        std::vector<std::string> scripts;
        std::error_code error_code_;
        const auto root_ =
            std::filesystem::weakly_canonical(scripts_root_, error_code_);
        if (error_code_) {
            return scripts;
        }

        std::filesystem::recursive_directory_iterator it(root_, error_code_);
        const std::filesystem::recursive_directory_iterator end_;
        for (; it != end_; it.increment(error_code_)) {
            if (error_code_) {
                break;
            }
            if (!it->is_regular_file(error_code_) ||
                it->path().extension() != ".lua") {
                continue;
            }
            const auto rel_ =
                std::filesystem::relative(it->path(), root_, error_code_);
            if (!error_code_) {
                scripts.push_back(rel_.generic_string());
            }
        }
        std::sort(scripts.begin(), scripts.end());
        return scripts;
    }

    Result Run() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        auto command_lock = robot_.LockToolState();
        if (robot_.IsRciEnabled()) return Result::ConflictTaskRunning;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Loaded) {
                return Result::LuaStateConflict;
            }
            if (robot_.script_active_) return Result::ConflictTaskRunning;
            state_ = State::Running;
            stop_requested_ = false;
            pause_requested_ = false;
            step_mode_ = false;
            step_started_ = false;
        }

        robot_.script_active_ = true;
        try {
            worker_ = std::thread(&Impl::execute, this);
        } catch (const std::system_error& exception_) {
            robot_.script_active_ = false;
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::Loaded;
            error_ = exception_.what();
            Logger::logCode(LogLevel::Error, "LuaScriptEngine", static_cast<int>(Result::ResourceUnavailable), "Cannot start Lua worker thread: {}", this->error_);
            return Result::ResourceUnavailable;
        }
        return Result::NoError;
    }

    Result Pause() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Running) {
                return Result::LuaStateConflict;
            }
            pause_requested_ = true;
            state_ = State::Pausing;
            motion_active_ = this->motion_active_;
        }

        if (motion_active_) {
            const Result result_ = robot_.PauseMotion();
            if (result_ != Result::NoError) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (state_ == State::Pausing) {
                    pause_requested_ = false;
                    state_ = State::Running;
                }
                return result_;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::Paused;
        }
        return Result::NoError;
    }

    Result Resume() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Paused) {
                return Result::LuaStateConflict;
            }
            motion_active_ = this->motion_active_;
        }

        if (motion_active_) {
            const Result result_ = robot_.ResumeMotion();
            if (result_ != Result::NoError) {
                return result_;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pause_requested_ = false;
            step_mode_ = false;
            step_started_ = false;
            state_ = State::Running;
        }
        control_cv_.notify_all();
        return Result::NoError;
    }

    Result Stop() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_;
        bool pause_requested_;
        State previous_state_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ == State::Empty ||
                state_ == State::Completed ||
                state_ == State::Failed ||
                state_ == State::Stopped) {
                return Result::NoError;
            }
            if (state_ == State::Loaded) {
                state_ = State::Stopped;
                return Result::NoError;
            }
            motion_active_ = this->motion_active_;
            pause_requested_ = this->pause_requested_;
            previous_state_ = state_;
            state_ = State::Stopping;
            stop_requested_ = true;
            this->pause_requested_ = false;
        }
        control_cv_.notify_all();

        if (motion_active_) {
            const Result result_ = robot_.StopMotion();
            if (result_ != Result::NoError &&
                robot_.GetStateString() != "STOPPED") {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = previous_state_;
                stop_requested_ = false;
                this->pause_requested_ = pause_requested_;
                return result_;
            }
        }
        return Result::NoError;
    }

    Result Step() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        auto command_lock = robot_.LockToolState();
        if (robot_.IsRciEnabled()) return Result::ConflictTaskRunning;
        bool motion_active_ = false;
        bool start_worker_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ == State::Loaded) {
                if (robot_.script_active_) return Result::ConflictTaskRunning;
                state_ = State::Running;
                stop_requested_ = false;
                pause_requested_ = false;
                step_mode_ = true;
                step_started_ = false;
                start_worker_ = true;
            } else {
                if (state_ != State::Paused) {
                    return Result::LuaStateConflict;
                }
                motion_active_ = this->motion_active_;
            }
        }

        if (start_worker_) {
            robot_.script_active_ = true;
            try {
                worker_ = std::thread(&Impl::execute, this);
            } catch (const std::system_error& exception_) {
                robot_.script_active_ = false;
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = State::Loaded;
                step_mode_ = false;
                error_ = exception_.what();
                Logger::logCode(LogLevel::Error, "LuaScriptEngine", static_cast<int>(Result::ResourceUnavailable), "Cannot start Lua worker thread: {}", this->error_);
                return Result::ResourceUnavailable;
            }
            return Result::NoError;
        }

        if (motion_active_) {
            const Result result_ = robot_.ResumeMotion();
            if (result_ != Result::NoError) {
                return result_;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Paused) {
                return Result::LuaStateConflict;
            }
            pause_requested_ = false;
            step_mode_ = true;
            step_started_ = true;
            state_ = State::Running;
        }
        control_cv_.notify_all();
        return Result::NoError;
    }

    Result AddBreakpoint(const std::string& filename, int line) {
        if (line <= 0) {
            return Result::LuaInvalidBreakpoint;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const std::string normalized_ =
            normalizeFilename(filename.empty() ? filename_ : filename);
        if (normalized_.empty()) {
            return Result::LuaInvalidBreakpoint;
        }
        breakpoints_.insert(Breakpoint{normalized_, line});
        return Result::NoError;
    }

    Result RemoveBreakpoint(const std::string& filename, int line) {
        if (line <= 0) {
            return Result::LuaInvalidBreakpoint;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const std::string normalized_ =
            normalizeFilename(filename.empty() ? filename_ : filename);
        if (breakpoints_.erase(Breakpoint{normalized_, line}) == 0) {
            return Result::LuaInvalidBreakpoint;
        }
        return Result::NoError;
    }

    Result ClearBreakpoints() {
        std::lock_guard<std::mutex> lock(mutex_);
        breakpoints_.clear();
        return Result::NoError;
    }

    Status GetStatus() const {
        std::lock_guard<std::mutex> lock(mutex_);
        Status status_;
        status_.state = state_;
        status_.script_id = script_id_;
        status_.location = location_;
        status_.error = error_;
        status_.motion_active = motion_active_;
        status_.breakpoints.reserve(breakpoints_.size());
        for (const auto& breakpoint_ : breakpoints_) {
            status_.breakpoints.push_back(
                SourceLocation{breakpoint_.filename, breakpoint_.line});
        }
        return status_;
    }

private:
    void initializeLua() {
        function_ = sol::protected_function();
        lua_ = std::make_unique<sol::state>();
        lua_->open_libraries(sol::lib::base,
                             sol::lib::math,
                             sol::lib::string,
                             sol::lib::table,
                             sol::lib::coroutine,
                             sol::lib::utf8);

        (*lua_)["dofile"] = sol::nil;
        (*lua_)["loadfile"] = sol::nil;
        (*lua_)["require"] = sol::nil;

        *static_cast<Impl**>(lua_getextraspace(lua_->lua_state())) = this;
        registerBindings();
    }

    void registerBindings() {
        sol::table robot_table_ = lua_->create_named_table("robot");

        robot_table_.set_function(
            "MoveJ",
            [this](const sol::table& joints,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                const int joint_count_ = robot_.getJointNum();
                if (joint_count_ <= 0 ||
                    joints.size() != static_cast<std::size_t>(joint_count_)) {
                    throw sol::error("MoveJ joint count does not match the robot.");
                }

                JntArray target_(static_cast<unsigned int>(joint_count_));
                for (int index_ = 0; index_ < joint_count_; ++index_) {
                    const sol::object value_ = joints[index_ + 1];
                    if (!value_.is<double>()) {
                        throw sol::error("MoveJ joint positions must be numeric.");
                    }
                    target_(static_cast<unsigned int>(index_)) =
                        requireFinite(value_.as<double>(), "MoveJ joint");
                }

                executeMotion(robot_.MoveJ(
                    target_,
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "MoveL",
            [this](const sol::table& pose,
                   sol::optional<std::string> tool_name,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveL(
                    toFrame(pose),
                    tool_name.value_or(""),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "MoveCByCenter",
            [this](const sol::table& start_pose,
                   const sol::table& center_pose,
                   double theta,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveC(
                    toFrame(start_pose),
                    toFrame(center_pose),
                    requireFinite(theta, "theta"),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "MoveCByPoints",
            [this](const sol::table& start_pose,
                   const sol::table& via_pose,
                   const sol::table& goal_pose,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveC(
                    toFrame(start_pose),
                    toFrame(via_pose),
                    toFrame(goal_pose),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "GetState", [this]() { return robot_.GetStateString(); });
        robot_table_.set_function(
            "IsEnabled", [this]() { return robot_.IsEnabled(); });
        robot_table_.set_function(
            "GetJointCount", [this]() { return robot_.getJointNum(); });
        robot_table_.set_function(
            "GetJointPosition", [this](int index) {
                const int joint_count_ = robot_.getJointNum();
                if (index <= 0 || index > joint_count_) {
                    throw sol::error("Joint index is out of range.");
                }
                return robot_.getJointPosition(index - 1);
            });
        robot_table_.set_function(
            "Sleep", [this](int milliseconds) { sleepInterruptibly(milliseconds); });
        robot_table_.set_function(
            "SetEnabled", [this]() {
                requireSuccess(robot_.SetEnabled(), "SetEnabled");
            });
        robot_table_.set_function(
            "SetDisabled", [this]() {
                requireSuccess(robot_.SetDisabled(), "SetDisabled");
            });
        robot_table_.set_function(
            "ResetFault", [this]() {
                requireSuccess(robot_.ResetFault(), "ResetFault");
            });

        // ===== 状态查询 =====
        robot_table_.set_function(
            "IsDisabled", [this]() { return robot_.IsDisabled(); });
        robot_table_.set_function(
            "IsRunning", [this]() { return robot_.IsRunning(); });
        robot_table_.set_function(
            "IsControlActive", [this]() { return robot_.IsControlActive(); });
        robot_table_.set_function(
            "IsMotionBusy", [this]() { return robot_.IsMotionBusy(); });

        // ===== 单轴状态（索引 1 基，与 GetJointPosition 一致） =====
        robot_table_.set_function(
            "GetJointVelocity", [this](int index) {
                return robot_.getJointVelocity(checkedJointIndex(index));
            });
        robot_table_.set_function(
            "GetJointTorque", [this](int index) {
                return robot_.getJointTorque(checkedJointIndex(index));
            });
        robot_table_.set_function(
            "GetJointLoadTorque", [this](int index) {
                return robot_.getJointLoadTorque(checkedJointIndex(index));
            });
        robot_table_.set_function(
            "GetJointTorqueFilter", [this](int index) {
                return robot_.getJointTorqueFilter(checkedJointIndex(index));
            });
        robot_table_.set_function(
            "GetJointName", [this](int index) {
                return robot_.getJointName(checkedJointIndex(index));
            });
        robot_table_.set_function(
            "GetJointStatus", [this](int index) {
                return robot_.getJointStatus(checkedJointIndex(index));
            });
        robot_table_.set_function(
            "GetFlange", [this]() { return frameToTable(robot_.getFlange()); });
        robot_table_.set_function(
            "GetTcpPose", [this](sol::optional<std::string> tool_name) {
                Frame tcp;
                requireSuccess(robot_.GetTcpPose(tcp, tool_name.value_or("")), "GetTcpPose");
                return frameToTable(tcp);
            });

        // ===== 运动（阻塞，与 MoveJ/MoveL 一致） =====
        robot_table_.set_function(
            "MoveJ_IK",
            [this](const sol::table& pose,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveJ_IK(
                    toFrame(pose),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });
        robot_table_.set_function(
            "MoveL_FK",
            [this](const sol::table& joints,
                   sol::optional<std::string> tool_name,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                const int joint_count_ = robot_.getJointNum();
                if (joint_count_ <= 0 ||
                    joints.size() != static_cast<std::size_t>(joint_count_)) {
                    throw sol::error("MoveL_FK joint count does not match the robot.");
                }
                executeMotion(robot_.MoveL_FK(
                    toJntArray(joints, "MoveL_FK"),
                    tool_name.value_or(""),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        // ===== 运动控制（非阻塞，立即返回） =====
        robot_table_.set_function(
            "PauseMotion", [this]() {
                requireSuccess(robot_.PauseMotion(), "PauseMotion");
            });
        robot_table_.set_function(
            "StopMotion", [this]() {
                requireSuccess(robot_.StopMotion(), "StopMotion");
            });
        robot_table_.set_function(
            "ResumeMotion", [this]() {
                requireSuccess(robot_.ResumeMotion(), "ResumeMotion");
            });

        // ===== 工作模式 / 伺服 =====
        robot_table_.set_function(
            "SetWorkMode", [this](const std::string& mode) {
                requireSuccess(robot_.SetWorkMode(mode), "SetWorkMode");
            });
        robot_table_.set_function(
            "SetServoMode", [this](const std::string& mode) {
                requireSuccess(robot_.SetServoMode(mode), "SetServoMode");
            });

        // ===== 负载参数 =====
        robot_table_.set_function(
            "HasLoadParameters", [this]() { return robot_.HasLoadParameters(); });
        robot_table_.set_function(
            "GetLoadMass", [this]() { return robot_.GetLoadMass(); });
        robot_table_.set_function(
            "GetLoadCom", [this]() { return vectorToTable(robot_.GetLoadCom()); });
        robot_table_.set_function(
            "SetLoadParameters", [this](double mass, const sol::table& com) {
                requireSuccess(robot_.SetLoadParameters(mass, toVector(com)),
                               "SetLoadParameters");
            });
        robot_table_.set_function(
            "LoadLoadParameters", [this](sol::optional<std::string> path) {
                requireSuccess(robot_.LoadLoadParameters(path.value_or("")),
                               "LoadLoadParameters");
            });
        robot_table_.set_function(
            "SaveLoadParameters", [this](sol::optional<std::string> path) {
                requireSuccess(robot_.SaveLoadParameters(path.value_or("")),
                               "SaveLoadParameters");
            });

        // ===== 命名坐标系（工具系 / 工件系） =====
        robot_table_.set_function(
            "GetToolFrameNames", [this]() { return robot_.GetToolFrameNames(); });
        robot_table_.set_function(
            "GetObjectFrameNames", [this]() { return robot_.GetObjectFrameNames(); });
        robot_table_.set_function(
            "HasToolFrame", [this](const std::string& name) {
                return robot_.HasToolFrame(name);
            });
        robot_table_.set_function(
            "HasObjectFrame", [this](const std::string& name) {
                return robot_.HasObjectFrame(name);
            });
        robot_table_.set_function(
            "IsValidFrameName", [this](const std::string& name) {
                return robot_.IsValidFrameName(name);
            });
        robot_table_.set_function(
            "GetToolFrame", [this](const std::string& name) {
                Frame frame_;
                requireSuccess(robot_.GetToolFrame(name, frame_), "GetToolFrame");
                return frameToTable(frame_);
            });
        robot_table_.set_function(
            "GetObjectFrame", [this](const std::string& name) {
                Frame frame_;
                requireSuccess(robot_.GetObjectFrame(name, frame_), "GetObjectFrame");
                return frameToTable(frame_);
            });
        robot_table_.set_function(
            "SetToolFrame", [this](const std::string& name, const sol::table& pose) {
                requireSuccess(robot_.SetToolFrame(name, toFrame(pose)),
                               "SetToolFrame");
            });
        robot_table_.set_function(
            "SetObjectFrame", [this](const std::string& name, const sol::table& pose) {
                requireSuccess(robot_.SetObjectFrame(name, toFrame(pose)),
                               "SetObjectFrame");
            });
        robot_table_.set_function(
            "AddToolFrame", [this](const std::string& name, const sol::table& pose) {
                requireSuccess(robot_.AddToolFrame(name, toFrame(pose)),
                               "AddToolFrame");
            });
        robot_table_.set_function(
            "AddObjectFrame", [this](const std::string& name, const sol::table& pose) {
                requireSuccess(robot_.AddObjectFrame(name, toFrame(pose)),
                               "AddObjectFrame");
            });
        robot_table_.set_function(
            "RemoveToolFrame", [this](const std::string& name) {
                requireSuccess(robot_.RemoveToolFrame(name), "RemoveToolFrame");
            });
        robot_table_.set_function(
            "RemoveObjectFrame", [this](const std::string& name) {
                requireSuccess(robot_.RemoveObjectFrame(name), "RemoveObjectFrame");
            });
        robot_table_.set_function(
            "SetActiveToolFrame", [this](const std::string& name) {
                requireSuccess(robot_.SetActiveToolFrame(name), "SetActiveToolFrame");
            });
        robot_table_.set_function(
            "SetActiveObjectFrame", [this](const std::string& name) {
                requireSuccess(robot_.SetActiveObjectFrame(name), "SetActiveObjectFrame");
            });
        robot_table_.set_function(
            "GetActiveToolFrameName", [this]() { return robot_.GetActiveToolFrameName(); });
        robot_table_.set_function(
            "GetActiveObjectFrameName", [this]() { return robot_.GetActiveObjectFrameName(); });
        robot_table_.set_function(
            "GetActiveToolFrame", [this]() { return frameToTable(robot_.GetActiveToolFrame()); });
        robot_table_.set_function(
            "GetActiveObjectFrame", [this]() { return frameToTable(robot_.GetActiveObjectFrame()); });
        robot_table_.set_function(
            "LoadFrames", [this](const std::string& path) {
                requireSuccess(robot_.LoadFrames(path), "LoadFrames");
            });
        robot_table_.set_function(
            "SaveFrames", [this](const std::string& path) {
                requireSuccess(robot_.SaveFrames(path), "SaveFrames");
            });

        // ===== 阻抗参数（返回 JSON 字符串） =====
        robot_table_.set_function(
            "GetImpedanceParams", [this]() { return robot_.GetImpedanceParams().dump(); });
    }

    static void debugHook(lua_State* lua_state, lua_Debug* debug_info) {
        auto* impl_ =
            *static_cast<Impl**>(lua_getextraspace(lua_state));
        if (impl_ != nullptr) {
            impl_->handleHook(lua_state, debug_info);
        }
    }

    void handleHook(lua_State* lua_state, lua_Debug* debug_info) {
        bool should_stop_ = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (debug_info->event == LUA_HOOKLINE) {
                location_ = SourceLocation{filename_, debug_info->currentline};

                if (step_mode_) {
                    if (step_started_) {
                        step_mode_ = false;
                        pause_requested_ = true;
                    } else {
                        step_started_ = true;
                    }
                }

                if (breakpoints_.count(
                        Breakpoint{filename_, debug_info->currentline}) != 0) {
                    pause_requested_ = true;
                }
            }

            if (pause_requested_ && !stop_requested_) {
                state_ = State::Paused;
                control_cv_.wait(lock, [this]() {
                    return !pause_requested_ || stop_requested_;
                });
                if (!stop_requested_) {
                    state_ = State::Running;
                }
            }
            should_stop_ = stop_requested_;
        }

        if (should_stop_) {
            luaL_error(lua_state, "%s", SCRIPT_STOP_MARKER);
        }
    }

    void execute() {
        lua_State* lua_state_ = lua_->lua_state();
        lua_sethook(lua_state_, &Impl::debugHook,
                    LUA_MASKLINE | LUA_MASKCOUNT, 1000);

        try {
            sol::protected_function_result result_ = function_();
            lua_sethook(lua_state_, nullptr, 0, 0);

            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = false;
            if (!result_.valid()) {
                const sol::error error_ = result_;
                this->error_ = error_.what();
                if (stop_requested_ &&
                    this->error_.find(SCRIPT_STOP_MARKER) !=
                        std::string::npos) {
                    state_ = State::Stopped;
                    this->error_.clear();
                } else {
                    state_ = State::Failed;
                }
            } else if (stop_requested_) {
                state_ = State::Stopped;
                error_.clear();
            } else {
                state_ = State::Completed;
                error_.clear();
            }
        } catch (const std::exception& exception_) {
            lua_sethook(lua_state_, nullptr, 0, 0);
            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = false;
            if (stop_requested_ &&
                std::string(exception_.what()).find(SCRIPT_STOP_MARKER) !=
                    std::string::npos) {
                state_ = State::Stopped;
                error_.clear();
            } else {
                state_ = State::Failed;
                error_ = exception_.what();
                Logger::logCode(LogLevel::Error, "LuaScriptEngine", static_cast<int>(Result::LuaExecutionError), "Lua script execution failed: {}", error_);
            }
        }
        robot_.script_active_ = false;
        control_cv_.notify_all();
    }

    void executeMotion(Result result) {
        if (result != Result::NoError && result != Result::PlanFinished) {
            throw sol::error(
                "Robot motion submission failed: " + to_string(result));
        }
        if (result == Result::PlanFinished) {
            return;
        }

        bool pause_requested_local = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = true;
            pause_requested_local = this->pause_requested_;
        }

        // 提交时若已有暂停请求（Pause 在 motion_active_ 置位前抢先），此处补发
        if (pause_requested_local) {
            const Result pause_result_ = robot_.PauseMotion();
            if (pause_result_ != Result::NoError) {
                std::lock_guard<std::mutex> lock(mutex_);
                motion_active_ = false;
                throw sol::error(
                    "Robot motion pause failed: " + to_string(pause_result_));
            }
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::Paused;
        }

        const Result wait_result_ = robot_.WaitMove();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = false;
        }

        if (wait_result_ == Result::Fatal) {
            throw sol::error("Robot motion terminated in ERROR_STATE.");
        }
        if (wait_result_ == Result::NotEnabled) {
            throw sol::error("Robot motion terminated in IDLE.");
        }

        bool should_stop_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            should_stop_ = stop_requested_;
        }
        if (should_stop_) {
            throw sol::error(SCRIPT_STOP_MARKER);
        }
    }

    void sleepInterruptibly(int milliseconds) {
        if (milliseconds < 0) {
            throw sol::error("Sleep duration must be nonnegative.");
        }

        auto remaining_ = std::chrono::milliseconds(milliseconds);
        while (remaining_.count() > 0) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_requested_) {
                throw sol::error(SCRIPT_STOP_MARKER);
            }
            if (pause_requested_) {
                state_ = State::Paused;
                control_cv_.wait(lock, [this]() {
                    return !pause_requested_ || stop_requested_;
                });
                if (stop_requested_) {
                    throw sol::error(SCRIPT_STOP_MARKER);
                }
                state_ = State::Running;
            }

            const auto start_ = std::chrono::steady_clock::now();
            control_cv_.wait_for(lock, remaining_, [this]() {
                return stop_requested_ || pause_requested_;
            });
            const auto elapsed_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_);
            remaining_ = elapsed_ >= remaining_
                             ? std::chrono::milliseconds(0)
                             : remaining_ - elapsed_;
        }
    }

    static double requireFinite(double value, const char* name) {
        if (!std::isfinite(value)) {
            throw sol::error(std::string(name) + " contains NaN or infinity.");
        }
        return value;
    }

    static double requirePositive(double value, const char* name) {
        requireFinite(value, name);
        if (value <= 0.0) {
            throw sol::error(std::string(name) + " must be greater than zero.");
        }
        return value;
    }

    // 将 Robot 控制接口的返回值统一转为 Lua 错误，与运动提交行为保持一致
    static void requireSuccess(Result result, const char* operation) {
        if (result != Result::NoError) {
            throw sol::error(std::string(operation) + " failed: " +
                             to_string(result));
        }
    }

    static double tableNumber(const sol::table& table, const char* key) {
        const sol::object value_ = table[key];
        if (!value_.is<double>()) {
            throw sol::error(std::string("pose.") + key + " must be numeric.");
        }
        return requireFinite(value_.as<double>(), key);
    }

    static Frame toFrame(const sol::table& table) {
        Pose pose_;
        pose_.x = tableNumber(table, "x");
        pose_.y = tableNumber(table, "y");
        pose_.z = tableNumber(table, "z");
        pose_.qx = tableNumber(table, "qx");
        pose_.qy = tableNumber(table, "qy");
        pose_.qz = tableNumber(table, "qz");
        pose_.qw = tableNumber(table, "qw");

        const double norm_ = std::sqrt(
            pose_.qx * pose_.qx + pose_.qy * pose_.qy +
            pose_.qz * pose_.qz + pose_.qw * pose_.qw);
        if (norm_ <= 1e-12) {
            throw sol::error("Pose quaternion must have nonzero norm.");
        }

        return Frame(
            rocos::Rotation::Quaternion(
                pose_.qx / norm_,
                pose_.qy / norm_,
                pose_.qz / norm_,
                pose_.qw / norm_),
            rocos::Vector(pose_.x, pose_.y, pose_.z));
    }

    // ---- Lua table ↔ C++ 类型转换辅助 ----

    // 校验并返回 0 基关节索引（Lua 侧统一 1 基）
    int checkedJointIndex(int index) const {
        const int joint_count_ = robot_.getJointNum();
        if (index <= 0 || index > joint_count_) {
            throw sol::error("Joint index is out of range.");
        }
        return index - 1;
    }

    // Lua 数组（1 基）→ JntArray
    static JntArray toJntArray(const sol::table& joints, const char* name) {
        const std::size_t count_ = joints.size();
        if (count_ == 0) {
            throw sol::error(std::string(name) + " joint array must not be empty.");
        }
        JntArray result_(static_cast<unsigned int>(count_));
        for (std::size_t i = 0; i < count_; ++i) {
            const sol::object value_ = joints[i + 1];
            if (!value_.is<double>()) {
                throw sol::error(std::string(name) + " joint positions must be numeric.");
            }
            result_(static_cast<unsigned int>(i)) =
                requireFinite(value_.as<double>(), name);
        }
        return result_;
    }

    // Lua 表 {x=, y=, z=} → Vector
    static Vector toVector(const sol::table& table) {
        const sol::object x_ = table["x"];
        const sol::object y_ = table["y"];
        const sol::object z_ = table["z"];
        if (!x_.is<double>() || !y_.is<double>() || !z_.is<double>()) {
            throw sol::error("Vector must contain numeric x, y and z fields.");
        }
        return Vector(requireFinite(x_.as<double>(), "x"),
                      requireFinite(y_.as<double>(), "y"),
                      requireFinite(z_.as<double>(), "z"));
    }

    // Frame → Lua 表 {x,y,z,qx,qy,qz,qw}
    sol::table frameToTable(const Frame& frame) {
        double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
        frame.M.GetQuaternion(qx, qy, qz, qw);
        sol::table table_ = lua_->create_table();
        table_["x"] = frame.p.x();
        table_["y"] = frame.p.y();
        table_["z"] = frame.p.z();
        table_["qx"] = qx;
        table_["qy"] = qy;
        table_["qz"] = qz;
        table_["qw"] = qw;
        return table_;
    }

    // Vector → Lua 表 {x,y,z}
    sol::table vectorToTable(const Vector& vec) {
        sol::table table_ = lua_->create_table();
        table_["x"] = vec.x();
        table_["y"] = vec.y();
        table_["z"] = vec.z();
        return table_;
    }

    void joinWorker() {
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    Robot& robot_;
    std::filesystem::path scripts_root_;
    Logger::logger_ptr log_ptr_;

    std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    std::condition_variable control_cv_;
    std::thread worker_;
    std::unique_ptr<sol::state> lua_;
    sol::protected_function function_;

    State state_{State::Empty};
    std::string source_;
    std::string filename_;
    std::string script_id_;
    std::string error_;
    SourceLocation location_;
    std::set<Breakpoint> breakpoints_;
    std::uint64_t script_counter_{0};
    bool stop_requested_{false};
    bool pause_requested_{false};
    bool step_mode_{false};
    bool step_started_{false};
    bool motion_active_{false};

};

LuaScriptEngine::LuaScriptEngine(
    Robot& robot, std::filesystem::path scripts_root)
    : impl_(std::make_unique<Impl>(
          robot, std::move(scripts_root))) {}

LuaScriptEngine::~LuaScriptEngine() = default;

Result LuaScriptEngine::LoadSource(
    const std::string& source, const std::string& filename) {
    return impl_->LoadSource(source, filename);
}

Result LuaScriptEngine::LoadFile(
    const std::filesystem::path& relative_path) {
    return impl_->LoadFile(relative_path);
}

Result LuaScriptEngine::ReadScriptFile(
    const std::filesystem::path& relative_path, std::string& source) const {
    return impl_->ReadScriptFile(relative_path, source);
}

std::vector<std::string> LuaScriptEngine::ListScripts() const {
    return impl_->ListScripts();
}

Result LuaScriptEngine::Run() {
    return impl_->Run();
}

Result LuaScriptEngine::Pause() {
    return impl_->Pause();
}

Result LuaScriptEngine::Resume() {
    return impl_->Resume();
}

Result LuaScriptEngine::Stop() {
    return impl_->Stop();
}

Result LuaScriptEngine::Step() {
    return impl_->Step();
}

Result LuaScriptEngine::AddBreakpoint(
    const std::string& filename, int line) {
    return impl_->AddBreakpoint(filename, line);
}

Result LuaScriptEngine::RemoveBreakpoint(
    const std::string& filename, int line) {
    return impl_->RemoveBreakpoint(filename, line);
}

Result LuaScriptEngine::ClearBreakpoints() {
    return impl_->ClearBreakpoints();
}

LuaScriptEngine::Status LuaScriptEngine::GetStatus() const {
    return impl_->GetStatus();
}

const char* LuaScriptEngine::ToString(State state) noexcept {
    switch (state) {
        case State::Empty:
            return "EMPTY";
        case State::Loaded:
            return "LOADED";
        case State::Running:
            return "RUNNING";
        case State::Pausing:
            return "PAUSING";
        case State::Paused:
            return "PAUSED";
        case State::Stopping:
            return "STOPPING";
        case State::Completed:
            return "COMPLETED";
        case State::Failed:
            return "FAILED";
        case State::Stopped:
            return "STOPPED";
    }
    return "UNKNOWN";
}

}  // namespace rocos
