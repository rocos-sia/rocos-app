// Copyright 2026, Yang Luo"
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
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "move_servo.hpp"
#include "feedback_validation.hpp"
#include <openssl/rand.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include "log_manager.hpp"
#include "logger.hpp"
#include "position_command_validation.hpp"
#include "realtime_thread.hpp"

namespace rocos {

// ============================================================================
// 构造函数
// ============================================================================

MoveServo::MoveServo(HardwareInterface* hw, ModelInterface* model, uint16_t listen_port,
                     const ServoSettings& settings, const Frame& tool, std::shared_ptr<ServoSession> session)
    : MotionInterface(model), hw_(hw), listen_port_(listen_port), settings_(settings), tool_(tool), session_(std::move(session)) {
    sockpp::initialize();

    if (hw_) {
        joint_count_ = std::max(0, hw_->GetDriveNum());
    }
    if (model_) {
        joint_count_ = model_->GetJointNum();
    }

    cmd_ = MotionGeneratorCommand{};
}

// ============================================================================
// 析构函数
// ============================================================================

MoveServo::~MoveServo() { Abort(); }

void MoveServo::Abort() {
    if (session_) session_->valid.store(false);
    running_.store(false);
    stopped_.store(true);
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) thread_.join();
    // 线程已结束，最后一次非阻塞发送不依赖客户端继续发包；UDP 仍是尽力交付。
    if (sock_.is_open() && peer_bound_) {
        auto state = buildRobotState();
        sock_.send_to(&state, sizeof(state), peer_);
    }
    sock_.close();
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (cycle_event_fd_ >= 0) close(cycle_event_fd_);
        cycle_event_fd_ = -1;
    }
    peer_bound_ = false;
}

// ============================================================================
// ValidateParameters
// ============================================================================

Result MoveServo::ValidateParameters() const {
    if (!hw_ || !model_) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (joint_count_ <= 0 || joint_count_ > static_cast<int>(MAX_DOF) ||
        hw_->GetPosition().rows() != static_cast<unsigned int>(joint_count_)) {
        return Result::UnmatchedJointsNumber;
    }
    if (listen_port_ == 0 || hw_->GetDt() == 0 || !std::isfinite(settings_.cutoff_hz) ||
        settings_.cutoff_hz <= 0 || !std::isfinite(settings_.minimum_success_rate) ||
        settings_.minimum_success_rate < 0 || settings_.minimum_success_rate > 1 ||
        !settings_.max_missing_cycles || !settings_.low_quality_cycles ||
        !settings_.first_command_timeout_ms) {
        return Result::IllegalParameter;
    }
    const auto& velocity = model_->GetVelocityLimit();
    if (velocity.rows() != static_cast<unsigned int>(joint_count_)) {
        return Result::UnmatchedJointsNumber;
    }
    for (unsigned int i = 0; i < velocity.rows(); ++i) {
        if (!std::isfinite(velocity(i)) || velocity(i) < 0) {
            return Result::IllegalParameter;
        }
    }
    return Result::NoError;
}

// ============================================================================
// Reset —— 初始化状态并启动 UDP 接收线程
// ============================================================================

Result MoveServo::Reset() {
    // 先停止旧线程，再重置其访问的数据和 socket。
    Abort();
    if (!session_) session_=std::make_shared<ServoSession>();
    if (RAND_bytes(session_->id.data(),session_->id.size())!=1) return Result::ServoNotInitialized;
    session_->valid.store(true);
    joint_count_ = model_ ? model_->GetJointNum() : 0;
    Result rc = ValidateParameters();
    if (rc != Result::NoError) return rc;
    cycle_period_us_ = hw_->GetDt();
    rc = processor_.Reset(model_, hw_->GetJointDerivativeLimits(), hw_->GetPosition(),
                          cycle_period_us_ * 1e-6, settings_, tool_);
    if (rc != Result::NoError) return rc;

    sock_ = sockpp::udp_socket{};
    if (!sock_.is_open() || !sock_.bind(sockpp::inet_address(INADDR_ANY, listen_port_))) {
        sock_.close();
        return Result::ServoBindFail;
    }
    // 在线程启动前完成所有可失败的 socket 设置，直接向调用方返回错误。
    if (!sock_.set_non_blocking(true)) {
        sock_.close();
        return Result::ServoNotInitialized;
    }

    cycle_event_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (cycle_event_fd_ < 0) {
        sock_.close();
        return Result::ServoNotInitialized;
    }

    std::string timing_error;
    timing_trace_ = ServoTimingTrace::Create(session_->Hex(), timing_error);
    timing_cycle_ = {};
    if (timing_trace_)
        Logger::getInstance("MoveServo")->info("Servo timing capture: {} (65536 records per stream)",
                                               timing_trace_->Path());
    else if (!timing_error.empty())
        Logger::getInstance("MoveServo")->warn("Servo timing capture disabled: {}", timing_error);

    // 重置共享缓冲区
    {
        std::lock_guard<std::mutex> lock(mtx_);
        published_state_=RobotState{};
        applied_id_=generated_id_=0;

        cmd_ = MotionGeneratorCommand{};
        has_new_cmd_ = false;
        first_cmd_ = MotionGeneratorCommand{};
        first_command_checked_ = false;
        fault_ = stop_reason_ = Result::NoError;
        stopping_ = stop_complete_ = false;
        processor_ready_ = false;
        consumed_id_ = cycle_ = low_quality_count_ = 0;
        communication_.Reset();
        started_at_ = last_received_at_ = last_cycle_at_ = Clock::now();
    }
    stopped_.store(false);
    running_.store(true);
    message_id_counter_ = 0;

    if (current_mode_ == MotionMode::kNone) {
        current_mode_ = MotionMode::kJointPosition;
    }

    // 启动 UDP 接收线程
    try {
        thread_ = std::thread(&MoveServo::udpReceiveLoop, this);
    } catch (const std::system_error&) {
        running_.store(false);
        stopped_.store(true);
        sock_.close();
        close(cycle_event_fd_);
        cycle_event_fd_ = -1;
        return Result::ServoThreadStartFailed;
    }

    return Result::NoError;
}

// ============================================================================
// Pause / Resume / Stop
// ============================================================================

Result MoveServo::Pause() { return Result::FunctionNotSupported; }

Result MoveServo::Resume() { return Result::FunctionNotSupported; }

Result MoveServo::Stop() {
    bool no_command = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        beginStop(Result::NoError);
        if (!processor_ready_ || processor_.AtRest()) stopped_.store(true);
        no_command = !has_new_cmd_;
    }
    if (no_command) Abort();
    return Result::NoError;
}

Result MoveServo::SetMode(MotionMode mode) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mode != MotionMode::kJointPosition && mode != MotionMode::kCartesianPosition)
        return Result::IllegalParameter;
    if (current_mode_.load() != mode) {
        if (stopping_ || (processor_ready_ && !processor_.AtRest()))
            return Result::ConflictTaskRunning;
        current_mode_.store(mode);
        has_new_cmd_ = false;
        first_command_checked_ = processor_ready_ = false;
        communication_.Reset();
        consumed_id_ = low_quality_count_ = 0;
        started_at_ = last_cycle_at_ = Clock::now();
    }
    return Result::NoError;
}

Result MoveServo::fail(Result result) {
    fault_ = result;
    stopped_.store(true);
    emitLogEvent(LogEventType::Result, LogModule::MoveServo, static_cast<int>(result));
    return result;
}

void MoveServo::beginStop(Result reason) {
    if (stopping_) return;
    stopping_ = true;
    stop_reason_ = reason;
    if (reason == Result::LostHeartbeat)
        emitLogEvent(LogEventType::ServoCommunicationFault, LogModule::MoveServo,
                     static_cast<int>(reason), -1, communication_.SuccessRate(),
                     communication_.MissingCycles(), settings_.minimum_success_rate);
    else if (reason != Result::NoError)
        emitLogEvent(LogEventType::ServoGuardStop, LogModule::MoveServo,
                     static_cast<int>(reason));
}

Result MoveServo::processStop(Reference& ref_out) {
    if (stop_complete_) {
        if (stop_reason_ != Result::NoError) return fail(stop_reason_);
        stopped_.store(true);
        ref_out = processor_.Position();
        return Result::PlanFinished;
    }
    if (!processor_ready_) {
        if (stop_reason_ != Result::NoError) return fail(stop_reason_);
        const auto rc = fillCurrentReference(ref_out);
        return rc == Result::NoError ? Result::PlanFinished : fail(rc);
    }
    JntArray output;
    const auto rc = processor_.Step(processor_.Position(), true, output);
    if (rc != Result::NoError) return fail(rc);
    auto coupled = output;
    const auto coupling_rc = hw_->ApplyPositionCoupling(coupled);
    if (coupling_rc != Result::NoError) return fail(coupling_rc);
    for (unsigned int i = 0; i < output.rows(); ++i)
        if (std::abs(coupled(i) - output(i)) > 1e-9) return fail(Result::MotionLimit);
    ref_out = output;
    stop_complete_ = processor_.AtRest();
    // 最后一个零速度/加速度样本先交给 Executor，下周期再结束/报错。
    return Result::NoError;
}

MoveServo::Diagnostics MoveServo::GetDiagnostics() const {
    std::lock_guard<std::mutex> lock(mtx_);
    Diagnostics out;
    out.cycle = cycle_;
    out.received_id = cmd_.message_id;
    out.success_rate = communication_.SuccessRate();
    out.control_cycles = communication_.ControlCycles();
    out.elapsed_periods = communication_.ElapsedPeriods();
    out.missing_cycles = communication_.MissingCycles();
    out.stopping = stopping_;
    out.fault = fault_ != Result::NoError ? fault_ : stop_reason_;
    if (processor_ready_) {
        out.desired_position = processor_.Position();
        out.desired_velocity = processor_.Velocity();
        out.desired_acceleration = processor_.Acceleration();
    }
    return out;
}

// ============================================================================
// GenerateRef —— 将最新收到的指令转为 Reference
// ============================================================================

Result MoveServo::GenerateRef(Reference& ref_out) {
    const auto entry_ns = timing_trace_ ? ServoTimingTrace::Now() : 0;
    std::lock_guard<std::mutex> lock(mtx_);
    ++cycle_;
    if (timing_trace_) {
        timing_cycle_ = {};
        timing_cycle_.cycle = cycle_;
        timing_cycle_.entry_ns = entry_ns;
        timing_cycle_.locked_ns = ServoTimingTrace::Now();
    }
    if (!hw_) return Result::ParameterPointerEqualsNullptr;
    if (!model_) return fillCurrentReference(ref_out);
    if (fault_ != Result::NoError) return fault_;
    if (cycle_period_us_ && hw_->GetDt() != cycle_period_us_) return fail(Result::IllegalParameter);
    if (session_ && !session_->Active() && !stopping_) beginStop(Result::LostHeartbeat);
    const auto feedback_rc=hw_->ReadFeedbackSnapshot(cycle_feedback_);
    if (feedback_rc!=Result::NoError) return fail(feedback_rc);
    // 正常运动和制动共用实际反馈范围/有效性检查。
    const auto validation = ValidateFeedback(*model_, *hw_, cycle_feedback_);
    if (validation!=Result::NoError) {
        emitLogEvent(LogEventType::ServoFeedbackInvalid, LogModule::MoveServo,
                     static_cast<int>(validation));
        return fail(validation);
    }
    if (processor_ready_ && !stopping_) {
        const auto feedback =
            processor_.CheckFeedback(cycle_feedback_.position, cycle_feedback_.velocity);
        if (feedback != Result::NoError) beginStop(feedback);
    }
    if (stopping_) return processStop(ref_out);
    if (stopped_.load()) {
        const auto rc = fillCurrentReference(ref_out);
        return rc == Result::NoError ? Result::PlanFinished : rc;
    }
    const auto now = Clock::now();
    if (!has_new_cmd_) {
        if (running_.load() && settings_.communication_guard_enabled &&
            now - started_at_ >= std::chrono::milliseconds(settings_.first_command_timeout_ms)) {
            beginStop(Result::LostHeartbeat);
            return processStop(ref_out);
        }
        return fillCurrentReference(ref_out);
    }
    const double dt = hw_->GetDt() * 1e-6;
    const bool missed_deadline =
        processor_ready_ &&
        now - last_cycle_at_ >= std::chrono::duration<double>(dt * settings_.max_missing_cycles);
    const uint64_t period_ns = uint64_t(cycle_period_us_) * 1000;
    const uint64_t elapsed_ns = first_command_checked_
        ? std::chrono::duration_cast<std::chrono::nanoseconds>(now-last_cycle_at_).count()
        : period_ns;
    last_cycle_at_ = now;
    if (timing_trace_) {
        timing_cycle_.consumed_ns = ServoTimingTrace::Now();
        timing_cycle_.consumed_id = cmd_.message_id;
    }
    // 统计取用点的新序号覆盖；包全到但发布跨过取用边界时仍会记缺帧。
    // 原因与排查：docs/servo_timing_root_cause_and_diagnosis.md。
    communication_.RecordCycle(cmd_.message_id != consumed_id_, elapsed_ns, period_ns);
    consumed_id_ = cmd_.message_id;
    if (communication_.FullWindow() &&
        communication_.SuccessRate() < settings_.minimum_success_rate)
        ++low_quality_count_;
    else
        low_quality_count_ = 0;
    if (settings_.communication_guard_enabled &&
        (missed_deadline || communication_.MissingCycles() >= settings_.max_missing_cycles ||
         now - last_received_at_ >=
             std::chrono::duration<double>(dt * settings_.max_missing_cycles) ||
         low_quality_count_ >= settings_.low_quality_cycles))
        beginStop(Result::LostHeartbeat);
    if (stopping_) return processStop(ref_out);

    Reference candidate;
    const bool first = !first_command_checked_;
    if (timing_trace_) timing_cycle_.reference_id = first ? first_cmd_.message_id : cmd_.message_id;
    auto rc = commandReference(first ? first_cmd_ : cmd_, candidate);
    if (rc == Result::NoError && first) rc = validateStartReference(candidate);
    if (rc != Result::NoError) return fail(rc);
    if (first) {
        rc = processor_.Reset(model_, hw_->GetJointDerivativeLimits(), hw_->GetPosition(), dt,
                              settings_, tool_);
        if (rc != Result::NoError) return fail(rc);
        // 启动导数按静止条件检查，不能把运动中的反馈伪装成零速度。
        const auto velocity = hw_->GetVelocity();
        const auto limits = hw_->GetJointDerivativeLimits();
        if (velocity.rows() != static_cast<unsigned int>(joint_count_))
            return fail(Result::UnmatchedJointsNumber);
        for (unsigned int i = 0; i < velocity.rows(); ++i)
            if (!std::isfinite(velocity(i)) || std::abs(velocity(i)) > limits[i].acceleration * dt)
                return fail(Result::HomePositionError);
        first_command_checked_ = processor_ready_ = true;
    }
    JntArray output;
    rc = processor_.Step(candidate, false, output);
    if (rc == Result::IkTrack || rc == Result::MotionLimit || rc == Result::IkCalcFail) {
        beginStop(rc);
        return processStop(ref_out);
    }
    if (rc != Result::NoError) return fail(rc);
    auto coupled = output;
    rc = hw_->ApplyPositionCoupling(coupled);
    if (rc != Result::NoError) return fail(rc);
    for (unsigned int i = 0; i < output.rows(); ++i)
        if (std::abs(coupled(i) - output(i)) > 1e-9) return fail(Result::MotionLimit);
    ref_out = output;
    generated_id_ = first ? first_cmd_.message_id : cmd_.message_id;
    return Result::NoError;
}

Result MoveServo::commandReference(const MotionGeneratorCommand& cmd, Reference& ref_out) const {
    if (current_mode_.load() == MotionMode::kJointPosition) {
        JntArray q(static_cast<unsigned int>(joint_count_));
        for (int i = 0; i < joint_count_; ++i) {
            if (!std::isfinite(cmd.q_c[i])) return Result::ParameterNanOrInf;
            q(i) = cmd.q_c[i];
        }
        ref_out = std::move(q);
        return Result::NoError;
    }
    if (current_mode_.load() != MotionMode::kCartesianPosition) return Result::IllegalParameter;
    for (double value : cmd.tcp_c)
        if (!std::isfinite(value)) return Result::ParameterNanOrInf;
    const auto& m = cmd.tcp_c;
    if (std::abs(m[12]) > 1e-9 || std::abs(m[13]) > 1e-9 || std::abs(m[14]) > 1e-9 ||
        std::abs(m[15] - 1) > 1e-9)
        return Result::IllegalParameter;
    const Frame frame = matrixToFrame(m);
    const auto rotation = frame.M;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double dot = 0;
            for (int k = 0; k < 3; ++k) dot += rotation(k, i) * rotation(k, j);
            if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-6) return Result::IllegalParameter;
        }
    }
    if (dot(rotation.UnitX() * rotation.UnitY(), rotation.UnitZ()) < 0)
        return Result::IllegalParameter;
    ref_out = frame;
    return Result::NoError;
}

Result MoveServo::validateStartReference(const Reference& ref) {
    if (!model_ || !hw_) return Result::ParameterPointerEqualsNullptr;
    const auto actual = hw_->GetPosition();
    JntArray target;
    if (const auto* q = std::get_if<JntArray>(&ref)) {
        target = *q;
    } else {
        target.resize(static_cast<unsigned int>(joint_count_));
        const auto rc =
            model_->InverseKinematics(actual, std::get<Frame>(ref) * tool_.Inverse(), target);
        if (rc != Result::NoError) return rc;
    }
    // 与硬件实际下发使用相同联动约束，所有轴均相对当前反馈检查。
    auto rc = hw_->ApplyPositionCoupling(target);
    if (rc != Result::NoError) return rc;
    JointCommandViolation violation;
    const double dt = hw_->GetDt() * 1e-6;
    rc = ValidateJointCommand(*model_, target, &actual, dt, &violation);
    if (rc == Result::SpeedLimit) {
        emitLogEvent(LogEventType::ServoStartPoseInvalid, LogModule::MoveServo,
                     static_cast<int>(Result::HomePositionError), violation.joint,
                     std::abs(target(violation.joint) - actual(violation.joint)), 0,
                     violation.upper * dt);
        return Result::HomePositionError;
    }
    if (rc != Result::NoError) return rc;
    const auto limits = hw_->GetJointDerivativeLimits();
    if (limits.size() != target.rows()) return Result::UnmatchedJointsNumber;
    JntArray dec(target.rows()), offset(target.rows()), lower, upper;
    for (unsigned int i = 0; i < target.rows(); ++i) {
        dec(i) = limits[i].deceleration;
        offset(i) = limits[i].velocity_offset;
    }
    rc = model_->GetPositionVelocityLimits(actual, dec, offset, lower, upper);
    if (rc != Result::NoError) return rc;
    for (unsigned int i = 0; i < target.rows(); ++i) {
        const auto delta = target(i) - actual(i);
        const double tolerance = (delta >= 0 ? upper(i) : -lower(i)) * dt;
        if (std::abs(delta) > tolerance) {
            emitLogEvent(LogEventType::ServoStartPoseInvalid, LogModule::MoveServo,
                         static_cast<int>(Result::HomePositionError), i, std::abs(delta), 0,
                         tolerance);
            return Result::HomePositionError;
        }
    }
    return rc;
}

// ============================================================================
// fillCurrentReference —— 按当前模式把机器人实际位置填入 Reference
// ============================================================================

Result MoveServo::fillCurrentReference(Reference& ref_out) {
    if (hw_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (joint_count_ <= 0) {
        return Result::UnmatchedJointsNumber;
    }

    JntArray q_now = hw_->GetPosition();
    if (static_cast<int>(q_now.rows()) < joint_count_) {
        return Result::UnmatchedJointsNumber;
    }

    // 始终向控制器传关节保持目标，避免 TCP 工具偏置被二次当作法兰 IK。
    JntArray q_ref(static_cast<unsigned int>(joint_count_));
    for (int i = 0; i < joint_count_; ++i) {
        auto ui = static_cast<unsigned int>(i);
        q_ref(ui) = q_now(ui);
    }
    ref_out = std::move(q_ref);
    return Result::NoError;
}

// ============================================================================
// buildRobotState —— 复制周期快照并添加回包序号（UDP 线程 / Abort join 后调用）
// ============================================================================

RobotState MoveServo::buildRobotState() {
    std::lock_guard<std::mutex> lock(mtx_);
    auto state=published_state_;
    state.message_id=++message_id_counter_;
    state.session_id=session_->id;
    return state;
}

RobotState MoveServo::GetPublishedState() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return published_state_;
}

nlohmann::json MoveServo::GetStatus() const {
    std::lock_guard<std::mutex> lock(mtx_);
    const auto& s = published_state_;
    return {{"cycle", s.cycle}, {"sample_time_ns", s.sample_time_ns},
            {"source_time_us", s.source_time_us},
            {"received_message_id", s.received_message_id},
            {"applied_message_id", s.applied_message_id},
            {"fault_code", static_cast<int>(fault_ != Result::NoError ? fault_ : stop_reason_)},
            {"errors", s.errors}, {"q", s.q}, {"dq", s.dq},
            {"q_d", s.q_d}, {"dq_d", s.dq_d}, {"ddq_d", s.ddq_d},
            {"q_cmd", s.q_cmd}, {"tau_cmd", s.tau_cmd},
            {"output_mode", s.output_mode}, {"output_valid", s.output_valid},
            {"control_command_success_rate", s.control_command_success_rate},
            {"control_cycles", communication_.ControlCycles()},
            {"elapsed_periods", communication_.ElapsedPeriods()}};
}

void MoveServo::CompleteCycle(Result result) {
    std::lock_guard<std::mutex> lock(mtx_);
    RobotState state{};
    if (static_cast<int>(result)<0 && fault_==Result::NoError) fault_=result;
    state.session_id=session_->id;
    state.cycle=cycle_;
    state.sample_time_ns=cycle_feedback_.sample_time_ns;
    state.source_time_us=cycle_feedback_.source_time_us;
    state.received_message_id=cmd_.message_id;
    state.output_valid=static_cast<int>(result)>=0 && !hw_->SafetyStopRequested();
    if (state.output_valid) applied_id_=generated_id_;
    state.applied_message_id=applied_id_;
    auto copy=[](const JntArray& input, auto& output) {
        for (unsigned int i=0;i<input.rows() && i<MAX_DOF;++i) output[i]=input(i);
    };
    copy(cycle_feedback_.position,state.q); copy(cycle_feedback_.velocity,state.dq);
    copy(cycle_feedback_.torque,state.tau); copy(cycle_feedback_.load,state.load);
    if (processor_ready_) {
        copy(processor_.Position(),state.q_d); copy(processor_.Velocity(),state.dq_d);
        copy(processor_.Acceleration(),state.ddq_d);
    } else {
        // 首点前控制器得到的是当前关节保持参考，不能把零向量当作期望位置。
        copy(cycle_feedback_.position,state.q_d);
    }
    const auto output=hw_->GetOutputSnapshot();
    state.output_mode=output.mode;
    copy(output.position,state.q_cmd); copy(output.torque,state.tau_cmd);
    Frame flange;
    if (cycle_feedback_.position.rows()==static_cast<unsigned int>(joint_count_) &&
        model_->ForwardKinematics(cycle_feedback_.position,flange)==Result::NoError) {
        frameToMatrix(flange,state.flange_to_base);
        frameToMatrix(flange*tool_,state.tcp_to_base);
    } else state.output_valid=false;
    const auto fault=static_cast<int>(result)<0?result:(fault_!=Result::NoError?fault_:stop_reason_);
    state.errors[0]=fault!=Result::NoError;
    state.errors[1]=fault==Result::HomePositionError;
    state.errors[2]=fault==Result::LostHeartbeat;
    state.errors[3]=fault==Result::PosLimit || fault==Result::SpeedLimit || fault==Result::AccLimit ||
                    fault==Result::JerkLimit || fault==Result::MotionLimit;
    state.mode=current_mode_;
    state.control_command_success_rate=communication_.SuccessRate();
    published_state_=state;
    if (timing_trace_) {
        timing_cycle_.applied_id = applied_id_;
        timing_cycle_.result = static_cast<int>(result);
        timing_cycle_.completed_ns = ServoTimingTrace::Now();
        timing_trace_->PushControl(timing_cycle_);
    }
    if (running_.load() && cycle_event_fd_ >= 0) {
        const uint64_t notify = 1;
        // 用控制周期驱动状态，避免等客户端独立 1 ms 节拍来包才回复旧快照。
        // 非阻塞计数通知；积压时 UDP 只取最新快照，不逐次补发旧周期。
        const auto written = write(cycle_event_fd_, &notify, sizeof(notify));
        (void)written;
    }
}

// ============================================================================
// UDP 收发线程 —— 接收指令；由 CompleteCycle 通知推送最新 RobotState
// ============================================================================

void MoveServo::udpReceiveLoop() {
    pthread_setname_np(pthread_self(), "rocos_servo_udp");
    const auto scheduling = ConfigureRealtimeThread(20);
    const auto logger = Logger::getInstance("MoveServo");
    if (scheduling.ready()) logger->info("UDP scheduling: {}", scheduling.description());
    else logger->warn("UDP realtime scheduling unavailable: {}", scheduling.description());
    uint64_t latest_message_id = 0;
    sockpp::inet_address peer;
    bool peer_bound=false;
    sockpp::inet_address stream_peer;
    bool stream_ready = false;
    uint64_t sent_cycle = 0;
    // 多读一个字节，防止超长报文被截断成恰好合法的长度。
    std::array<char, sizeof(MotionGeneratorCommand) + 1> buf{};
    while (running_.load(std::memory_order_relaxed)) {
        sockpp::inet_address reply_addr;
        bool reply = false;
        // 有界排空，持续来包也不能饿死回包或停止请求。
        for (int count = 0; count < 256 && running_.load(); ++count) {
            sockpp::inet_address addr;
            const auto n = sock_.recv_from(buf.data(), buf.size(), &addr);
            const auto receive_ns = timing_trace_ && n > 0 ? ServoTimingTrace::Now() : 0;
            if (n < 0) break;
            if (n != sizeof(MotionGeneratorCommand)) continue;
            MotionGeneratorCommand command{};
            std::memcpy(&command, buf.data(), sizeof(command));
            if (!session_->Active() || command.protocol_version!=SERVO_PROTOCOL_VERSION ||
                command.session_id!=session_->id) continue;
            if (session_->client_ipv4 && addr.address() != session_->client_ipv4) continue;
            if (peer_bound && (addr.address()!=peer.address() || addr.port()!=peer.port())) continue;
            if (command.message_id == 0) {
                // 查询只登记周期状态地址，不绑定运动端口或更新通信看门狗。
                // 后续其他端口的查询可单次回复，不能改写已登记的推送地址。
                if (!stream_ready) {
                    stream_peer = addr;
                    stream_ready = true;
                }
                reply_addr = addr;
                reply = true;
                continue;
            }
            ServoTimingTrace::UdpRecord timing{};
            if (timing_trace_) {
                timing.message_id = command.message_id;
                timing.receive_ns = receive_ns;
            }
            if (command.message_id <= latest_message_id) {
                if (timing_trace_) {
                    timing.disposition = 3;
                    timing_trace_->PushUdp(timing);
                }
                continue;
            }
            if (!peer_bound) {
                peer=addr; peer_bound=true;
                peer_=addr; peer_bound_=true; // 仅接收线程写，Abort join 后读取。
                stream_peer = addr;
                stream_ready = true;
            }
            latest_message_id = command.message_id;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if (timing_trace_) timing.locked_ns = ServoTimingTrace::Now();
                if (!running_.load()) break;
                if (stopping_ || fault_ != Result::NoError) {
                    // 停止期间仍回复状态，但永不替换已锁存的运动目标。
                    if (timing_trace_) {
                        timing.disposition = 2;
                        timing_trace_->PushUdp(timing);
                    }
                    continue;
                }
                if (!has_new_cmd_) first_cmd_ = command;
                cmd_ = command;
                has_new_cmd_ = true;
                last_received_at_ = Clock::now();
                if (timing_trace_) timing.ready_ns = ServoTimingTrace::Now();
            }
            if (timing_trace_) {
                timing.published_ns = ServoTimingTrace::Now();
                timing.disposition = 1;
                timing_trace_->PushUdp(timing);
            }
        }
        // 同批次正序号包可能刚绑定端点，不把先前查询回复发给另一个端口。
        if (reply && peer_bound && (reply_addr.address() != peer.address() ||
                                   reply_addr.port() != peer.port())) reply = false;
        uint64_t notifications = 0;
        const bool cycle_ready = read(cycle_event_fd_, &notifications, sizeof(notifications)) ==
                                 sizeof(notifications);
        if (running_.load() && (reply || (cycle_ready && stream_ready))) {
            RobotState state = buildRobotState();
            if (reply) {
                sock_.send_to(&state, sizeof(state), reply_addr);
                if (stream_ready && reply_addr.address() == stream_peer.address() &&
                    reply_addr.port() == stream_peer.port()) sent_cycle = state.cycle;
            }
            if (cycle_ready && stream_ready && state.cycle > sent_cycle) {
                sock_.send_to(&state, sizeof(state), stream_peer);
                sent_cycle = state.cycle;
            }
        }
        if (running_.load()) {
            // 两种事件均立即唤醒，保持原有 1 ms 有界等待和停止回收行为。
            pollfd ready[]{{sock_.handle(), POLLIN, 0}, {cycle_event_fd_, POLLIN, 0}};
            ::poll(ready, 2, 1);
        }
    }
}

// ============================================================================
// 矩阵转换辅助
// ============================================================================

Frame MoveServo::matrixToFrame(const std::array<double, 16>& mat) {
    // mat 是 row-major 4x4 齐次变换矩阵
    // rocos::Rotation 的 9 元素构造使用 row-major 顺序
    // mat[i*4 + j] 对应第 i 行、第 j 列
    rocos::Rotation R(mat[0], mat[1], mat[2],  // 第 0 行: (0,0), (0,1), (0,2)
                      mat[4], mat[5], mat[6],  // 第 1 行: (1,0), (1,1), (1,2)
                      mat[8], mat[9], mat[10]  // 第 2 行: (2,0), (2,1), (2,2)
    );
    rocos::Vector p(mat[3], mat[7], mat[11]);  // 平移: (0,3), (1,3), (2,3)
    return {R, p};
}

void MoveServo::frameToMatrix(const Frame& f, std::array<double, 16>& mat) {
    mat.fill(0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat[static_cast<size_t>(i * 4 + j)] =
                f.M(static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        }
    }
    mat[3] = f.p.x();   // row 0, col 3
    mat[7] = f.p.y();   // row 1, col 3
    mat[11] = f.p.z();  // row 2, col 3
    mat[15] = 1.0;      // row 3, col 3
}

}  // namespace rocos
