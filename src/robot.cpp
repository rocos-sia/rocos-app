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
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "robot.hpp"

#include <yaml-cpp/yaml.h>

#include <boost/sml.hpp>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <type_traits>
#include <kdl_parser/kdl_parser.hpp>  // 用于将urdf文件解析为KDL::Tree
#include <sstream>

#include "cartesian_impedance_controller.hpp"
#include "hardware.hpp"
#include "joint_admittance_controller.hpp"
#include "joint_impedance_controller.hpp"
#include "model.hpp"
#include "move_circle.hpp"
#include "move_circle_offline.hpp"  // v3.0 离线圆弧规划 + 全轨迹 IK 验证版本
#include "move_hold.hpp"
#include "move_jog.hpp"
#include "move_joint.hpp"
#include "move_line.hpp"          // v3.0 保留，原有在线规划版本
#include "move_line_offline.hpp"  // v3.0 离线规划 + 全轨迹 IK 验证版本
#include "move_null_jog.hpp"
#include "move_svd_jog.hpp"
#include "move_servo.hpp"
#include "position_controller.hpp"
#include "zero_force_drag_controller.hpp"

namespace {
    // 状态定义
    class ERROR_STATE {
    }; // 0 错误状态，任何状态发生错误都转到这个状态[初始状态]
    class IDLE {
    }; // 1 空闲状态，机器人传感器与执行器就绪，等待使能命令
    class STOPPED {
    }; // 2 停止状态，机器人上使能，不会动
    class RUNNING {
    }; // 3 运行状态，机器人正在执行运动
    class PAUSED {
    }; // 4 暂停状态，机器人暂停在当前位置，等待继续或停止命令
    class SERVOING {
    }; // 5 伺服状态，用于高速udp伺服指令发送

    // class IDENTIFYING {};  // 动力学参数辨识状态，从STOPPED进入，辨识完成回到STOPPED

    // 中间状态定义
    class ENABLING {
    }; // 6  正在上使能状态
    class DISABLING {
    }; // 7  正在下使能状态
    class STARTING {
    }; // 8  正在启动状态，机器人正在启动
    class STOPPING {
    }; // 9  正在停止状态，机器人正在停止
    class PAUSING {
    }; // 10 正在暂停状态
    class RESUMING {
    }; // 11 正在恢复状态
    class RESETTING {
    }; // 12 正在复位状态，机器人正在复位

    // 机器人事件定义
    struct EventRunning {
    }; // 机器人开始运行事件
    struct EventAtTarget {
    }; // 机器人到达目标事件
    struct EventStopped {
    }; // 机器人已停止事件
    struct EventEnabled {
    }; // 机器人已上使能事件
    struct EventDisabled {
    }; // 机器人已下使能事件
    struct EventErrorOccurred {
    }; // 发生错误
    struct EventStartFailedReq {
    }; // 启动失败请求
    struct EventPauseFailed { //TODO: 暂停失败很危险，应该删除 by think
    };

    struct EventSuccessed {
    };

    // 指令事件定义
    struct EventEnableReq {
    }; // 上使能请求
    struct EventDisableReq {
    }; // 下使能请求
    struct EventStartReq {
    }; // 启动请求
    struct EventStopReq {
    }; // 停止请求
    struct EventPauseReq {
    }; // 暂停请求
    struct EventResumeReq {
    }; // 继续请求
    struct EventServoReq {
    }; // 伺服请求
    struct EventResetReq {
    }; // 恢复请求

    std::string joinNames(const std::vector<std::string> &names) {
        std::ostringstream oss;
        for (size_t i = 0; i < names.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << names[i];
        }
        return oss.str();
    }

    std::string joinDriveIds(const std::vector<int32_t> &ids) {
        std::ostringstream oss;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i > 0) oss << ", ";
            oss << ids[i];
        }
        return oss.str();
    }

    rocos::Twist rotateTwist(const rocos::Twist &twist, const KDL::Rotation &rotation) {
        return rocos::Twist(rotation * twist.vel, rotation * twist.rot);
    }

    namespace sml = boost::sml;

    const auto action_stopped = [](rocos::Robot &robot) { robot.on_fsm_stopped(); };
    const auto action_starting = [](rocos::Robot &robot) { robot.on_fsm_starting(); };
    const auto action_running = [](rocos::Robot &robot) { robot.on_fsm_running(); }; //TODO：目前没有任何处理
    const auto action_pausing = [](rocos::Robot &robot) { robot.on_fsm_pausing(); };
    const auto action_resuming = [](rocos::Robot &robot) { robot.on_fsm_resuming(); };
    const auto action_stopping = [](rocos::Robot &robot) { robot.on_fsm_stopping(); };
    const auto action_resetting = [](rocos::Robot &robot) { robot.on_fsm_resetting(); };
    const auto action_enabling = [](rocos::Robot &robot) { robot.on_fsm_enabling(); };
    const auto action_disabling = [](rocos::Robot &robot) { robot.on_fsm_disabling(); };
    const auto action_servoing = [](rocos::Robot &robot) { robot.on_fsm_servoing(); };
    const auto action_error = [](rocos::Robot &robot) { robot.on_fsm_error(); }; //TODO: 进入错误状态时的必要处理

    struct StateMachine {
        auto operator()() const noexcept {
            using namespace sml;
            return make_transition_table(
                // 初始化
                *state<class ERROR_STATE> + event<EventResetReq> = state<class RESETTING>, // 0 ERROR_STATE->RESETTING
                // state<class ERROR_STATE> + event<EventStopReq> = state<class STOPPING>, //   ERROR_STATE->STOPPING
                state<class ERROR_STATE> + sml::on_entry<_> / action_error, //   ERROR_STATE->RESETTING

                state<class IDLE> + event<EventEnableReq> = state<class ENABLING>, // 1 IDLE->ENABLING

                state<class STOPPED> + sml::on_entry<_> / action_stopped, // 2 STOPPED->DISABLING
                state<class STOPPED> + event<EventDisableReq> = state<class DISABLING>, // 2 STOPPED->DISABLING
                state<class STOPPED> + event<EventStartReq> = state<class STARTING>, //   STOPPED->STARING
                state<class STOPPED> + event<EventServoReq> = state<class SERVOING>, //   STOPPED->SERVOING

                state<class RUNNING> + sml::on_entry<_> / action_running, // 3 RUNNING
                state<class RUNNING> + event<EventStopped> = state<class STOPPED>,
                //todo     EventAtTarget不是必要的应该直接success
                state<class RUNNING> + event<EventAtTarget> = state<class STOPPED>,
                state<class RUNNING> + event<EventSuccessed> = state<class STOPPED>, //   RUNNING->STOPPED

                state<class RUNNING> + event<EventPauseReq> = state<class PAUSING>, //   RUNNING->PAUSING
                state<class RUNNING> + event<EventStopReq> = state<class STOPPING>, //   RUNNING->STOPPING

                state<class PAUSED> + event<EventResumeReq> = state<class RESUMING>, // 4 PAUSED->RESUMING
                state<class PAUSED> + event<EventStopReq> = state<class STOPPING>, //   PAUSED->STOPPING

                state<class SERVOING> + sml::on_entry<_> / action_servoing, // 5 SERVOING
                state<class SERVOING> + event<EventStopReq> = state<class STOPPING>, //   SERVOING->STOPPING
                state<class SERVOING> + event<EventSuccessed> = state<class STOPPED>, //   SERVOING->STOPPED（安全兜底）

                state<class ENABLING> + sml::on_entry<_> / action_enabling, // 6 ENABLING
                state<class ENABLING> + event<EventEnabled> = state<class STOPPED>, // 6 ENABLING->STOPPED

                state<class DISABLING> + sml::on_entry<_> / action_disabling, // 7 DISABLING
                state<class DISABLING> + event<EventDisabled> = state<class IDLE>, //   DISABLING->IDLE


                state<class STARTING> + sml::on_entry<_> / action_starting, // 8 STARTING
                state<class STARTING> + event<EventAtTarget> = state<class STOPPED>,
                state<class STARTING> + event<EventStartFailedReq> = state<class STOPPED>,
                state<class STARTING> + event<EventRunning> = state<class RUNNING>, //   STARTING->RUNNING

                state<class STOPPING> + sml::on_entry<_> / action_stopping, // 9 STOPPING
                state<class STOPPING> + event<EventStopped> = state<class STOPPED>,
                state<class STOPPING> + event<EventSuccessed> = state<class STOPPED>, //   STOPPING->STOPPED

                state<class PAUSING> + sml::on_entry<_> / action_pausing, // 10 PAUSING->RESUMING
                // TODO                这个地方应该是success
                state<class PAUSING> + event<EventStopped> = state<class PAUSED>,
                state<class PAUSING> + event<EventStopReq> = state<class STOPPING>,
                state<class PAUSING> + event<EventPauseFailed> = state<class RUNNING>, //TODO：暂停失败退回RUNNING是否合适？
                state<class PAUSING> + event<EventSuccessed> = state<class PAUSED>, //TODO: 为什么需要Successed
                //   PAUSING->PAUSED

                state<class RESUMING> + sml::on_entry<_> / action_resuming, // 11 RESUMING
                state<class RESUMING> + event<EventRunning> = state<class RUNNING>,
                state<class RESUMING> + event<EventStopReq> = state<class STOPPING>,
                state<class RESUMING> + event<EventSuccessed> = state<class RUNNING>, //    RESUMING->RUNNING

                state<class RESETTING> + sml::on_entry<_> / action_resetting, // 12 RESETTING
                state<class RESETTING> + event<EventDisabled> = state<class IDLE>, // 启动初始化完成后保持下使能空闲态
                state<class RESETTING> + event<EventEnabled> = state<class STOPPED>, //    RESETTING->STOPPED


                // ANY STATE JUMP TO ERROR_STATE
                state<class ERROR_STATE> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 0
                state<class IDLE> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 1
                state<class STOPPED> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 2
                state<class RUNNING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 3
                state<class PAUSED> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 4
                state<class SERVOING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 5
                state<class ENABLING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 6
                state<class DISABLING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 7
                state<class STARTING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 8
                state<class STOPPING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 9
                state<class PAUSING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 10
                state<class RESUMING> + event<EventErrorOccurred> = state<class ERROR_STATE>, // 11
                state<class RESETTING> + event<EventErrorOccurred> = state<class ERROR_STATE> // 12
            );
        }
    };
} // namespace

namespace rocos {
    class Robot::Impl {
    public:
        explicit Impl(Robot &owner) : sm_{owner} {
        }

        // 运动线程与 HTTP 线程都会触发事件，必须串行化处理。
        // 用递归锁：action 回调（on_entry）内会再次调用 process_event（如 ENABLING
        // 进入时 on_fsm_enable 再触发 EventSuccess），属同线程重入，普通锁会死锁。
        template<typename Event>
        bool process_event(const Event &event) {
            // 外部启动/使能等命令与工具事务互斥；实时状态读取和故障事件不等待磁盘。
            std::unique_lock<std::recursive_mutex> command_lock(command_mtx_, std::defer_lock);
            if constexpr (std::is_same_v<Event, EventStartReq> || std::is_same_v<Event, EventServoReq> ||
                          std::is_same_v<Event, EventResumeReq> || std::is_same_v<Event, EventEnableReq> ||
                          std::is_same_v<Event, EventDisableReq> || std::is_same_v<Event, EventResetReq>) {
                command_lock.lock();
            }
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            return sm_.process_event(event);
        }

        template<typename TState>
        bool is(const TState &s) const {
            std::lock_guard<std::recursive_mutex> lock(mtx_);
            return sm_.is(s);
        }

        std::unique_lock<std::recursive_mutex> lock() const { return std::unique_lock<std::recursive_mutex>(command_mtx_); }
        std::unique_lock<std::recursive_mutex> lockState() const { return std::unique_lock<std::recursive_mutex>(mtx_); }

    private:
        sml::sm<StateMachine> sm_; // 状态机实例
        mutable std::recursive_mutex command_mtx_; // 工具事务与外部状态切换命令的门闩
        mutable std::recursive_mutex mtx_; // 保护状态机的并发访问（允许同线程重入）
    };

#pragma region 状态机action处理函数

    void Robot::on_fsm_enabling() {
        log_ptr_->info("Robot enable requested.");

        setEnabled();

        if (IsEnabled()) {
            startControlThread();

            log_ptr_->info("Robot enable confirmed; transitioning to STOPPED.");
            impl_->process_event(EventEnabled{});
        } else {
            log_ptr_->error("Robot enable was not confirmed; transitioning to ERROR_STATE.");
            impl_->process_event(EventErrorOccurred{});
        }
    }

    void Robot::on_fsm_disabling() {
        log_ptr_->info("Robot disable requested.");

        setDisabled();
        if (IsDisabled()) {
            log_ptr_->info("Robot disable confirmed; transitioning to IDLE.");
            impl_->process_event(EventDisabled{});
        } else {
            log_ptr_->error("Robot disable was not confirmed; transitioning to ERROR_STATE.");
            impl_->process_event(EventErrorOccurred{});
        }
    }

    void Robot::on_fsm_starting() {
        // 此处确认启动成功并转入 RUNNING。
        auto rc = data_ready_callback_();
        if (rc != Result::NoError) {
            if (rc == Result::PlanFinished) {
                log_ptr_->info("Robot is already at the requested target; no motion is required.");
                impl_->process_event(EventAtTarget{});
                return;
            }

            Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "Motion start failed: result={}, state={}, enabled={}; returning to STOPPED.",
                            static_cast<int>(rc), GetStateString(), IsEnabled());
            impl_->process_event(EventStartFailedReq{});


            return;
        }


        // 控制线程已在 on_fsm_enable 中创建，此处直接复用
        impl_->process_event(EventRunning{});
    }

    void Robot::on_fsm_running() {
        emitLogEvent(LogEventType::EnteredRunning, LogModule::Robot, 0, -1, 0, 0, 0, LogLevel::Info);

    }

    void Robot::on_fsm_stopping() {

        Result rc = Result::Fatal;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            rc = executor->Stop();
        }
        if (rc == Result::NoError) {
            log_ptr_->info("Motion stop request accepted.");
        } else {
            Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "Motion stop request failed: result={}, state={}; transitioning to ERROR_STATE.",
                            static_cast<int>(rc), GetStateString());
            impl_->process_event(EventErrorOccurred{});
        }

    }

    void Robot::on_fsm_pausing() {
        // PAUSING 进入：确认暂停并转入 PAUSED。
        Result rc = Result::Fatal;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            rc = executor->Pause();
        }
        if (rc == Result::NoError) {
            log_ptr_->info("Motion pause request accepted.");
        } else {
            //TODO: 暂停失败  这块我感觉不允许失败 by think
            Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "Motion pause request failed: result={}, state={}; returning to RUNNING.",
                            static_cast<int>(rc), GetStateString());
            impl_->process_event(EventPauseFailed{});
        }

    }

    void Robot::on_fsm_resuming() {
        // CONTINUING 进入：确认继续并转回 RUNNING。
        Result rc = Result::Fatal;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            rc = executor->Resume();
        }
        if (rc == Result::NoError) {
            log_ptr_->info("Motion resume request accepted; transitioning to RUNNING.");
            impl_->process_event(EventRunning{});
        } else {
            Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "Motion resume request failed: result={}, state={}.",
                            static_cast<int>(rc), GetStateString());
        }

    }
    void Robot::on_fsm_identify() {

    }

    void Robot::on_fsm_error() {
      if (!fsm_error_entered_once_) {
        // FSM 的初始状态就是 ERROR_STATE，构造时 on_entry 会进入一次；
        // 这不是真正的错误，只做标记，避免打印误导性的“机器人进入ERROR”。
        fsm_error_entered_once_ = true;
        return;
      }
      emitLogEvent(LogEventType::EnteredError, LogModule::Robot, 0);
      {
        std::lock_guard<std::mutex> lock(mtx_);
        if (motion) {
          motion->Stop();  // 停止当前 motion，确保 MoveServo 的 UDP 线程等资源被释放
        }
      }
    }
    void Robot::on_fsm_stopped() {
      emitLogEvent(LogEventType::EnteredStopped, LogModule::Robot, 0, -1, 0, 0, 0, LogLevel::Info);

      {
        std::lock_guard<std::mutex> lock(mtx_);

        motion = std::make_unique<MoveHold>(hardware->GetPosition());
        executor->SwitchMotion(motion.get());

      }

    }

    void Robot::on_fsm_resetting() {
        log_ptr_->info("Robot entered RESETTING; clearing hardware faults and checking enable state.");

        bool controller_reset_failed = false;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (motion) {
                const Result motion_reset = motion->Reset();
                if (motion_reset != Result::NoError && motion_reset != Result::PlanFinished) {
                    Logger::logCode(LogLevel::Warn, "Robot", static_cast<int>(motion_reset), "ResetFault motion reset returned result={}.", static_cast<int>(motion_reset));
                }
                motion.reset();
                if (executor) {
                    executor->SwitchMotion(nullptr);
                }
            }

            if (controller && !controller->Reset()) {
                log_ptr_->error("ResetFault controller reset failed; remaining in ERROR_STATE.");
                controller_reset_failed = true;
            }
        }
        if (controller_reset_failed) {
            impl_->process_event(EventErrorOccurred{});
            return;
        }

        if (hardware) {
            const Result clear_result = hardware->ClearFault(); //TODO： 这一步会下使能
            if (clear_result != Result::NoError &&
                clear_result != Result::FunctionNotSupported) {
                Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(clear_result), "ResetFault hardware fault clear failed: result={}.", static_cast<int>(clear_result));
                impl_->process_event(EventErrorOccurred{});
                return;
            }
            if (clear_result == Result::FunctionNotSupported) {
                log_ptr_->warn("Hardware does not implement ClearFault; continuing the enable-state check.");
            }
        }

        if (IsEnabled()) {
            log_ptr_->info("ResetFault completed and enable state confirmed; transitioning to STOPPED.");
            impl_->process_event(EventEnabled{}); // 模拟初始化成功事件
            startControlThread(); //TODO: 兼容设计，原本在Enabling中启动线程
        } else {
          log_ptr_->info("Robot initialization completed; remaining disabled in IDLE.");
          impl_->process_event(EventDisabled{});
        }
    }

    void Robot::on_fsm_servoing() {
        log_ptr_->info("Robot entered SERVOING.");
        if (data_ready_callback_) {
            Result rc = data_ready_callback_();
            if (rc != Result::NoError && rc != Result::PlanFinished) {
                Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "Servo motion initialization failed: result={}.", static_cast<int>(rc));
                impl_->process_event(EventErrorOccurred{});
            }
        }
    }

#pragma endregion

    void Robot::startControlThread() {

      stopControlThread();

        control_thread_ = std::thread([this]() {
            bindRtLogProducer(rt_log_producer_.get());
            const auto monotonic_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            setRtLogClock(static_cast<uint64_t>(logNowMicros()), monotonic_us);
            uint64_t cycle = 0;
            while (IsControlActive()) {
                setRtLogCycle(cycle, 0); // Executor 的现有性能采样更新实际周期时间。
                ++cycle;
                RunCycle();
                waitControlCycle();
            }
            bindRtLogProducer(nullptr);
        });
    }

    void Robot::stopControlThread() {
        joinControlThreadIfJoinable();
    }

    void Robot::joinControlThreadIfJoinable() {
        if (!control_thread_.joinable()) {
            return;
        }
        if (control_thread_.get_id() == std::this_thread::get_id()) {
            emitLogEvent(LogEventType::ThreadJoinSkipped, LogModule::Robot, 0, -1, 0, 0, 0, LogLevel::Warn);
            return;
        }
        control_thread_.join();
    }

    void Robot::jointBinding() {
      // --- JointBinding 初始化：在 controller 创建之前完成轴绑定 ---
        const auto model_joint_names = model->GetJointNames();
        const auto hardware_drive_ids = hardware->GetDriveIds();

        log_ptr_->info(
            "Joint binding initialization started: model joints={} [{}], hardware drives={} [{}], "
            "yaml={}",
            model_joint_names.size(), joinNames(model_joint_names),
            hardware_drive_ids.size(), joinDriveIds(hardware_drive_ids),
            joint_binding_path_);

        joint_binding_ = std::make_unique<JointBinding>();
        Result rc =
            joint_binding_->Configure(model_joint_names, hardware_drive_ids);
        if (rc != Result::NoError) {
          log_ptr_->error(
              "Joint binding configuration failed: model joints={}, hardware drives={}.",
              model_joint_names.size(), hardware_drive_ids.size());
          throw std::runtime_error(
              "JointBinding Configure failed: model joints=" +
              std::to_string(model_joint_names.size()) +
              " hw drives=" + std::to_string(hardware_drive_ids.size()));
        }

        // 检查 joint_binding.yaml 是否存在
        std::ifstream binding_file(joint_binding_path_);
        if (joint_binding_path_.empty()) {
          // 新结构只认 hardware 中的 joint_name，不按列表顺序推断。
          for (const auto id : hardware_drive_ids) {
            const auto name = hardware->getJointName(id);
            if (name.empty() || joint_binding_->HasBindingForJoint(name) ||
                joint_binding_->Bind(name, id) != Result::NoError) {
              throw std::runtime_error("Invalid hardware joint_name binding: " + name);
            }
          }
        } else if (binding_file.good()) {
          log_ptr_->info("Loading explicit joint binding from YAML: {}",
                         joint_binding_path_);
          rc = joint_binding_->LoadFromYaml(joint_binding_path_);
          if (rc != Result::NoError) {
            log_ptr_->error("Joint binding YAML load failed: {}",
                            joint_binding_path_);
            throw std::runtime_error("JointBinding LoadFromYaml failed: " +
                                     joint_binding_path_);
          }
        } else {
          // 无 YAML 时要求 model joint 数量 == hardware drive 数量
          if (model_joint_names.size() != hardware_drive_ids.size()) {
            log_ptr_->error(
                "Joint binding YAML is missing and joint counts differ: yaml={}, model "
                "joints={}, hw drives={}",
                joint_binding_path_, model_joint_names.size(),
                hardware_drive_ids.size());
            throw std::runtime_error(
                "joint_binding.yaml missing and joint/drive count mismatch: "
                "model=" +
                std::to_string(model_joint_names.size()) +
                " hw=" + std::to_string(hardware_drive_ids.size()));
          }
          log_ptr_->warn(
              "Joint binding YAML is missing; using sequential binding: model joints={}, hardware "
              "drives={}",
              model_joint_names.size(), hardware_drive_ids.size());
          // 自动生成默认顺序绑定
          for (size_t i = 0; i < model_joint_names.size(); ++i) {
            joint_binding_->Bind(model_joint_names[i], hardware_drive_ids[i]);
          }
        }

        rc = joint_binding_->Validate();
        if (rc != Result::NoError) {
          log_ptr_->error("Joint binding validation failed: yaml={}",
                          joint_binding_path_);
          throw std::runtime_error("JointBinding Validate failed");
        }

        const auto model_index_to_drive_id =
            joint_binding_->GetModelIndexToDriveIds();
        log_ptr_->info(
            "Applying hardware joint binding: model_index_to_drive_id=[{}]",
            joinDriveIds(model_index_to_drive_id));
        rc = hardware->SetJointBinding(model_index_to_drive_id);
        if (rc != Result::NoError) {
          log_ptr_->error(
              "Hardware joint binding failed: model_index_to_drive_id=[{}]",
              joinDriveIds(model_index_to_drive_id));
          throw std::runtime_error("Hardware SetJointBinding failed");
        }

        log_ptr_->info(
            "Joint binding initialization completed: model joints={}, hardware drives={}, "
            "binding={}",
            model_joint_names.size(), hardware_drive_ids.size(),
            joint_binding_path_);
    }

    Robot::Robot() : Robot(Config{}) {}

    Robot::Robot(const Config &config) : frames_(config.frames, config.tool_library, config.id), log_ptr_(Logger::getInstance("Robot")), impl_(std::make_unique<Impl>(*this)) {
        executor = std::make_unique<Executor>();

        joint_binding_path_ = config.joint_binding;
        load_params_path_ = config.payload;
        urdf_path_ = config.urdf;
        model = std::make_unique<Model>(config.urdf, config.base_link, config.tip_link);
        if (config.composed_configuration && config.master_state != "op") {
            throw std::runtime_error("Robot control requires master.state=op; other states are master maintenance modes");
        }
        hardware = config.hardware ? std::make_unique<Hardware>(*config.hardware, config.ecat_id)
                                   : std::make_unique<Hardware>(config.hardware_config, config.ecat_id);
        if (config.master_cycle_us > 0 && hardware->GetDt() != static_cast<uint32_t>(config.master_cycle_us)) {
            throw std::runtime_error("Running master cycle differs from robot.master.cycle; apply the exported master configuration first");
        }

        jointBinding();
        if (!config.composed_configuration && std::filesystem::exists(config.frames) && frames_.load() != Result::NoError) {
            throw std::runtime_error("Cannot load configured coordinate frames: " + config.frames);
        }
        if (config.composed_configuration) {
            if (config.tool) frames_.initializePackage(*config.tool);
        } else frames_.reloadPackages(false);

        // 尝试从配置文件加载已知负载参数（存在则跳过辨识）
        load_params_metadata_ = YAML::Clone(config.payload_metadata);
        load_params_overridden_ = config.payload_overridden;
        const Result load_rc = config.initial_payload
            ? SetLoadParameters(config.initial_payload->first, config.initial_payload->second)
            : (config.composed_configuration ? Result::ResourceUnavailable : LoadLoadParameters());
        if (load_rc == Result::NoError) {
            log_ptr_->info("Persisted payload parameters loaded; zero-force drag will skip payload identification.");
        }

        dt_=hardware->GetDt()/1000000.0; // 将微秒转换为秒

        executor->SwitchHardware(hardware.get());

        motion = std::make_unique<MoveHold>(hardware->GetPosition());
        executor->SwitchMotion(motion.get());

        const Result work_mode_result = SetWorkMode("position");
        if (work_mode_result != Result::NoError) {
          Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(work_mode_result), "Default robot work mode initialization failed: result={}.", static_cast<int>(work_mode_result));
          throw std::runtime_error("Robot default work mode initialization failed");
        }

        log_ptr_->info("Robot initialization started.");
        impl_->process_event(EventResetReq{}); // 进入初始化状态
    }

    Robot::~Robot() {
        if (control_thread_.joinable() &&
            !impl_->is(sml::state<class IDLE>) &&
            !impl_->is(sml::state<class ERROR_STATE>)) {
            impl_->process_event(EventErrorOccurred{});
        }
        joinControlThreadIfJoinable();

        // Delete logger pointer
        if (log_ptr_) {
            log_ptr_->flush();
            log_ptr_.reset();
        }

        // Delete FSM pack
        impl_.reset();
    }

    std::string Robot::GetStateString() const {
        if (impl_->is(sml::state<class IDLE>)) {
            return "IDLE";
        } else if (impl_->is(sml::state<class ENABLING>)) {
            return "ENABLING";
        } else if (impl_->is(sml::state<class DISABLING>)) {
            return "DISABLING";
        } else if (impl_->is(sml::state<class STARTING>)) {
            return "STARTING";
        } else if (impl_->is(sml::state<class STOPPING>)) {
            return "STOPPING";
        } else if (impl_->is(sml::state<class PAUSING>)) {
            return "PAUSING";
        } else if (impl_->is(sml::state<class RESUMING>)) {
            return "RESUMING";
        } else if (impl_->is(sml::state<class RESETTING>)) {
            return "RESETTING";
        } else if (impl_->is(sml::state<class RUNNING>)) {
            return "RUNNING";
        } else if (impl_->is(sml::state<class PAUSED>)) {
            return "PAUSED";
        } else if (impl_->is(sml::state<class STOPPED>)) {
            return "STOPPED";
        } else if (impl_->is(sml::state<class SERVOING>)) {
            return "SERVOING";
        } else if (impl_->is(sml::state<class ERROR_STATE>)) {
            return "ERROR_STATE";
        } else {
            return "UNKNOWN_STATE";
        }
    }

    Robot::RobotStateSnapshot Robot::GetRobotStateSnapshot() const {
        RobotStateSnapshot snap;

        // 时间戳
        snap.timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // FSM 状态
        snap.state_string = GetStateString();
        snap.is_enabled = IsEnabled();
        snap.is_running = IsRunning();
        snap.control_active = IsControlActive();
        snap.motion_busy = IsMotionBusy();
        {
            std::lock_guard<std::mutex> lock(mtx_);
            snap.work_mode = work_mode_;
        }

        // 关节状态
        const int n = getJointNum();
        snap.joints.reserve(static_cast<size_t>(n));
        const auto positions = hardware ? hardware->GetPosition() : JntArray();
        for (int i = 0; i < n; ++i) {
            RobotStateSnapshot::JointState js;
            js.id = i;
            js.name = getJointName(i);
            js.position = static_cast<unsigned int>(i) < positions.rows() ? positions(i) : 0.0;
            js.velocity = getJointVelocity(i);
            js.torque = getJointTorque(i);
            js.load_torque = getJointLoadTorque(i);
            js.status = getJointStatus(i);
            snap.joints.push_back(std::move(js));
        }

        // 法兰末端位姿
        const auto flange_result = ComputeFlangePose(positions, snap.flange);

        // 激活的坐标系
        const auto frames = frames_.snapshot();
        snap.active_tool_frame_name = frames->active_tool;
        snap.active_object_frame_name = frames->active_object;
        if (frames->tools.count(frames->active_tool)) snap.active_tool_frame = frames->tools.at(frames->active_tool);
        if (frames->packages.count(frames->active_tool)) snap.active_tool_package = frames->packages.at(frames->active_tool);
        if (frames->objects.count(frames->active_object)) snap.active_object_frame = frames->objects.at(frames->active_object);
        snap.tcp_valid = flange_result == Result::NoError &&
                         FrameRegistry::resolveTcp(*frames, snap.flange, snap.tcp) == Result::NoError;

        // 硬件摘要
        snap.joint_num = n;
        if (hardware) {
            snap.hardware_state = static_cast<int>(hardware->GetState());
        }

        return snap;
    }

    Result Robot::SetEnabled() {
        log_ptr_->info("Enable command received.");

        //TODO: 代码清理 by think
        // if (impl_->is(sml::state<class IDLE>) ||
        //     impl_->is(sml::state<class ERROR_STATE>)) {
        //     stopControlThread();
        // }

        if (!impl_->process_event(EventEnableReq{})) {
            log_ptr_->error("Enable command rejected in the current robot state.");
            return Result::JointStateError;
        }

        return Result::NoError;
    }

    Result Robot::SetDisabled() {
        log_ptr_->info("Disable command received.");
        if (!impl_->process_event(EventDisableReq{})) {
            log_ptr_->error("Disable command rejected in the current robot state.");
            return Result::JointStateError;
        }

        //TODO: 代码清理 by think
        // if (impl_->is(sml::state<class IDLE>) ||
        //     impl_->is(sml::state<class ERROR_STATE>)) {
        //     stopControlThread();
        // }

        return Result::NoError;
    }

    Result Robot::ResetFault() {
        log_ptr_->info("ResetFault command received: state={}.", GetStateString());
        if (!impl_->is(sml::state<class ERROR_STATE>)) {
            log_ptr_->error("ResetFault rejected: state={} is not ERROR_STATE.", GetStateString());
            return Result::JointStateError;
        }

        // stopControlThread(); //TODO： 代码清理 by think

        if (!impl_->process_event(EventResetReq{})) {
            log_ptr_->error("ResetFault state transition rejected: state={}.", GetStateString());
            return Result::JointStateError;
        }

        if (impl_->is(sml::state<class STOPPED>)) {
            return Result::NoError;
        }

        log_ptr_->error("ResetFault did not reach STOPPED: state={}.", GetStateString());
        return Result::Fatal;
    }

    Result Robot::SetWorkMode(const std::string& mode) {
        // 仅允许在 IDLE 或 STOPPED 状态下切换控制器
        const bool is_switchable_state =
            impl_->is(sml::state<class IDLE>) ||
            impl_->is(sml::state<class STOPPED>) ||
            impl_->is(sml::state<class ERROR_STATE>); // 允许在 ERROR_STATE 下切换控制器，以便恢复
        if (!is_switchable_state) {
            log_ptr_->error("SetWorkMode requires IDLE or STOPPED: current state={}.",
                            GetStateString());
            return Result::IllegalParameter;
        }


        std::unique_ptr<ControllerInterface> new_controller;

        if (mode == "position") {
            new_controller = std::make_unique<PositionController>();
        } else if (mode == "jnt_imp") {
            new_controller = std::make_unique<JointImpedanceController>();
        } else if (mode == "jnt_admit_teach") {
            new_controller = std::make_unique<JointAdmittanceController>();
        } else if (mode == "cart_imp") {
            new_controller = std::make_unique<CartesianImpedanceController>();
        } else if (mode == "zero_force_drag") {
            auto zfd = std::make_unique<ZeroForceDragController>();
            // 若已预置负载参数，则跳过辨识直接进入拖动
            if (load_params_valid_) {
                zfd->SetLoadParameters(load_mass_, load_com_);
            }

            // 辨识完成后回写 Robot 层并落盘，供下次启动跳过辨识
            zfd->SetLoadParamsCallback([this](double mass, const KDL::Vector& com) {
                load_mass_ = mass;
                load_com_ = com;
                load_params_valid_ = true;
                SaveLoadParameters();
            });

            new_controller = std::move(zfd);
        } else if (mode == "ee_admit_teach") {
            log_ptr_->error("SetWorkMode is not implemented for mode='{}'.", mode);
            return Result::IllegalParameter;
        } else {
            log_ptr_->error("SetWorkMode received unknown mode='{}'.", mode);
            return Result::IllegalParameter;
        }

        Result rc = new_controller->SetHardware(hardware.get());
        if (rc != Result::NoError) {
            log_ptr_->error("SetWorkMode hardware binding failed: mode={}.", mode);
            return rc;
        }

        rc = new_controller->SetModel(model.get());
        if (rc != Result::NoError) {
            log_ptr_->error("SetWorkMode model binding failed: mode={}.", mode);
            return rc;
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);

            if (!executor->SwitchController(nullptr)) {
                log_ptr_->error("SetWorkMode could not clear the executor controller: mode={}.", mode);
                return Result::Fatal;
            }
            controller.reset();

            rc = new_controller->SetReady();
            if (rc != Result::NoError) {
                log_ptr_->error("SetWorkMode controller preparation failed: mode={}.", mode);

                auto fallback_controller = std::make_unique<PositionController>();
                Result fallback_rc = fallback_controller->SetHardware(hardware.get());
                if (fallback_rc != Result::NoError) {
                    log_ptr_->error("SetWorkMode fallback hardware binding failed.");
                    return rc;
                }

                fallback_rc = fallback_controller->SetModel(model.get());
                if (fallback_rc != Result::NoError) {
                    log_ptr_->error("SetWorkMode fallback model binding failed.");
                    return rc;
                }

                fallback_rc = fallback_controller->SetReady();
                if (fallback_rc != Result::NoError) {
                    log_ptr_->error("SetWorkMode fallback controller preparation failed.");
                    return rc;
                }

                controller = std::move(fallback_controller);
                if (!executor->SwitchController(controller.get())) {
                    log_ptr_->error("SetWorkMode fallback controller switch failed.");
                    return Result::Fatal;
                }
                work_mode_ = "position";
                return rc;
            }
            controller = std::move(new_controller);
            if (!executor->SwitchController(controller.get())) {
                log_ptr_->error("SetWorkMode executor controller switch failed: mode={}.", mode);
                return Result::Fatal;
            }
            work_mode_ = mode;
        }

        log_ptr_->info("Work mode changed to '{}'.", mode);
        return Result::NoError;
    }

    nlohmann::json Robot::GetImpedanceParams() const {
        nlohmann::json result;
        result["work_mode"] = work_mode_;

        // 关节空间参数
        nlohmann::json joint;
        joint["valid"] = false;
        if (work_mode_ == "jnt_imp") {
            auto* jnt_ctrl = dynamic_cast<JointImpedanceController*>(controller.get());
            if (jnt_ctrl != nullptr) {
                const auto& K = jnt_ctrl->GetStiffness();
                const auto& D = jnt_ctrl->GetDamping();
                nlohmann::json stiffness = nlohmann::json::array();
                nlohmann::json damping   = nlohmann::json::array();
                for (unsigned int i = 0; i < K.rows(); ++i) {
                    stiffness.push_back(K(i));
                    damping.push_back(D(i));
                }
                joint["stiffness"] = stiffness;
                joint["damping"]   = damping;
                joint["valid"]     = true;
            }
        }
        result["joint_space"] = joint;

        // 笛卡尔空间参数
        nlohmann::json cart;
        cart["valid"] = false;
        if (work_mode_ == "cart_imp") {
            auto* cart_ctrl = dynamic_cast<CartesianImpedanceController*>(controller.get());
            if (cart_ctrl != nullptr) {
                cart["translational_stiffness"] = cart_ctrl->GetTranslationalStiffness();
                cart["rotational_stiffness"]    = cart_ctrl->GetRotationalStiffness();
                cart["translational_damping"]   = cart_ctrl->GetTranslationalDamping();
                cart["rotational_damping"]      = cart_ctrl->GetRotationalDamping();
                cart["valid"] = true;
            }
        }
        result["cartesian_space"] = cart;

        return result;
    }


    Result Robot::SetLoadParameters(double mass, const Vector& com) {
        // 非法参数拒绝，避免把错误负载补偿下发到硬件；mass=0 表示无负载（合法）
        if (mass < 0.0 || !std::isfinite(mass) ||
            !std::isfinite(com.x()) || !std::isfinite(com.y()) ||
            !std::isfinite(com.z())) {
            log_ptr_->error("SetLoadParameters rejected invalid payload parameters: mass={} kg.", mass);
            return Result::ParameterNanOrInf;
        }

        load_mass_ = mass;
        load_com_ = com;
        load_params_valid_ = true;

        log_ptr_->info("SetLoadParameters: mass={:.3f} kg, com=({:.3f}, {:.3f}, {:.3f}) m",
                       mass, com.x(), com.y(), com.z());
        return Result::NoError;
    }

    Result Robot::LoadLoadParameters(const std::string& path) {
        const std::string file = path.empty() ? load_params_path_ : path;

        // 文件不存在时静默返回，视为"尚未辨识过负载"，走辨识流程
        std::ifstream in(file);
        if (!in.is_open()) {
            log_ptr_->info("Payload parameter file '{}' was not found; payload identification is required.", file);
            return Result::ResourceUnavailable;
        }
        in.close();

        try {
            YAML::Node root = YAML::LoadFile(file);
            if (!root["mass"]) {
                log_ptr_->error("Payload parameter file '{}' is missing the mass field.", file);
                return Result::IllegalParameter;
            }
            const double mass = root["mass"].as<double>();

            Vector com = Vector::Zero();
            if (root["com"]) {
                const YAML::Node com_node = root["com"];
                if (com_node.IsSequence() && com_node.size() >= 3) {
                    com = Vector(com_node[0].as<double>(),
                                 com_node[1].as<double>(),
                                 com_node[2].as<double>());
                } else {
                    log_ptr_->error("Payload parameter file '{}' contains an invalid center-of-mass field.", file);
                    return Result::IllegalParameter;
                }
            }

            const Result rc = SetLoadParameters(mass, com);
            if (rc != Result::NoError) {
                return rc;
            }
            log_ptr_->info("Payload parameters loaded from '{}': mass={:.3f} kg.", file, mass);
            return Result::NoError;
        } catch (const YAML::Exception& e) {
            log_ptr_->error("Payload YAML parse failed for '{}': {}", file, e.what());
            return Result::ResourceUnavailable;
        }
    }

    Result Robot::SaveLoadParameters(const std::string& path) const {
        const std::string file = path.empty() ? load_params_path_ : path;

        if (file == load_params_path_ && load_params_overridden_) {
            log_ptr_->error("Payload save conflicts with user.yaml overrides; edit the user configuration first.");
            return Result::ResourceUnavailable;
        }

        if (!load_params_valid_) {
            log_ptr_->error("Cannot save payload parameters: no valid payload parameters are available.");
            return Result::ResourceUnavailable;
        }

        if (file == load_params_path_ && load_params_metadata_.IsMap()) {
            const auto context = load_params_metadata_["context"];
            const auto state = frames_.snapshot();
            const auto tool = state->packages.find(state->active_tool);
            if (tool == state->packages.end() ||
                tool->second.name != context["tool"]["name"].as<std::string>() ||
                tool->second.version != context["tool"]["package_version"].as<std::string>() ||
                (context["tool"]["digest"] && tool->second.digest != context["tool"]["digest"].as<std::string>()) ||
                !KDL::Equal(tool->second.mount, ToolPackageFiles::pose(context["tool_install"]), 1e-9)) {
                log_ptr_->error("Payload save conflicts with the recorded tool installation context.");
                return Result::ResourceUnavailable;
            }
        }

        YAML::Node root = file == load_params_path_ && load_params_metadata_.IsMap()
            ? YAML::Clone(load_params_metadata_) : YAML::Node(YAML::NodeType::Map);
        root["mass"] = load_mass_;
        YAML::Node com;
        com.push_back(load_com_.x());
        com.push_back(load_com_.y());
        com.push_back(load_com_.z());
        root["com"] = com;

        std::ofstream out(file);
        if (!out.is_open()) {
            log_ptr_->error("Cannot open payload parameter file '{}' for writing.", file);
            return Result::ResourceUnavailable;
        }
        out << root;
        if (!out.good()) {
            log_ptr_->error("Writing payload parameter file '{}' failed.", file);
            return Result::ResourceUnavailable;
        }

        log_ptr_->info("Payload parameters saved to '{}': mass={:.3f} kg.",
                       file, load_mass_);
        return Result::NoError;
    }


    bool Robot::IsEnabled() const {
        if (hardware == nullptr) {
            return false;
        }
        auto state = hardware->GetState();
        return state == JntState::ENABLED;
    }

    bool Robot::IsDisabled() const {
        if (hardware == nullptr) {
            return false;
        }
        auto state = hardware->GetState();
        return state == JntState::DISABLED;
    }

    bool Robot::IsRunning() const {
        return impl_->is(sml::state<class RUNNING>);
    }

    bool Robot::IsControlActive() const {
        return impl_->is(sml::state<class ENABLING>)
               || impl_->is(sml::state<class RUNNING>)
               || impl_->is(sml::state<class PAUSING>)
               || impl_->is(sml::state<class PAUSED>)
               || impl_->is(sml::state<class RESUMING>)
               || impl_->is(sml::state<class STOPPING>)
               || impl_->is(sml::state<class DISABLING>)
               || impl_->is(sml::state<class STOPPED>)
               || impl_->is(sml::state<class STARTING>)
               || impl_->is(sml::state<class SERVOING>);
    }

    bool Robot::IsMotionBusy() const {
        return impl_->is(sml::state<class STARTING>)
            || (IsControlActive() && !impl_->is(sml::state<class STOPPED>));
    }

    Result Robot::WaitMove() {
        using namespace std::chrono_literals;

        std::this_thread::sleep_for(20ms);
        while (true) {
            if (impl_->is(sml::state<class ERROR_STATE>)) {
                return Result::Fatal;
            }
            if (impl_->is(sml::state<class IDLE>)) {
                return Result::NotEnabled;
            }
            if (!IsMotionBusy()) {
                return Result::NoError;
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    std::vector<Robot::JointInfo> Robot::GetJointInfo() const {
        std::vector<JointInfo> joint_infos;
        const auto *hw = dynamic_cast<const Hardware *>(hardware.get());
        if (hw == nullptr) return joint_infos;

        const auto &drives = hw->getConfig().drives;
        joint_infos.reserve(drives.size());
        for (const auto &drive : drives) {
            JointInfo info;
            info.id = drive.id;
            info.name = drive.joint_name;
            info.cnt_per_unit = drive.transform.cnt_per_unit;
            info.torque_per_unit = drive.transform.torque_per_unit;
            info.ratio = drive.transform.ratio;
            info.unit_name = drive.transform.user_unit_name;
            info.zero_offset = drive.transform.offset_pos_cnt;
            joint_infos.push_back(std::move(info));
        }
        return joint_infos;
    }

    void Robot::waitControlCycle() {
        hardware->WaitForSignal();
    }

    void Robot::setEnabled() {
        hardware->SetEnabled();
    }

    void Robot::setDisabled() {
        hardware->SetDisabled();
    }

    /////// Motion Command /////////////

    Result Robot::Start() {
        return Result::NoError;
    }

    // ============================================================================
    // RunCycle：控制循环主函数
    //
    // ============================================================================

    void Robot::RunCycle() {
      Result res = Result::NoError;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        res = executor->Update();
      }

      if (static_cast<int>(res) < 0) {
        emitLogEvent(LogEventType::CycleFailed, LogModule::Robot, static_cast<int>(res));
        impl_->process_event(EventErrorOccurred{});
      }
      else if (static_cast<int>(res) > 0) {
        emitLogEvent(LogEventType::CycleFinished, LogModule::Robot, static_cast<int>(res), -1, 0, 0, 0, LogLevel::Info);
        impl_->process_event(EventSuccessed{});
      }

    }

    // ============================================================================
    // MoveJ：关节空间点到点，一次性下发跑完即停
    // ============================================================================

    Result Robot::MoveJ(const JntArray &q_goal,
                        double v_limit, double a_limit, double j_limit) {
        auto command_lock = LockToolState();

        data_ready_callback_ = [this, q_goal, v_limit, a_limit, j_limit]() -> Result {
            const int n = getJointNum();
            JntArray q_start(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_start(i) = getJointPosition(i);

            auto new_motion = std::make_unique<MoveJoint>(
                q_start, q_goal, v_limit, a_limit, j_limit, dt_);


            Result rc = new_motion->Reset();
            if (rc != Result::NoError) {
                Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc),
                    "MoveJ planning failed: result={}, joint_count={}, velocity={} rad/s, acceleration={} rad/s^2, jerk={} rad/s^3, dt={} s.",
                    static_cast<int>(rc), q_goal.rows(), v_limit, a_limit, j_limit, dt_);
                return rc;
            }

            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(new_motion); // unique_ptr会自动清理旧对象，只需要在析构中处理资源清理即可
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;
        };
        if (impl_->process_event(EventStartReq{})) {
            // STOPPED → RUNNING
        } else {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            log_ptr_->error("MoveJ request rejected by the state machine: state={}, enabled={}, running={}.",
                            GetStateString(), IsEnabled(), IsRunning());
            if (impl_->is(sml::state<class IDLE>)) {
                log_ptr_->error("MoveJ requires an enabled robot; current state is IDLE.");
                return Result::NotEnabled;
            } else if (impl_->is(sml::state<class ERROR_STATE>)) {
                log_ptr_->error("MoveJ rejected in ERROR_STATE; reset the fault before requesting motion.");
                return Result::Fatal;
            } else {
                log_ptr_->error("MoveJ rejected in the current state: {}.", GetStateString());
                return Result::ConflictTaskRunning;
            }
        }


        return Result::NoError;
    }

    // ============================================================================
    // MoveL：笛卡尔直线，支持多工具坐标系
    // ============================================================================

    Result Robot::MoveL(const Frame &pose_goal,
                        const std::string &tool_name,
                        double v_limit, double a_limit, double j_limit) {
        auto command_lock = LockToolState();
        if (!tool_name.empty() && !HasToolFrame(tool_name)) {
            log_ptr_->error("Requested tool frame does not exist: {}", tool_name);
            return Result::ResourceUnavailable;
        }

        data_ready_callback_ = [this, pose_goal, tool_name, v_limit, a_limit, j_limit]() -> Result {
            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);

            Frame pose_flange;
            if (model) {
                const Result fk_result = model->ForwardKinematics(q_current, pose_flange);
                if (fk_result != Result::NoError) {
                    Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(fk_result), "MoveL start forward kinematics failed: result={}, joint_count={}, state={}.",
                                    static_cast<int>(fk_result), n, GetStateString());
                }
            } else {
                log_ptr_->error("MoveL start forward kinematics failed: robot model pointer is null.");
            }

            Frame T_tool;
            if (!tool_name.empty()) {
                Result rc = GetToolFrame(tool_name, T_tool);
                if (rc != Result::NoError) {
                    Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "MoveL tool frame lookup failed: tool='{}', result={}.",
                                    tool_name, static_cast<int>(rc));
                    return rc;
                }
            }

            // v3.0 替换为离线规划版本：Reset 中全轨迹 IK 验证，
            // 任一点逆解失败直接退回 Stopped，避免在线执行到一半才发现不可达。
            // 暂停/继续走 Jacobian 积分通道，保证在笛卡尔直线上减速和恢复。
            auto new_motion = std::make_unique<MoveLineOffline>(
                pose_flange, pose_goal * T_tool.Inverse(),
                model.get(),
                v_limit, a_limit, j_limit, dt_);

            new_motion->SetInitialJointPosition(q_current);  // IK 暖启动
            Result rc = new_motion->Reset();

            // ---- 原在线版本（保留以备回退） ----
            // auto new_motion = std::make_unique<MoveLine>(
            //     pose_flange, pose_goal * T_tool.Inverse(),
            //     v_limit, a_limit, j_limit, dt_, model.get());
            // Result rc = new_motion->Reset();
            if (rc != Result::NoError) {
                Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc),
                    "MoveL planning failed: result={}, tool='{}', velocity={} m/s, acceleration={} m/s^2, jerk={} m/s^3, dt={} s.",
                    static_cast<int>(rc), tool_name, v_limit, a_limit, j_limit, dt_);
                return rc;
            }

            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(new_motion);
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            log_ptr_->error("MoveL request rejected by the state machine: state={}, enabled={}, running={}, tool='{}'.",
                            GetStateString(), IsEnabled(), IsRunning(), tool_name);
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    Result Robot::MoveJ_IK(const Frame &pose_goal,
                           double v_limit, double a_limit, double j_limit) {
        auto command_lock = LockToolState();
        if (!model) {
            log_ptr_->error("MoveJ_IK requires an initialized robot model.");
            return Result::ResourceUnavailable;
        }

        const int n = getJointNum();
        JntArray q_current(static_cast<unsigned int>(n));
        for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);

        JntArray q_goal(static_cast<unsigned int>(n));
        Result rc = model->InverseKinematics(q_current, pose_goal, q_goal);
        if (rc != Result::NoError) {
            Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc), "MoveJ_IK inverse kinematics failed: result={}, joint_count={}, state={}.",
                            static_cast<int>(rc), n, GetStateString());
            return rc;
        }

        return MoveJ(q_goal, v_limit, a_limit, j_limit);
    }

    Result Robot::MoveL_FK(const JntArray &q_goal,
                           const std::string &tool_name,
                           double v_limit, double a_limit, double j_limit) {
        auto command_lock = LockToolState();
        if (!model) {
            log_ptr_->error("MoveL_FK requires an initialized robot model.");
            return Result::ResourceUnavailable;
        }

        Frame pose_goal;
        Result rc = model->ForwardKinematics(q_goal, pose_goal);
        if (rc != Result::NoError) return rc;

        return MoveL(pose_goal, tool_name, v_limit, a_limit, j_limit);
    }

    std::vector<std::string> Robot::GetToolFrameNames() const { return frames_.names(true); }
    std::vector<std::string> Robot::GetObjectFrameNames() const { return frames_.names(false); }
    bool Robot::HasToolFrame(const std::string& name) const { Frame frame; return frames_.get(name, true, frame) == Result::NoError; }
    bool Robot::HasObjectFrame(const std::string& name) const { Frame frame; return frames_.get(name, false, frame) == Result::NoError; }
    bool Robot::IsValidFrameName(const std::string& name) const { return FrameRegistry::validName(name); }
    Result Robot::ComputeFlangePose(const JntArray& joints, Frame& flange) const {
        if (!model) return Result::ResourceUnavailable;
        if (joints.rows() == 0 || joints.rows() != static_cast<unsigned int>(model->GetJointNum()))
            return Result::FkCalcFail;
        for (unsigned int i = 0; i < joints.rows(); ++i)
            if (!std::isfinite(joints(i))) return Result::FkCalcFail;
        Frame value;
        const auto result = model->ForwardKinematics(joints, value);
        if (result != Result::NoError) return result;
        if (!FrameRegistry::validFrame(value)) return Result::FkCalcFail;
        flange = value;
        return Result::NoError;
    }
    Result Robot::ReadFlangePose(Frame& flange) const {
        if (!hardware) return Result::ResourceUnavailable;
        return ComputeFlangePose(hardware->GetPosition(), flange);
    }
    Result Robot::GetTcpPose(Frame& tcp, const std::string& tool_name) const {
        const auto frames = frames_.snapshot();
        Frame flange;
        const auto result = ReadFlangePose(flange);
        if (result != Result::NoError) return result;
        return FrameRegistry::resolveTcp(*frames, flange, tcp, tool_name);
    }
    Result Robot::GetToolFrame(const std::string& name, Frame& frame) const { return frames_.get(name, true, frame); }
    Result Robot::GetObjectFrame(const std::string& name, Frame& frame) const { return frames_.get(name, false, frame); }
    Frame Robot::GetToolFrame(const std::string& name) const { Frame frame; frames_.get(name, true, frame); return frame; }
    Frame Robot::GetObjectFrame(const std::string& name) const { Frame frame; frames_.get(name, false, frame); return frame; }
    Result Robot::SetToolFrame(const std::string& name, const Frame& frame) { return ModifyFrames([&] { return frames_.set(name, frame, true); }); }
    Result Robot::SetToolFrame(const std::string& name, const Frame& frame, bool persist) { return ModifyFrames([&] { return frames_.set(name, frame, true, false, persist); }); }
    Result Robot::SetObjectFrame(const std::string& name, const Frame& frame) { return ModifyFrames([&] { return frames_.set(name, frame, false); }); }
    Result Robot::AddToolFrame(const std::string& name, const Frame& frame) { return ModifyFrames([&] { return frames_.set(name, frame, true, true); }); }
    Result Robot::AddObjectFrame(const std::string& name, const Frame& frame) { return ModifyFrames([&] { return frames_.set(name, frame, false, true); }); }
    Result Robot::RemoveToolFrame(const std::string& name) { return ModifyFrames([&] { return frames_.remove(name, true); }); }
    Result Robot::RemoveToolFrame(const std::string& name, bool persist) { return ModifyFrames([&] { return frames_.remove(name, true, persist); }); }
    Result Robot::RemoveObjectFrame(const std::string& name) { return ModifyFrames([&] { return frames_.remove(name, false); }); }
    Result Robot::SetActiveToolFrame(const std::string& name) { return ModifyFrames([&] { return frames_.activate(name, true); }); }
    Result Robot::SetActiveToolFrame(const std::string& name, bool persist) { return ModifyFrames([&] { return frames_.activate(name, true, persist); }); }
    Result Robot::SetActiveObjectFrame(const std::string& name) { return ModifyFrames([&] { return frames_.activate(name, false); }); }
    std::string Robot::GetActiveToolFrameName() const { return frames_.snapshot()->active_tool; }
    std::string Robot::GetActiveObjectFrameName() const { return frames_.snapshot()->active_object; }
    Result Robot::GetActiveToolFrame(Frame& frame) const {
        auto state = frames_.snapshot();
        auto found = state->tools.find(state->active_tool);
        if (found == state->tools.end()) return Result::ResourceUnavailable;
        frame = found->second;
        return Result::NoError;
    }
    Result Robot::GetActiveObjectFrame(Frame& frame) const {
        auto state = frames_.snapshot();
        auto found = state->objects.find(state->active_object);
        if (found == state->objects.end()) return Result::ResourceUnavailable;
        frame = found->second;
        return Result::NoError;
    }
    Frame Robot::GetActiveToolFrame() const { Frame frame; GetActiveToolFrame(frame); return frame; }
    Frame Robot::GetActiveObjectFrame() const { Frame frame; GetActiveObjectFrame(frame); return frame; }
    Result Robot::LoadFrames(const std::string& path) { return ModifyFrames([&] { return frames_.load(path); }); }
    Result Robot::SaveFrames(const std::string& path) const { return ModifyFrames([&] { return frames_.save(path); }); }

    std::unique_lock<std::recursive_mutex> Robot::LockToolState() const { return impl_->lock(); }
    bool Robot::CanChangeToolState() const {
        return impl_->is(sml::state<class IDLE>) || impl_->is(sml::state<class STOPPED>);
    }
    Result Robot::ModifyFrames(const std::function<Result()>& change) const {
        auto state_lock = LockToolState();
        if (!CanChangeToolState()) return Result::CallingConflictError;
        return change();
    }
    Result Robot::CaptureCalibrationPose(Frame& flange, std::vector<double>& joints, double& timestamp) const {
        // 采集耗时短，持命令门闩及 FSM → Robot 锁；状态检查和采集为同一操作。
        auto command_lock = LockToolState();
        auto state_lock = impl_->lockState();
        if (!CanChangeToolState()) return Result::CallingConflictError;
        std::lock_guard<std::mutex> lock(mtx_);
        if (!hardware || !model) return Result::CallingConflictError;
        const auto q = hardware->GetPosition();
        const auto velocity = hardware->GetVelocity();
        if (q.rows() == 0 || velocity.rows() != q.rows()) return Result::ResourceUnavailable;
        for (unsigned int i = 0; i < q.rows(); ++i) {
            if (!std::isfinite(q(i)) || !std::isfinite(velocity(i)) || std::abs(velocity(i)) > 0.001)
                return Result::CallingConflictError;
        }
        const auto result = model->ForwardKinematics(q, flange);
        if (result != Result::NoError || !FrameRegistry::validFrame(flange)) return Result::FkCalcFail;
        joints.resize(q.rows());
        for (unsigned int i = 0; i < q.rows(); ++i) joints[i] = q(i);
        timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        return Result::NoError;
    }

    // ============================================================================
    // MoveC 圆心+角度
    // ============================================================================

    Result Robot::MoveC(const Frame &pose_start, const Frame &center_frame,
                        double theta, double v_limit, double a_limit, double j_limit) {
        auto command_lock = LockToolState();
        data_ready_callback_ = [this, pose_start, center_frame, theta, v_limit, a_limit, j_limit]() -> Result {
            auto new_motion = std::make_unique<MoveCircleOffline>(
                pose_start, center_frame, theta, model.get(),
                v_limit, a_limit, j_limit, dt_);

            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
            new_motion->SetInitialJointPosition(q_current);

            Result rc = new_motion->Reset();
            if (rc != Result::NoError) {
                Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc),
                    "MoveC center-angle planning failed: result={}, angle={} rad, velocity={} m/s, acceleration={} m/s^2, jerk={} m/s^3, dt={} s.",
                    static_cast<int>(rc), theta, v_limit, a_limit, j_limit, dt_);
                return rc;
            }

            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(new_motion);
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;
        };

        if (!impl_->process_event(EventStartReq{})) {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            log_ptr_->error("MoveC center-angle request rejected: state={}, enabled={}, running={}.",
                            GetStateString(), IsEnabled(), IsRunning());
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveC 三点圆弧
    // ============================================================================

    Result Robot::MoveC(const Frame &pose_start, const Frame &pose_via,
                        const Frame &pose_goal,
                        double v_limit, double a_limit, double j_limit) {
        auto command_lock = LockToolState();
        data_ready_callback_ = [this, pose_start, pose_via, pose_goal, v_limit, a_limit, j_limit]() -> Result {
            auto new_motion = std::make_unique<MoveCircleOffline>(
                pose_start, pose_via, pose_goal, model.get(),
                v_limit, a_limit, j_limit, dt_);

            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
            new_motion->SetInitialJointPosition(q_current);

            Result rc = new_motion->Reset();
            if (rc != Result::NoError) {
                Logger::logCode(LogLevel::Error, "Robot", static_cast<int>(rc),
                    "MoveC three-point planning failed: result={}, velocity={} m/s, acceleration={} m/s^2, jerk={} m/s^3, dt={} s.",
                    static_cast<int>(rc), v_limit, a_limit, j_limit, dt_);
                return rc;
            }

            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(new_motion);
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;
        };

        if (!impl_->process_event(EventStartReq{})) {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            log_ptr_->error("MoveC three-point request rejected: state={}, enabled={}, running={}.",
                            GetStateString(), IsEnabled(), IsRunning());
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    Result Robot::PauseMotion() {
        //TODO: 直接判断是否支持暂停，不支持直接忽略请求
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!executor->CanPause()) {
                log_ptr_->error("Pause rejected: current motion does not support pausing; state={}, motion_ptr={}.",
                                GetStateString(), static_cast<const void*>(motion.get()));
                return Result::FunctionNotSupported;
            }
        }

        if (!impl_->process_event(EventPauseReq{})) {
            log_ptr_->error("Pause rejected in the current state: {}.", GetStateString());
            return Result::Fatal;
        }

        return Result::NoError;
    }

    Result Robot::StopMotion() {
        //TODO: 直接判断是否支持停止，不支持直接忽略，但应该都需要支持，所以后续要删掉
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!executor->CanStop()) {
                log_ptr_->error("Stop rejected: current motion does not support stopping; state={}, motion_ptr={}.",
                                GetStateString(), static_cast<const void*>(motion.get()));
                return Result::FunctionNotSupported;
            }
        }

        if (!impl_->process_event(EventStopReq{})) {
            log_ptr_->error("Stop rejected in the current state: {}.", GetStateString());
            return Result::Fatal;
        }
        return Result::NoError;
    }

    Result Robot::ResumeMotion() {
        //TODO: 直接判断是否支持恢复，不支持直接忽略请求
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!executor->CanResume()) {
                log_ptr_->error("Resume rejected: current motion does not support resuming; state={}, motion_ptr={}.",
                                GetStateString(), static_cast<const void*>(motion.get()));
                return Result::FunctionNotSupported;
            }
        }

        if (!impl_->process_event(EventResumeReq{})) {
            log_ptr_->error("Resume rejected in the current state: {}.", GetStateString());
            return Result::Fatal;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveJog：连续点动 + 活性保持
    //
    // 语义分派（与 motion_fsm_executor_design.md 场景 8/9/10/11 对齐）：
    //   1) 当前 motion 已经是活跃 MoveJog  → 视为“喂饭”，转发 FeedJog()
    //   2) 当前存在其他 motion 且正在跑    → 冲突拒绝
    //   3) 其它情况                        → 新建 MoveJog，装入 executor，触发 FSM
    // ============================================================================
    Result Robot::MoveJogging(const Twist &direction,
                              JogFrame frame,
                              double speed,
                              double timeout,
                              double dir_threshold) {
        auto command_lock = LockToolState();
        KDL::Rotation R_base_frame = KDL::Rotation::Identity();

        switch (frame) {
            case JogFrame::BASE:
                break;
            case JogFrame::FLANGE:
                R_base_frame = getFlange().M;
                break;
            case JogFrame::TOOL: {
                Frame T_flange_tool;
                const Result rc = GetActiveToolFrame(T_flange_tool);
                if (rc != Result::NoError) return rc;
                R_base_frame = (getFlange() * T_flange_tool).M;
                break;
            }
            case JogFrame::OBJECT: {
                Frame T_base_object;
                const Result rc = GetActiveObjectFrame(T_base_object);
                if (rc != Result::NoError) return rc;
                R_base_frame = T_base_object.M;
                break;
            }
        }

        Twist base_direction;
        base_direction.vel = R_base_frame * direction.vel;
        base_direction.rot = R_base_frame * direction.rot;
        return MoveJogging(JogVec{base_direction}, speed, timeout, dir_threshold);
    }

    Result Robot::MoveJogging(const JogVec &direction, double speed,
                              double timeout, double dir_threshold) {
        auto command_lock = LockToolState();
        // ── 分支 1：已有活跃 MoveJog → 旁路喂饭 ──
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (auto *jog = dynamic_cast<MoveJog *>(motion.get())) {
                const Result rc = jog->FeedJog(direction, speed);
                if (rc == Result::NoError) return rc;
            }
        }

        // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
        const bool running = IsRunning();
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (motion && running) return Result::ConflictTaskRunning;
        }

        // ── 分支 3：全新启动 ──
        data_ready_callback_ = [this, direction, speed, timeout,
                    dir_threshold]() -> Result {
                    auto new_jog = std::make_unique<MoveJog>(
                        dt_, timeout, model.get(), dir_threshold);

                    const int n = getJointNum();
                    JntArray q_current(static_cast<unsigned int>(n));
                    for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
                    new_jog->setInitialPosition(q_current);
                    Result rc = new_jog->Reset();
                    if (rc != Result::NoError) return rc;

                    rc = new_jog->FeedJog(direction, speed);
                    if (rc != Result::NoError) return rc;

                    {
                        std::lock_guard<std::mutex> lock(mtx_);
                        motion = std::move(new_jog);
                        executor->SwitchMotion(motion.get());
                    }
                    return Result::NoError;

                    log_ptr_->error("executor is nullptr");
                    return Result::Fatal;
                };

        if (!impl_->process_event(EventStartReq{})) {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveNullJogging — 零空间点动（伪逆投影法）
    // ============================================================================

    Result Robot::MoveNullJogging(const JntArray &intent_direction, double speed,
                                  double timeout, double dir_threshold) {
        auto command_lock = LockToolState();
        // ── 分支 1：已有活跃 MoveNullJog → 旁路喂饭 ──
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (auto *jog = dynamic_cast<MoveNullJog *>(motion.get())) {
                const Result rc = jog->FeedNullJog(intent_direction, speed);
                if (rc == Result::NoError) return rc;
            }
        }

        // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
        const bool running = IsRunning();
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (motion && running) return Result::ConflictTaskRunning;
        }

        // ── 分支 3：全新启动 ──
        data_ready_callback_ = [this, intent_direction, speed, timeout, dir_threshold]() -> Result {
            auto new_jog = std::make_unique<MoveNullJog>(
                dt_, timeout, model.get(), dir_threshold);

            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
            new_jog->setInitialPosition(q_current);

            Result rc = new_jog->Reset();
            if (rc != Result::NoError) return rc;

            rc = new_jog->FeedNullJog(intent_direction, speed);
            if (rc != Result::NoError) return rc;

            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(new_jog);
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

    // ============================================================================
    // MoveSvdJogging — 零空间点动（SVD 基向量法）
    // ============================================================================
    // ============================================================================

    Result Robot::MoveSvdJogging(const std::vector<double> &dim_speeds,
                                 double timeout, double dir_threshold) {
        auto command_lock = LockToolState();
        // ── 分支 1：已有活跃 MoveSvdJog → 旁路喂饭 ──
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (auto *jog = dynamic_cast<MoveSvdJog *>(motion.get())) {
                const Result rc = jog->FeedSvdJog(dim_speeds);
                if (rc == Result::NoError) return rc;
            }
        }

        // ── 分支 2：其他 motion 正在运行 → 拒绝 ──
        const bool running = IsRunning();
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (motion && running) return Result::ConflictTaskRunning;
        }

        // ── 分支 3：全新启动 ──
        data_ready_callback_ = [this, dim_speeds, timeout, dir_threshold]() -> Result {
            auto new_jog = std::make_unique<MoveSvdJog>(
                dt_, timeout, model.get(), dir_threshold);

            const int n = getJointNum();
            JntArray q_current(static_cast<unsigned int>(n));
            for (int i = 0; i < n; ++i) q_current(i) = getJointPosition(i);
            new_jog->setInitialPosition(q_current);

            Result rc = new_jog->Reset();
            if (rc != Result::NoError) return rc;

            rc = new_jog->FeedSvdJog(dim_speeds);
            if (rc != Result::NoError) return rc;

            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(new_jog);
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;

            log_ptr_->error("executor is nullptr");
            return Result::Fatal;
        };

        if (!impl_->process_event(EventStartReq{})) {
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }
    // ============================================================================
    // SetServoMode —— 设置伺服运动模式
    // ============================================================================

    Result Robot::SetServoMode(const std::string& mode_str) {
        auto command_lock = LockToolState();
        MotionMode new_mode;
        if (mode_str == "joint") {
            new_mode = MotionMode::kJointPosition;
        } else if (mode_str == "cartesian") {
            new_mode = MotionMode::kCartesianPosition;
        } else {
            log_ptr_->error("SetServoMode received unknown mode='{}'; supported modes are joint and cartesian.", mode_str);
            return Result::IllegalParameter;
        }

        servo_mode_ = new_mode;

        // 如果当前正在伺服模式，动态更新运行中的 MoveServo
        if (impl_->is(sml::state<class SERVOING>)) {
            std::lock_guard<std::mutex> lock(mtx_);
            auto* servo = dynamic_cast<MoveServo*>(motion.get());
            if (servo) {
                servo->SetMode(new_mode);
            } else {
                log_ptr_->warn("SetServoMode cannot update the active motion: it is not a MoveServo instance.");
            }
        }

        log_ptr_->info("Servo mode set to {}.", mode_str);
        return Result::NoError;
    }

    // ============================================================================
    // MoveServoing —— UDP 伺服模式
    // ============================================================================

    Result Robot::MoveServoing(uint16_t port) {
        auto command_lock = LockToolState();
        data_ready_callback_ = [this, port]() -> Result {
            auto servo = std::make_unique<MoveServo>(
                hardware.get(), model.get(), port);
            servo->SetMode(servo_mode_);
            Result rc = servo->Reset();
            if (rc != Result::NoError) {
                return rc;
            }
            {
                std::lock_guard<std::mutex> lock(mtx_);
                motion = std::move(servo);
                executor->SwitchMotion(motion.get());
            }
            return Result::NoError;
        };

        if (!impl_->process_event(EventServoReq{})) {
            log_ptr_->error("Servo receiver start failed: port={}.", port);
            data_ready_callback_ = nullptr;  // 切换失败，清除无效回调
            if (impl_->is(sml::state<class IDLE>)) return Result::NotEnabled;
            if (impl_->is(sml::state<class ERROR_STATE>)) return Result::Fatal;
            return Result::ConflictTaskRunning;
        }
        return Result::NoError;
    }

} // namespace rocos
