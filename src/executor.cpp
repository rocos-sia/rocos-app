#include "executor.hpp"
#include "move_servo.hpp"

namespace rocos {

Executor::Executor(MotionInterface* motion, ControllerInterface* controller,
                   HardwareInterface* hardware) {

  log_ptr_ = Logger::getInstance("Executor");

  SwitchMotion(motion);
  SwitchController(controller);
  SwitchHardware(hardware);

  profiler_ = std::make_unique<PerformanceProfiler>(
    std::vector<PerformanceMeasureInfo>{
        {kMotionMeasurement, "Motion"},
        {kControllerMeasurement, "Controller"},
        {kExecutorMeasurement, "Executor Update"},
        {kExecutorMeasurement, "Cycle Time"}});

}

Executor::~Executor() {
  if (motion_)
    motion_->Stop();

  if (controller_)
    controller_->Reset();

  if (hardware_)
    hardware_->Reset();

  motion_ = nullptr;
  controller_ = nullptr;




}

Result Executor::Update() {

  Result res = Result::NoError;
  int sum = 0; //TODO:

  if (motion_ == nullptr || controller_ == nullptr || hardware_ == nullptr) {
    emitLogEvent(LogEventType::Result, LogModule::Executor,
                 static_cast<int>(Result::ParameterPointerEqualsNullptr));
    return Result::ParameterPointerEqualsNullptr;
  }

  auto* servo=dynamic_cast<MoveServo*>(motion_);
  auto complete=[&](Result result) {
    if (servo) servo->CompleteCycle(result);
    if (static_cast<int>(result)<0 && servo) hardware_->RequestSafetyStop();
    return result;
  };
  if (hardware_->SafetyStopRequested()) return complete(Result::LostHeartbeat);
  profiler_->MeasureEnd(kCycleMeasurement);
  profiler_->MeasureStart(kExecutorMeasurement);

  // ① Motion → 当前参考位姿
  profiler_->MeasureStart(kMotionMeasurement);
  Reference ref;
  res = motion_->GenerateRef(ref);

  profiler_->MeasureEnd(kMotionMeasurement);

  if (static_cast<int>(res) < 0) {
    emitLogEvent(LogEventType::MotionFailed, LogModule::Executor, static_cast<int>(res));
    return complete(res);
  }

  sum += static_cast<int>(res);

  // ② Controller → 参考转关节指令
  profiler_->MeasureStart(kControllerMeasurement);
  JntArray q_cmd;
  res = controller_->GenerateCmd(ref, q_cmd);
  profiler_->MeasureEnd(kControllerMeasurement);

  if (static_cast<int>(res) < 0) {
    emitLogEvent(LogEventType::ControllerFailed, LogModule::Executor, static_cast<int>(res));
    return complete(res);
  }

  sum += static_cast<int>(res);


  // ③ 下发硬件
  res = controller_->UpdateCmd(q_cmd);

  if (static_cast<int>(res) < 0) {
    emitLogEvent(LogEventType::CommandFailed, LogModule::Executor, static_cast<int>(res));
    return complete(res);
  }

  sum += static_cast<int>(res);


  profiler_->MeasureEnd(kExecutorMeasurement);
  profiler_->MeasureStart(kCycleMeasurement);

  if (sum > 0) {
    return complete(Result::PlanFinished);
  }

  return complete(Result::NoError);
}

bool Executor::SwitchController(ControllerInterface* new_controller) {

  controller_ = new_controller;
  return true;

}

bool Executor::SwitchHardware(HardwareInterface* new_hardware) {
  hardware_ = new_hardware;
  return true;
}

bool Executor::SwitchMotion(MotionInterface* new_motion) {
  motion_ = new_motion; // 资源释放只需在子类析构处理，这里只需要直接替换
  return true;
}

}  // namespace rocos
