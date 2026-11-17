#include "dualarm_controller.hpp"

#include <future>
#include <stdexcept>
#include <thread>

namespace rocos::dualarm {
DualArmController::DualArmController(const DualArmConfig& config)
    : config_(config) {
  // Check both buses before constructing either Robot; this also catches cycle
  // mismatches.
  for (const auto& arm : config_.arms) {
    EcatConfig probe(arm.robot.ecat_id);
    if (!probe.connect(true))
      throw std::runtime_error("Cannot connect master " +
                               std::to_string(arm.robot.ecat_id));
    if (probe.getDt() != arm.robot.master_cycle_us)
      throw std::runtime_error(
          "Master cycle does not match robot configuration");
  }
  for (size_t i = 0; i < robots_.size(); ++i) {
    ScopedLogContext log_scope({i == 0 ? LogScope::Left : LogScope::Right,
                                config_.arms[i].robot.ecat_id});
    contexts_[i] = std::make_unique<ArmApiContext>(config_.control_ttl_seconds);
    robots_[i] = std::make_unique<Robot>(config_.arms[i].robot);
    if (!robots_[i]->IsDisabled() || robots_[i]->GetStateString() != "IDLE")
      throw std::runtime_error(
          "Robot failed to initialize in disabled IDLE state");
  }
}
DualArmController::~DualArmController() { Shutdown(); }
bool DualArmController::Shutdown() {
  if (stopped_) return true;
  stopped_ = true;
  std::array<std::future<bool>, 2> workers;
  for (size_t i = 0; i < robots_.size(); ++i) {
    workers[i] = std::async(std::launch::async, [&, i] {
      ScopedLogContext log_scope({i == 0 ? LogScope::Left : LogScope::Right,
                                  config_.arms[i].robot.ecat_id});
      auto& robot = *robots_[i];
      try {
        if (robot.GetStateString() == "IDLE" && robot.IsDisabled()) return true;
        if (robot.IsMotionBusy()) {
          robot.StopMotion();
          const auto deadline =
              std::chrono::steady_clock::now() + std::chrono::seconds(3);
          while (robot.IsMotionBusy() &&
                 std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        robot.SetDisabled();
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < deadline) {
          if (robot.IsDisabled() && robot.GetStateString() == "IDLE")
            return true;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        robot.hardware->RequestSafetyStop();
      } catch (...) {
        robot.hardware->RequestSafetyStop();
      }
      return false;
    });
  }
  bool ok = true;
  for (auto& worker : workers) ok = worker.get() && ok;
  return ok;
}
}  // namespace rocos::dualarm
