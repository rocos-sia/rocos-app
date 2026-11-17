#pragma once
#include <array>
#include <memory>

#include "arm_api_context.hpp"
#include "dualarm_config.hpp"

namespace rocos::dualarm {
class DualArmController {
 public:
  explicit DualArmController(const DualArmConfig& config);
  ~DualArmController();
  Robot& Arm(size_t index) { return *robots_.at(index); }
  ArmApiContext& Context(size_t index) { return *contexts_.at(index); }
  const DualArmConfig& Config() const { return config_; }
  bool Shutdown();

 private:
  DualArmConfig config_;
  std::array<std::unique_ptr<Robot>, 2> robots_;
  std::array<std::unique_ptr<ArmApiContext>, 2> contexts_;
  bool stopped_ = false;
};
}  // namespace rocos::dualarm
