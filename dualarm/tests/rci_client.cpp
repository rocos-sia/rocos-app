#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "librocos/robot.h"

using namespace std::chrono_literals;
namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

struct MotionFeedback {
  std::atomic<double> displacement{0};
  std::atomic<uint64_t> applied{0};
};

void Stream(librocos::Robot& arm, double offset, MotionFeedback& feedback,
            const std::atomic<bool>& interrupt) {
  librocos::JointArray initial{};
  double elapsed = 0;
  arm.control(
      [&](const librocos::RobotState& state, librocos::Duration period) {
        if (period.toNSec() == 0) initial = state.q_d;
        elapsed += period.toSec();
        feedback.displacement = state.q[0] - initial[0];
        feedback.applied = state.applied_message_id;
        if (interrupt.load()) std::this_thread::sleep_for(150ms);
        const double phase = std::min(elapsed / 2.5, 1.0);
        const double blend =
            phase * phase * phase * (10 + phase * (-15 + 6 * phase));
        librocos::JointPositions target(initial, state.dof);
        target.q[0] += offset * blend;
        return elapsed < 6 ? target : librocos::MotionFinished(target);
      });
}

template <typename Condition>
void WaitForMotion(Condition ready, std::future<void>& active) {
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (!ready()) {
    if (active.wait_for(5ms) == std::future_status::ready) {
      active.get();
      throw std::runtime_error(
          "SDK control ended before reaching the expected position");
    }
    Require(std::chrono::steady_clock::now() < deadline,
            "Timed out waiting for arm motion");
  }
}

void CheckRci(uint16_t left_port, uint16_t right_port) {
  librocos::Options left_options;
  left_options.arm = "left";
  left_options.udp_port = left_port;
  auto right_options = left_options;
  right_options.arm = "right";
  right_options.udp_port = right_port;
  librocos::Robot left("127.0.0.1", left_options);
  librocos::Robot right("127.0.0.1", right_options);
  const auto left_initial = left.readOnce();
  Require(left_initial.dof == 7 && right.readOnce().dof == 7,
          "Expected two seven-axis arms");

  MotionFeedback left_feedback, right_feedback;
  std::atomic<bool> no_interrupt{false}, interrupt_right{false};
  right.enableRci();
  auto right_worker = std::async(std::launch::async, [&] {
    Stream(right, 0.07, right_feedback, interrupt_right);
  });
  WaitForMotion([&] { return right_feedback.displacement > 0.006; },
                right_worker);
  const auto left_hold = left.readOnce();
  Require(left_hold.robot_state == "STOPPED",
          "Right RCI affected left arm state");
  for (size_t joint = 0; joint < left_hold.dof; ++joint)
    Require(std::abs(left_hold.q[joint] - left_initial.q[joint]) < 0.003,
            "Right RCI moved a left arm joint");

  left.enableRci();
  const double right_before_overlap = right_feedback.displacement;
  auto left_worker = std::async(std::launch::async, [&] {
    Stream(left, -0.07, left_feedback, no_interrupt);
  });
  WaitForMotion([&] { return left_feedback.displacement < -0.01; },
                left_worker);
  const double right_during_overlap = right_feedback.displacement;
  const double left_during_overlap = left_feedback.displacement;
  const auto right_applied = right_feedback.applied.load();
  const auto left_applied = left_feedback.applied.load();
  Require(right.rciStatus().robot_state == "SERVOING" &&
              left.rciStatus().robot_state == "SERVOING" &&
              right_during_overlap - right_before_overlap > 0.01 &&
              right_applied > 0 && left_applied > 0,
          "Two distinct SDK sessions did not move both arms concurrently");

  interrupt_right = true;
  try {
    right_worker.get();
    throw std::runtime_error("Right arm accepted a stalled UDP stream");
  } catch (const librocos::ControlException& error) {
    if (error.code() != -1008) throw;
  }
  const auto right_fault = right.rciStatus();
  Require(right_fault.robot_state == "ERROR_STATE" && !right_fault.enabled &&
              !right_fault.control_active && right_fault.last_control &&
              right_fault.last_control->fault_code == -1008,
          "Right arm did not record the communication fault and close RCI");
  WaitForMotion(
      [&] { return left_feedback.displacement < left_during_overlap - 0.01; },
      left_worker);
  Require(left.rciStatus().robot_state == "SERVOING" &&
              left_feedback.applied > left_applied,
          "Left arm stopped applying commands after right UDP loss");
  left.stop();
  left_worker.get();
  const auto left_stopped = left.rciStatus();
  Require(left_stopped.robot_state == "RCI" && !left_stopped.control_active,
          "Explicit stop did not return left arm to RCI standby");
  left.disableRci();
  Require(left.rciStatus().robot_state == "STOPPED",
          "RCI disable did not restore STOPPED");

  std::cout << "{\"left\":{\"port\":" << left_port
            << ",\"joint1_delta_rad\":" << left_during_overlap
            << ",\"applied_message_id\":" << left_applied
            << "},\"right\":{\"port\":" << right_port
            << ",\"joint1_delta_rad\":" << right_during_overlap
            << ",\"applied_message_id\":" << right_applied
            << "},\"right_fault_code\":" << right_fault.last_control->fault_code
            << "}\n";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    Require(argc == 3, "Expected left and right UDP ports");
    CheckRci(static_cast<uint16_t>(std::stoi(argv[1])),
             static_cast<uint16_t>(std::stoi(argv[2])));
  } catch (const std::exception& error) {
    std::cerr << "Dual-arm librocos RCI: " << error.what() << '\n';
    return 1;
  }
}
