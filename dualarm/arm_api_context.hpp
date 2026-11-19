#pragma once
#include <chrono>
#include <mutex>
#include <string>

#include "calibration_sessions.hpp"
#include "json.hpp"
#include "motion_commands.hpp"

namespace rocos::dualarm {
// The caller holds the arm command mutex through authentication AND dispatch.
// Clock injection allows expiry tests without wall-clock sleeps.
class ControlLease {
 public:
  using Clock = std::chrono::steady_clock;
  explicit ControlLease(int ttl_seconds);
  bool Acquire(std::string client, std::string name, std::string ip, int port,
               std::string agent, nlohmann::json& data,
               Clock::time_point now = Clock::now(), int command_protocol = 0);
  int Authorize(const std::string& token, const std::string& client,
                Clock::time_point now = Clock::now());
  nlohmann::json Status(Clock::time_point now = Clock::now());
  void Release();
  int CommandProtocol() const { return command_protocol_; }
  void EnableCommandProtocol() { command_protocol_ = 1; }

 private:
  static std::string NewToken();
  void Expire(Clock::time_point now);
  int ttl_;
  std::string token_, client_, name_, ip_, agent_;
  int port_ = 0;
  int command_protocol_ = 0;
  Clock::time_point acquired_{}, last_seen_{};
};
struct ArmApiContext {
  explicit ArmApiContext(int ttl) : lease(ttl) {}
  std::mutex command_mutex;
  std::mutex status_mutex;
  ControlLease lease;
  CalibrationSessions calibration;
  MotionCommands commands;
};
}  // namespace rocos::dualarm
