#include "arm_api_context.hpp"

#include <openssl/rand.h>

#include <algorithm>
#include <array>
#include <stdexcept>

namespace rocos::dualarm {
ControlLease::ControlLease(int ttl_seconds) : ttl_(ttl_seconds) {
  if (ttl_ < 1) throw std::invalid_argument("Control TTL must be positive");
}
std::string ControlLease::NewToken() {
  std::array<unsigned char, 32> bytes{};
  if (RAND_bytes(bytes.data(), bytes.size()) != 1)
    throw std::runtime_error("Cannot generate control credentials");
  const char* hex = "0123456789abcdef";
  std::string out;
  for (auto b : bytes) {
    out += hex[b >> 4];
    out += hex[b & 15];
  }
  return out;
}
void ControlLease::Expire(Clock::time_point now) {
  if (!token_.empty() && now - last_seen_ >= std::chrono::seconds(ttl_))
    Release();
}
void ControlLease::Release() {
  token_.clear();
  client_.clear();
  name_.clear();
  ip_.clear();
  agent_.clear();
  port_ = 0;
}
nlohmann::json ControlLease::Status(Clock::time_point now) {
  Expire(now);
  const bool held = !token_.empty();
  const auto age =
      held ? std::chrono::duration_cast<std::chrono::seconds>(now - acquired_)
                 .count()
           : 0;
  const auto idle =
      held ? std::chrono::duration_cast<std::chrono::seconds>(now - last_seen_)
                 .count()
           : 0;
  return {{"has_owner", held},
          {"owner_ip", ip_},
          {"owner_port", port_},
          {"owner_connection", held ? ip_ + ":" + std::to_string(port_) : ""},
          {"owner_client_id", client_},
          {"owner_name", name_},
          {"owner_agent", agent_},
          {"held_for_seconds", age},
          {"idle_seconds", idle},
          {"expires_in_seconds", held ? std::max<int64_t>(0, ttl_ - idle) : 0},
          {"ttl_seconds", ttl_},
          {"heartbeat_interval_seconds", std::min(5.0, ttl_ / 3.0)}};
}
bool ControlLease::Acquire(std::string client, std::string name, std::string ip,
                           int port, std::string agent, nlohmann::json& data,
                           Clock::time_point now) {
  Expire(now);
  if (!token_.empty()) {
    data = Status(now);
    return false;
  }
  const bool automatic = client.empty();
  auto new_token = NewToken();
  if (automatic) client = NewToken();
  token_ = std::move(new_token);
  client_ = std::move(client);
  name_ = std::move(name);
  ip_ = std::move(ip);
  port_ = port;
  agent_ = std::move(agent);
  acquired_ = last_seen_ = now;
  data = Status(now);
  data["token"] = token_;
  data["client_id"] = client_;
  data["client_id_auto"] = automatic;
  return true;
}
int ControlLease::Authorize(const std::string& token, const std::string& client,
                            Clock::time_point now) {
  Expire(now);
  if (token.empty()) return 3006;
  if (token_ != token || client.empty() || client_ != client) return 3007;
  last_seen_ = now;
  return 0;
}
}  // namespace rocos::dualarm
