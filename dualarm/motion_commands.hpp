#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "api_values.hpp"

namespace rocos::dualarm {
// Protected by ArmApiContext::status_mutex. Dispatch additionally holds the
// arm command mutex, so validating a fence and accepting a command are atomic
// with respect to stop. Old epochs remain invalid after history is discarded.
class MotionCommands {
 public:
  bool Validate(const Json& body, const std::string& instance, bool required,
                bool precise) const;
  bool Begin(const Json& body, const std::string& operation,
             const std::string& owner);
  void Finish(const std::string& id, bool success, int code,
              const std::string& message);
  void Stop();
  void Observe(const Json& state);
  Json Metadata(const std::string& instance) const;
  Json Read(const std::string& id, const std::string& owner) const;

 private:
  struct Record {
    Json body, data;
    std::string owner;
    uint64_t after_sequence = 0;
    unsigned quiet = 0;
    bool stopping = false;
  };
  static bool Terminal(const std::string& status);
  uint64_t epoch_ = 1;
  uint64_t sequence_ = 0;
  std::map<std::string, Record> records_;
  std::string latest_;
};
}  // namespace rocos::dualarm
