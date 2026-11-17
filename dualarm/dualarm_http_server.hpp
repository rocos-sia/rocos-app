#pragma once
#include <atomic>
#include <thread>

#include "dualarm_controller.hpp"
#include "httplib.h"
#include "model_bundle.hpp"

namespace rocos::dualarm {
class DualArmHttpServer {
 public:
  explicit DualArmHttpServer(DualArmController& controller);
  ~DualArmHttpServer();
  void Start();
  void Stop();
  bool Running() const { return running_.load(); }

 private:
  void Handle(const httplib::Request& req, httplib::Response& res);
  void HandleModel(const httplib::Request& req, httplib::Response& res,
                   const std::string& action);
  DualArmController& controller_;
  std::unique_ptr<ModelBundle> model_;
  httplib::Server server_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<uint64_t> request_sequence_{0};
};
}  // namespace rocos::dualarm
