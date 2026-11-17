#pragma once
#include "log_manager.hpp"
namespace rocos {
// Optional INFO+ stream. Its cursor and memory budget are independent of legacy
// alerts.
class LogJournal {
 public:
  LogJournal();
  ~LogJournal() { stop(); }
  void publish(LogJson event);
  void publishGap(uint64_t count);
  LogJson query(const std::string& after, size_t limit) const;
  std::shared_ptr<LogSubscription> subscribe(const std::string& after);
  void stop();

 private:
  uint64_t cursor(const std::string& after, bool& restarted) const;
  LogJson queryLocked(const std::string& after, size_t limit) const;
  void retain(LogFrame frame);
  bool running_{true};
  mutable std::mutex history_mutex_;
  std::deque<std::shared_ptr<const LogFrame>> history_;
  size_t history_bytes_{0};
  uint64_t sequence_{0}, evicted_{0};
  std::string session_;
  std::vector<std::weak_ptr<LogSubscription>> subscribers_;
  std::atomic<uint64_t> client_gaps_{0};
};
}  // namespace rocos
