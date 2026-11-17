#include "log_journal.hpp"

#include <unistd.h>

#include <charconv>
namespace rocos {
namespace {
LogJson gapJson(const char* reason, const std::string& earliest,
                uint64_t count = 0) {
  LogJson gap{{"reason", reason}, {"earliest_id", earliest}};
  if (count) gap["count"] = count;
  return gap;
}
std::string gapSse(const LogJson& gap) {
  return "event: gap\ndata: " + gap.dump() + "\n\n";
}
LogJson frameJson(const LogFrame& frame) {
  const auto start = frame.sse.find("data: ") + 6;
  return LogJson::parse(frame.sse.data() + start,
                        frame.sse.data() + frame.sse.size() - 2);
}
}  // namespace
LogJournal::LogJournal() {
  session_ =
      "dual-" + std::to_string(logNowMicros()) + "-" + std::to_string(getpid());
}
void LogJournal::publish(LogJson event) {
  std::lock_guard<std::mutex> lock(history_mutex_);
  if (!running_) return;
  const auto id = session_ + ":" + std::to_string(++sequence_);
  event["id"] = id;
  auto serialized =
      event.dump(-1, ' ', false, LogJson::error_handler_t::replace);
  while (serialized.size() > 12288) {
    auto& message = event["message"].get_ref<std::string&>();
    if (message.size() < 64) return;
    message.resize(message.size() / 2);
    message += " [truncated]";
    serialized = event.dump(-1, ' ', false, LogJson::error_handler_t::replace);
  }
  retain({sequence_, true, id,
          "id: " + id + "\nevent: log\ndata: " + serialized + "\n\n", 0});
}
void LogJournal::retain(LogFrame frame) {
  // 计入实际字符串容量、帧结构和保守的容器/共享句柄额度，不保留第二份 JSON 树。
  if (frame.id.empty())
    frame.id = session_ + ":" + std::to_string(frame.sequence);
  frame.bytes =
      frame.sse.capacity() + frame.id.capacity() + sizeof(LogFrame) + 128;
  auto shared = std::make_shared<const LogFrame>(std::move(frame));
  history_bytes_ += shared->bytes;
  history_.push_back(std::move(shared));
  while (history_.size() > 4096 || history_bytes_ > 16 * 1024 * 1024) {
    history_bytes_ -= history_.front()->bytes;
    history_.pop_front();
    ++evicted_;
  }
  for (auto it = subscribers_.begin(); it != subscribers_.end();) {
    if (auto subscriber = it->lock()) {
      subscriber->push(history_.back());
      ++it;
    } else
      it = subscribers_.erase(it);
  }
}
void LogJournal::publishGap(uint64_t count) {
  std::lock_guard<std::mutex> lock(history_mutex_);
  auto gap =
      gapJson("producer_overflow",
              history_.empty()
                  ? session_ + ":0"
                  : session_ + ":" + std::to_string(history_.front()->sequence),
              count);
  auto text = gapSse(gap);
  retain({++sequence_, false, "", text, 0});
}
uint64_t LogJournal::cursor(const std::string& after, bool& restarted) const {
  const auto colon = after.find(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 == after.size())
    throw std::invalid_argument("Invalid log cursor");
  for (size_t i = 0; i < colon; ++i) {
    char c = after[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-'))
      throw std::invalid_argument("Invalid log session");
  }
  uint64_t value = 0;
  auto parsed = std::from_chars(after.data() + colon + 1,
                                after.data() + after.size(), value);
  if (parsed.ec != std::errc{} || parsed.ptr != after.data() + after.size())
    throw std::invalid_argument("Invalid log cursor sequence");
  restarted = after.substr(0, colon) != session_;
  if (!restarted && value > sequence_)
    throw std::invalid_argument("Log cursor is ahead of the latest record");
  return restarted ? 0 : value;
}
LogJson LogJournal::queryLocked(const std::string& after, size_t limit) const {
  if (limit < 1 || limit > 500)
    throw std::invalid_argument("limit must be in [1, 500]");
  if (after.size() > 128) throw std::invalid_argument("Log cursor is too long");
  bool restarted = false;
  uint64_t start = after.empty() ? 0 : cursor(after, restarted);
  auto earliest =
      session_ + ":" +
      std::to_string(history_.empty() ? sequence_ : history_.front()->sequence);
  LogJson gap = nullptr;
  if (restarted)
    gap = gapJson("server_restarted", earliest);
  else if (!after.empty() && !history_.empty() &&
           start < history_.front()->sequence - 1)
    gap = gapJson("buffer_overrun", earliest);
  size_t skip = 0;
  if (after.empty()) {
    size_t count = 0;
    for (const auto& frame : history_)
      if (frame->is_log) ++count;
    skip = count > limit ? count - limit : 0;
  }
  LogJson events = LogJson::array();
  uint64_t next = after.empty() || restarted ? 0 : start;
  bool more = false;
  for (const auto& frame : history_) {
    if (frame->sequence <= start) continue;
    if (frame->is_log && skip) {
      --skip;
      continue;
    }
    if (events.size() >= limit) {
      more = true;
      break;
    }
    next = frame->sequence;
    if (!frame->is_log) {
      if (gap.is_null()) gap = frameJson(*frame);
    } else
      events.push_back(frameJson(*frame));
  }
  return {{"events", events},
          {"next_id", session_ + ":" + std::to_string(next)},
          {"has_more", more},
          {"gap", gap}};
}
LogJson LogJournal::query(const std::string& after, size_t limit) const {
  std::lock_guard<std::mutex> lock(history_mutex_);
  return queryLocked(after, limit);
}
std::shared_ptr<LogSubscription> LogJournal::subscribe(
    const std::string& after) {
  std::lock_guard<std::mutex> lock(history_mutex_);
  auto subscriber = std::make_shared<LogSubscription>();
  subscriber->gaps_ = &client_gaps_;
  if (!after.empty()) {
    bool restarted = false;
    auto start = cursor(after, restarted);
    if (restarted ||
        (!history_.empty() && start < history_.front()->sequence - 1)) {
      auto gap = gapJson(
          restarted ? "server_restarted" : "buffer_overrun",
          session_ + ":" +
              std::to_string(history_.empty() ? sequence_
                                              : history_.front()->sequence));
      subscriber->push(std::make_shared<const LogFrame>(
          LogFrame{0, false, "", gapSse(gap), 0}));
    }
    for (const auto& frame : history_)
      if (frame->sequence > start) subscriber->push(frame);
  }
  subscribers_.erase(
      std::remove_if(subscribers_.begin(), subscribers_.end(),
                     [](const auto& weak) { return weak.expired(); }),
      subscribers_.end());
  if (!running_)
    subscriber->close();
  else
    subscribers_.push_back(subscriber);
  return subscriber;
}

void LogJournal::stop() {
  std::lock_guard<std::mutex> lock(history_mutex_);
  running_ = false;
  for (auto& weak : subscribers_)
    if (auto sub = weak.lock()) {
      sub->close();
      sub->gaps_ = nullptr;
    }
  subscribers_.clear();
}
}  // namespace rocos
