#pragma once

#include "log_storage.hpp"
#include "json.hpp"
#include <condition_variable>
#include <vector>

namespace rocos {
using LogJson = nlohmann::json;
struct LogFrame {
    uint64_t sequence{0};
    bool is_log{false};
    std::string id;
    std::string sse; // 缓存和各会话共享同一不可变序列化帧。
    size_t bytes{0};
};
class LogSubscription {
public:
    // 只在会话线程消费；公共锁释放后才调用网络 write。
    bool wait(std::string& frame, std::chrono::milliseconds timeout);
    void close();
private:
    friend class LogManager;
    friend class LogJournal;
    void push(const std::shared_ptr<const LogFrame>& frame);
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::shared_ptr<const LogFrame>> frames_;
    size_t bytes_{0};
    bool closed_{false};
    std::atomic<uint64_t>* gaps_{nullptr};
};
class LogJournal;
struct LogOperation {
    std::string name, request_id, outcome;
    int http_status{0};
    double duration_ms{0};
};
class LogManager {
public:
    explicit LogManager(LoggingConfig config = {}, std::function<int64_t()> storage_clock = logNowMicros);
    ~LogManager();
    static LogManager& instance();
    // 仅在启动期且未建立模块实例前配置全局实例。
    static void configure(const LoggingConfig& config);
    static void shutdown();
    RtLogProducer* createRtProducer();
    void releaseRtProducer(RtLogProducer* producer);
    void submit(LogLevel level, std::string module, std::string message,
                int code = 0, int64_t epoch_us = 0, LogOperation operation = {});
    void enableDetailedEvents(const std::array<std::string, 2>& device_ids);
    std::shared_ptr<LogJournal> detailedEvents() const;
    void submitEvent(const RtLogEvent& event); // 仅非实时后备路径。
    LogJson query(const std::string& after, size_t limit) const;
    std::shared_ptr<LogSubscription> subscribe(const std::string& after);
    SnapshotResult snapshot(std::chrono::milliseconds timeout = std::chrono::milliseconds(250));
    LogJson metrics() const;
    void stop(); // 必须在实时生产者停止后调用。
private:
    struct Pending {
        LogLevel level;
        std::string module, message;
        int code;
        int64_t epoch_us;
        LogContext context{};
        LogOperation operation{};
    };
    uint64_t cursor(const std::string& after, bool& restarted) const;
    LogJson queryLocked(const std::string& after, size_t limit) const;
    void publish(Pending record);
    void publishGap(uint64_t count);
    void retain(LogFrame frame);
    void run();
    std::array<std::string, 2> device_ids_{};
    std::shared_ptr<LogJournal> detailed_;
    LogStorage storage_;
    std::atomic<bool> running_{true};
    std::thread thread_;
    std::mutex pending_mutex_;
    std::deque<Pending> pending_;
    std::atomic<uint64_t> nonrt_dropped_{0};
    mutable std::mutex history_mutex_;
    std::deque<std::shared_ptr<const LogFrame>> history_;
    size_t history_bytes_{0};
    uint64_t sequence_{0}, evicted_{0};
    std::string session_;
    std::vector<std::weak_ptr<LogSubscription>> subscribers_;
    std::atomic<uint64_t> client_gaps_{0};
    std::mutex producers_mutex_;
    std::array<RtLogProducer, 8> producers_{};
    std::array<bool, 8> producer_used_{};
    std::array<uint64_t, 8> observed_drops_{};
};
// 实时线程走固定记录；其他线程允许后台格式化后投递。
void emitLogEvent(LogEventType type, LogModule module, int code,
                  int joint = -1, double value = 0, double lower = 0, double upper = 0,
                  LogLevel level = LogLevel::Error);
} // namespace rocos
