#include "log_manager.hpp"
#include "result.hpp"
#include "spdlog/fmt/fmt.h"
#include <charconv>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace rocos {
namespace {
std::mutex global_mutex;
std::unique_ptr<LogManager> global_manager;
const char* moduleName(LogModule module) {
    switch (module) {
        case LogModule::Robot: return "Robot";
        case LogModule::Executor: return "Executor";
        case LogModule::PositionController: return "PositionController";
        case LogModule::Model: return "Model";
        case LogModule::MoveJoint: return "MoveJoint";
        case LogModule::Hardware: return "Hardware";
        case LogModule::MoveServo: return "MoveServo";
    }
    return "Unknown";
}
std::string eventMessage(const RtLogEvent& event) {
    const auto result = to_string(static_cast<Result>(event.code));
    switch (event.type) {
        case LogEventType::ServoCommunicationFault:
            return fmt::format("Servo communication fault: success_rate={}, missing_cycles={}, minimum_rate={}; controlled stop latched.", event.value, event.lower, event.upper);
        case LogEventType::ServoStartPoseInvalid:
            return fmt::format("start_pose_invalid: joint {} first servo target differs from actual position by {:.9f}; maximum allowed per control cycle: {:.9f} (model joint units). Move to the trajectory start before starting servo.",
                               event.joint + 1, event.value, event.upper);
        case LogEventType::VelocityLimit:
            return fmt::format("Joint {} command velocity exceeded the limit: {:.6f} rad/s; maximum allowed: {:.6f} rad/s.",
                               event.joint + 1, event.value, event.upper);
        case LogEventType::PositionLimit:
            return fmt::format("Joint {} command position is outside the allowed range: {:.6f} rad; allowed: [{:.6f}, {:.6f}] rad.",
                               event.joint + 1, event.value, event.lower, event.upper);
        case LogEventType::MotionFailed: return fmt::format("Motion reference generation failed: {} ({}).", result, event.code);
        case LogEventType::ControllerFailed: return fmt::format("Controller command generation failed: {} ({}).", result, event.code);
        case LogEventType::CommandFailed: return fmt::format("Hardware command validation or update failed: {} ({}).", result, event.code);
        case LogEventType::CycleFailed: return fmt::format("Control cycle {} failed: {} ({}).", event.cycle, result, event.code);
        case LogEventType::CycleFinished: return fmt::format("Motion completed at control cycle {}: {} ({}).", event.cycle, result, event.code);
        case LogEventType::ProfileFailed:
            return fmt::format("Joint motion velocity profile update failed: result={}; normalized position={:.6f}, normalized velocity={:.6f} 1/s.",
                               event.value, event.lower, event.upper);
        case LogEventType::EnteredError: return "Robot entered the error state.";
        case LogEventType::EnteredStopped: return "Robot entered the stopped state.";
        case LogEventType::EnteredRunning: return "Robot entered the running state.";
        case LogEventType::ThreadJoinSkipped: return "Control thread attempted to join itself; join skipped.";
        case LogEventType::HardwareSizeMismatch:
            return fmt::format("Slave {} PDO variable {} size mismatch: configured {:.0f} bytes; requested {:.0f} bytes.",
                               event.joint, event.value, event.lower, event.upper);
        case LogEventType::HardwareThreadLimit:
            return fmt::format("Cannot register another hardware signal consumer: maximum allowed is {:.0f} threads.", event.upper);
        case LogEventType::Result: return fmt::format("{} operation failed: {} ({}).", moduleName(event.module), result, event.code);
    }
    return "Unknown log event.";
}
LogJson gapJson(const char* reason, const std::string& earliest, uint64_t count = 0) {
    LogJson gap{{"reason", reason}, {"earliest_id", earliest}};
    if (count) gap["count"] = count;
    return gap;
}
std::string gapSse(const LogJson& gap) { return "event: gap\ndata: " + gap.dump() + "\n\n"; }
std::string consoleTimestamp(int64_t epoch_us) {
    const time_t seconds = epoch_us / 1000000;
    tm local{};
    localtime_r(&seconds, &local);
    char date[32];
    strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &local);
    std::ostringstream out;
    out << date << '.' << std::setw(3) << std::setfill('0') << (epoch_us / 1000 % 1000);
    return out.str();
}
std::string consoleLine(LogLevel level, const std::string& module, const std::string& message, int64_t epoch_us) {
    return fmt::format("[{}] [{}] [{}] {}\n", consoleTimestamp(epoch_us), module,
                       logLevelName(level), message);
}
LogJson frameJson(const LogFrame& frame) {
    const auto start = frame.sse.find("data: ") + 6;
    return LogJson::parse(frame.sse.data() + start, frame.sse.data() + frame.sse.size() - 2);
}
} // namespace
void emitLogEvent(LogEventType type, LogModule module, int code, int joint,
                  double value, double lower, double upper, LogLevel level) {
    if (hasRtLogProducer()) {
        emitRtLog(type, module, code, joint, value, lower, upper, level);
        return;
    }
    LogManager::instance().submitEvent({static_cast<uint64_t>(logNowMicros()), 0, type, level, module,
                                      code, joint, value, lower, upper});
}
void LogSubscription::push(const std::shared_ptr<const LogFrame>& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) return;
    if (frames_.size() >= 4097 || bytes_ + frame->sse.size() > 17 * 1024 * 1024) {
        frames_.clear(); bytes_ = 0;
        auto gap = std::make_shared<LogFrame>();
        gap->sse = gapSse(gapJson("buffer_overrun", frame->id));
        bytes_ += gap->sse.size(); frames_.push_back(std::move(gap));
        if (gaps_) ++*gaps_;
    }
    bytes_ += frame->sse.size();
    frames_.push_back(frame);
    cv_.notify_one();
}
bool LogSubscription::wait(std::string& frame, std::chrono::milliseconds timeout) {
    std::shared_ptr<const LogFrame> selected;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, timeout, [&] { return closed_ || !frames_.empty(); });
        if (closed_) return false;
        if (frames_.empty()) { frame.clear(); return true; }
        selected = std::move(frames_.front()); frames_.pop_front(); bytes_ -= selected->sse.size();
    }
    frame = selected->sse;
    return true;
}
void LogSubscription::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true; frames_.clear(); bytes_ = 0; cv_.notify_all();
}
LogManager::LogManager(LoggingConfig config, std::function<int64_t()> storage_clock)
    : storage_(std::move(config), std::move(storage_clock)) {
    session_ = fmt::format("{:x}-{:x}", logNowMicros(), static_cast<unsigned>(getpid()));
    thread_ = std::thread([this] { run(); });
}
LogManager::~LogManager() { stop(); }
LogManager& LogManager::instance() {
    std::lock_guard<std::mutex> lock(global_mutex);
    if (!global_manager) global_manager = std::make_unique<LogManager>();
    return *global_manager;
}
void LogManager::configure(const LoggingConfig& config) {
    std::lock_guard<std::mutex> lock(global_mutex);
    if (global_manager) throw std::logic_error("LogManager already initialized");
    global_manager = std::make_unique<LogManager>(config);
}
void LogManager::shutdown() {
    std::lock_guard<std::mutex> lock(global_mutex);
    if (global_manager) global_manager->stop();
}
RtLogProducer* LogManager::createRtProducer() {
    std::lock_guard<std::mutex> lock(producers_mutex_);
    for (size_t i = 0; i < producer_used_.size(); ++i) {
        if (!producer_used_[i]) { producer_used_[i] = true; return &producers_[i]; }
    }
    throw std::runtime_error("RT log producer capacity exhausted");
}
void LogManager::releaseRtProducer(RtLogProducer* producer) {
    std::lock_guard<std::mutex> lock(producers_mutex_);
    for (size_t i = 0; i < producers_.size(); ++i) {
        if (&producers_[i] == producer) { producer_used_[i] = false; return; }
    }
}
void LogManager::submit(LogLevel level, std::string module, std::string message, int code, int64_t epoch_us) {
    // 普通日志入口允许格式化；队列有界，竞争时也直接丢弃。
    if (!running_) return;
    // 重新拷贝边界内的内容，避免短 size 字符串仍携带调用方巨大的 capacity。
    module = std::string(module.data(), std::min<size_t>(module.size(), 128));
    if (message.size() > 8192) message = message.substr(0, 8176) + " [truncated]";
    else message = std::string(message.data(), message.size());
    std::unique_lock<std::mutex> lock(pending_mutex_, std::try_to_lock);
    if (!lock || pending_.size() >= 1024) { ++nonrt_dropped_; return; }
    pending_.push_back({level, std::move(module), std::move(message), code, epoch_us ? epoch_us : logNowMicros()});
}
void LogManager::submitEvent(const RtLogEvent& event) {
    submit(event.level, moduleName(event.module), eventMessage(event), event.code, event.elapsed_us);
}
void LogManager::retain(LogFrame frame) {
    // 计入实际字符串容量、帧结构和保守的容器/共享句柄额度，不保留第二份 JSON 树。
    if (frame.id.empty()) frame.id = session_ + ":" + std::to_string(frame.sequence);
    frame.bytes = frame.sse.capacity() + frame.id.capacity() + sizeof(LogFrame) + 128;
    auto shared = std::make_shared<const LogFrame>(std::move(frame));
    history_bytes_ += shared->bytes;
    history_.push_back(std::move(shared));
    while (history_.size() > 4096 || history_bytes_ > 16 * 1024 * 1024) {
        history_bytes_ -= history_.front()->bytes; history_.pop_front(); ++evicted_;
    }
    for (auto it = subscribers_.begin(); it != subscribers_.end();) {
        if (auto subscriber = it->lock()) { subscriber->push(history_.back()); ++it; }
        else it = subscribers_.erase(it);
    }
}
void LogManager::publish(Pending record) {
    LogJson event{{"id", ""}, {"timestamp", logTimestamp(record.epoch_us)}, {"level", logLevelName(record.level)},
                  {"module", record.module}, {"code", record.code ? LogJson(record.code) : LogJson(nullptr)},
                  {"message", record.message}};
    std::string serialized;
    if (record.level >= LogLevel::Warn) {
        std::lock_guard<std::mutex> lock(history_mutex_);
        const auto id = session_ + ":" + std::to_string(++sequence_);
        event["id"] = id;
        serialized = event.dump(-1, ' ', false, LogJson::error_handler_t::replace);
        bool truncated = record.message.size() >= 8192;
        while (serialized.size() > 8192 || truncated) {
            auto& message = event["message"].get_ref<std::string&>();
            const auto remove = serialized.size() > 8192 ? serialized.size() - 8192 + 32 : 32;
            message.resize(message.size() > remove ? message.size() - remove : 0);
            message += " [truncated]";
            truncated = false;
            serialized = event.dump(-1, ' ', false, LogJson::error_handler_t::replace);
        }
        LogFrame frame{sequence_, true, id, "id: " + id + "\nevent: log\ndata: " + serialized + "\n\n", 0};
        retain(std::move(frame));
    } else {
        event.erase("id");
        serialized = event.dump(-1, ' ', false, LogJson::error_handler_t::replace);
    }
    // 文件和控制台使用 spdlog 风格的可读文本；HTTP 查询/SSE 继续使用上面的 JSON。
    storage_.enqueue(std::make_shared<FileLogRecord>(FileLogRecord{
        record.level, consoleLine(record.level, record.module, record.message, record.epoch_us), record.epoch_us}));
}
void LogManager::publishGap(uint64_t count) {
    std::lock_guard<std::mutex> lock(history_mutex_);
    auto gap = gapJson("producer_overflow", history_.empty() ? session_ + ":0" : session_ + ":" + std::to_string(history_.front()->sequence), count);
    auto text = gapSse(gap);
    retain({++sequence_, false, "", text, 0});
}
uint64_t LogManager::cursor(const std::string& after, bool& restarted) const {
    const auto colon = after.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == after.size()) throw std::invalid_argument("Invalid log cursor");
    for (size_t i = 0; i < colon; ++i) {
        char c = after[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-'))
            throw std::invalid_argument("Invalid log session");
    }
    uint64_t value = 0;
    auto parsed = std::from_chars(after.data() + colon + 1, after.data() + after.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != after.data() + after.size()) throw std::invalid_argument("Invalid log cursor sequence");
    restarted = after.substr(0, colon) != session_;
    if (!restarted && value > sequence_) throw std::invalid_argument("Log cursor is ahead of the latest record");
    return restarted ? 0 : value;
}
LogJson LogManager::queryLocked(const std::string& after, size_t limit) const {
    if (limit < 1 || limit > 500) throw std::invalid_argument("limit must be in [1, 500]");
    if (after.size() > 128) throw std::invalid_argument("Log cursor is too long");
    bool restarted = false;
    uint64_t start = after.empty() ? 0 : cursor(after, restarted);
    auto earliest = session_ + ":" + std::to_string(history_.empty() ? sequence_ : history_.front()->sequence);
    LogJson gap = nullptr;
    if (restarted) gap = gapJson("server_restarted", earliest);
    else if (!after.empty() && !history_.empty() && start < history_.front()->sequence - 1)
        gap = gapJson("buffer_overrun", earliest);
    size_t skip = 0;
    if (after.empty()) {
        size_t count = 0;
        for (const auto& frame : history_) if (frame->is_log) ++count;
        skip = count > limit ? count - limit : 0;
    }
    LogJson events = LogJson::array();
    uint64_t next = after.empty() || restarted ? 0 : start;
    uint64_t last_event = 0;
    bool more = false;
    for (const auto& frame : history_) {
        if (frame->sequence <= start) continue;
        if (frame->is_log && skip) { --skip; continue; }
        if (events.size() >= limit) { more = true; break; }
        next = frame->sequence;
        if (!frame->is_log) { if (gap.is_null()) gap = frameJson(*frame); }
        else { events.push_back(frameJson(*frame)); last_event = frame->sequence; }
    }
    if (last_event) next = last_event;
    return {{"events", events}, {"next_id", session_ + ":" + std::to_string(next)}, {"has_more", more}, {"gap", gap}};
}
LogJson LogManager::query(const std::string& after, size_t limit) const {
    std::lock_guard<std::mutex> lock(history_mutex_);
    return queryLocked(after, limit);
}
std::shared_ptr<LogSubscription> LogManager::subscribe(const std::string& after) {
    std::lock_guard<std::mutex> lock(history_mutex_);
    auto subscriber = std::make_shared<LogSubscription>();
    subscriber->gaps_ = &client_gaps_;
    if (!after.empty()) {
        bool restarted = false;
        auto start = cursor(after, restarted);
        if (restarted || (!history_.empty() && start < history_.front()->sequence - 1)) {
            auto gap = gapJson(restarted ? "server_restarted" : "buffer_overrun", session_ + ":" + std::to_string(history_.empty() ? sequence_ : history_.front()->sequence));
            subscriber->push(std::make_shared<const LogFrame>(LogFrame{0, false, "", gapSse(gap), 0}));
        }
        for (const auto& frame : history_) if (frame->sequence > start) subscriber->push(frame);
    }
    subscribers_.erase(std::remove_if(subscribers_.begin(), subscribers_.end(),
        [](const auto& weak) { return weak.expired(); }), subscribers_.end());
    if (!running_) subscriber->close();
    else subscribers_.push_back(subscriber);
    return subscriber;
}
SnapshotResult LogManager::snapshot(std::chrono::milliseconds timeout) { return storage_.snapshot(timeout); }
LogJson LogManager::metrics() const {
    uint64_t rt_dropped = 0;
    for (const auto& producer : producers_) rt_dropped += producer.dropped();
    std::lock_guard<std::mutex> lock(history_mutex_);
    return {{"rt_dropped", rt_dropped}, {"nonrt_dropped", nonrt_dropped_.load()},
            {"file_dropped", storage_.dropped()}, {"file_failures", storage_.failures()},
            {"history_evicted", evicted_}, {"history_records", history_.size()}, {"history_bytes", history_bytes_}, {"client_gaps", client_gaps_.load()}};
}
void LogManager::run() {
    uint64_t observed_nonrt = 0;
    for (;;) {
        bool work = false;
        for (size_t i = 0; i < producers_.size(); ++i) {
            RtLogEvent event;
            // 每轮有上限，避免实时洪水饿死普通队列。
            for (size_t n = 0; n < 128 && producers_[i].queue_.tryPop(event); ++n) {
                publish({event.level, moduleName(event.module), eventMessage(event), event.code, static_cast<int64_t>(event.elapsed_us)});
                work = true;
            }
            const auto lost = producers_[i].dropped();
            if (lost != observed_drops_[i]) { publishGap(lost - observed_drops_[i]); observed_drops_[i] = lost; }
        }
        std::deque<Pending> pending;
        { std::lock_guard<std::mutex> lock(pending_mutex_); pending.swap(pending_); }
        work = work || !pending.empty();
        for (auto& record : pending) publish(std::move(record));
        const auto lost = nonrt_dropped_.load();
        if (lost != observed_nonrt) { publishGap(lost - observed_nonrt); observed_nonrt = lost; }
        if (!work) {
            if (!running_) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}
void LogManager::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    {
        std::lock_guard<std::mutex> lock(history_mutex_);
        for (auto& weak : subscribers_) if (auto sub = weak.lock()) { sub->close(); sub->gaps_ = nullptr; }
        subscribers_.clear();
    }
    storage_.stop();
}
} // namespace rocos
