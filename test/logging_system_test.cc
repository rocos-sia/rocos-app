#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/log_manager.hpp"
#include "src/logger.hpp"
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <new>
#include <unistd.h>
#include <sys/resource.h>
#include <csignal>

namespace {
thread_local bool count_allocations = false;
thread_local size_t allocations = 0;
}
void* operator new(std::size_t size) {
    if (count_allocations) ++allocations;
    if (auto ptr = std::malloc(size ? size : 1)) return ptr;
    throw std::bad_alloc();
}
void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete[](void* ptr) noexcept { std::free(ptr); }
void operator delete[](void* ptr, std::size_t) noexcept { std::free(ptr); }

namespace {
using namespace rocos;
namespace fs = std::filesystem;
struct Temp {
    fs::path path;
    Temp() {
        char pattern[] = "/tmp/rocos-logging-XXXXXX";
        auto p = mkdtemp(pattern);
        if (!p) throw std::runtime_error("mkdtemp failed");
        path = p;
    }
    ~Temp() { fs::remove_all(path); }
    LoggingConfig config() {
        LoggingConfig config;
        config.directory = path.string();
        config.console_level = LogLevel::Critical;
        config.file_level = LogLevel::Trace;
        return config;
    }
};
template <class F> bool until(F predicate, int timeout_ms = 5000) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    do {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}
std::string read(const std::shared_ptr<LogFileSnapshot>& file) {
    std::string result(file->size, '\0');
    size_t offset = 0;
    while (offset < result.size()) {
        auto count = pread(file->fd, result.data() + offset, result.size() - offset, offset);
        if (count <= 0) throw std::runtime_error("snapshot read failed");
        offset += count;
    }
    return result;
}
}

TEST_CASE("RT producer is fixed-size allocation-free and reports full queue immediately") {
    RtLogProducer producer;
    bindRtLogProducer(&producer);
    setRtLogEpoch(1000000);
    setRtLogCycle(12, 12000);
    allocations = 0;
    count_allocations = true;
    size_t accepted = 0;
    for (size_t i = 0; i < 100000; ++i)
        accepted += emitRtLog(LogEventType::VelocityLimit, LogModule::PositionController, -2310, 1, 1.8, 0, 1.5);
    count_allocations = false;
    bindRtLogProducer(nullptr);
    CHECK(allocations == 0);
    CHECK(accepted == 1023);
    CHECK(producer.dropped() == 100000 - accepted);
    CHECK(sizeof(RtLogEvent) <= 64);
}
TEST_CASE("SPSC concurrent producer and consumer preserve every accepted value") {
    SpscQueue<uint64_t, 1024> queue;
    std::thread producer([&] {
        for (uint64_t i = 0; i < 200000; ++i) while (!queue.tryPush(i)) std::this_thread::yield();
    });
    uint64_t value = 0;
    bool ordered = true;
    for (uint64_t expected = 0; expected < 200000; ++expected) {
        while (!queue.tryPop(value)) std::this_thread::yield();
        ordered &= value == expected;
    }
    producer.join();
    CHECK(ordered);
}
TEST_CASE("Query filters levels preserves codes paginates and rejects malformed cursors") {
    Temp temp;
    LogManager logs(temp.config());
    auto empty = logs.query("", 100);
    CHECK(empty["events"].empty());
    CHECK(empty["next_id"].get<std::string>().find(":0") != std::string::npos);
    for (auto level : {LogLevel::Trace, LogLevel::Debug, LogLevel::Info}) logs.submit(level, "Test", "ordinary");
    for (int i = 0; i < 3; ++i) {
        logs.submit(LogLevel::Warn, "Test", "warning " + std::to_string(i), -2310);
        REQUIRE(until([&] { return logs.query("", 100)["events"].size() == static_cast<size_t>(i + 1); }));
    }
    auto recent = logs.query("", 2);
    CHECK(recent["events"].size() == 2);
    CHECK(recent["events"][0]["message"] == "warning 1");
    auto first = logs.query(empty["next_id"], 1);
    CHECK(first["events"][0]["code"] == -2310);
    CHECK(first["events"][0].size() == 6);
    CHECK(first["has_more"] == true);
    auto next = logs.query(first["next_id"], 2);
    CHECK(next["events"].size() == 2);
    CHECK(next["has_more"] == false);
    CHECK_THROWS_AS(logs.query("bad", 1), std::invalid_argument);
    CHECK_THROWS_AS(logs.query("session:-1", 1), std::invalid_argument);
    CHECK_THROWS_AS(logs.query("session:18446744073709551616", 1), std::invalid_argument);
    CHECK_THROWS_AS(logs.query(first["next_id"].get<std::string>() + "0000", 1), std::invalid_argument);
    CHECK_THROWS_AS(logs.query("", 0), std::invalid_argument);
    CHECK_THROWS_AS(logs.query("", 501), std::invalid_argument);
    CHECK(logs.query("previous-session:42", 1)["gap"]["reason"] == "server_restarted");
    REQUIRE(until([&] { auto snap = logs.snapshot(); return snap.file && read(snap.file).find("warning 2") != std::string::npos; }));
    auto text = read(logs.snapshot().file);
    CHECK(text.find("[Test] [warn] warning 1") != std::string::npos);
    CHECK(text.find("\"id\"") == std::string::npos);
    CHECK(text.find("ordinary") != std::string::npos);
}
TEST_CASE("RT formatting uses command velocity numeric units and cycle timestamp") {
    Temp temp;
    LogManager logs(temp.config());
    auto producer = logs.createRtProducer();
    bindRtLogProducer(producer);
    setRtLogEpoch(1700000000000000);
    setRtLogCycle(12, 12000);
    emitLogEvent(LogEventType::VelocityLimit, LogModule::PositionController, -2310, 1, 1.8, 0, 1.5);
    bindRtLogProducer(nullptr);
    REQUIRE(until([&] { return logs.query("", 100)["events"].size() == 1; }));
    auto event = logs.query("", 1)["events"][0];
    CHECK(event["message"] == "Joint 2 command velocity exceeded the limit: 1.800000 rad/s; maximum allowed: 1.500000 rad/s.");
    CHECK(event["timestamp"] == logTimestamp(1700000000012000));
    CHECK(event["code"] == -2310);
    logs.releaseRtProducer(producer);
    CHECK(logs.createRtProducer() == producer);
}
TEST_CASE("Subscriptions replay atomically bound memory report gaps and close without heartbeat delay") {
    Temp temp;
    LogManager logs(temp.config());
    auto cursor = logs.query("", 1)["next_id"].get<std::string>();
    logs.submit(LogLevel::Warn, "Test", "before subscription");
    REQUIRE(until([&] { return logs.query("", 100)["events"].size() == 1; }));
    auto sub = logs.subscribe(cursor);
    std::string frame;
    REQUIRE(sub->wait(frame, std::chrono::milliseconds(100)));
    CHECK(frame.find("event: log") != std::string::npos);
    CHECK(frame.find("before subscription") != std::string::npos);
    auto latest = logs.subscribe("");
    REQUIRE(latest->wait(frame, std::chrono::milliseconds(5)));
    CHECK(frame.empty());
    for (int batch = 0; batch < 30; ++batch) {
        for (int i = 0; i < 160; ++i) logs.submit(LogLevel::Warn, "Test", "flood " + std::to_string(batch * 160 + i));
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    REQUIRE(until([&] { return logs.metrics()["history_evicted"].get<uint64_t>() > 0; }));
    CHECK(logs.query(cursor, 1)["gap"]["reason"] == "buffer_overrun");
    CHECK(logs.metrics()["client_gaps"].get<uint64_t>() > 0);
    REQUIRE(latest->wait(frame, std::chrono::milliseconds(100)));
    CHECK(frame.find("event: gap") != std::string::npos);
    CHECK(frame.find("id:") == std::string::npos);
    latest->close();
    CHECK_FALSE(latest->wait(frame, std::chrono::milliseconds(100)));
    logs.stop();
    CHECK_FALSE(sub->wait(frame, std::chrono::milliseconds(100)));
}
TEST_CASE("Serialized warning stays within 8 KiB and explicitly marks truncation") {
    Temp temp;
    LogManager logs(temp.config());
    logs.submit(LogLevel::Warn, "Test", std::string(20000, '\n'));
    REQUIRE(until([&] { return !logs.query("", 1)["events"].empty(); }));
    auto event = logs.query("", 1)["events"][0];
    CHECK(event.dump().size() <= 8192);
    CHECK(event["message"].get<std::string>().find("[truncated]") != std::string::npos);
    CHECK(event["code"].is_null());
}
TEST_CASE("Storage rotation retention day rollover and open-handle snapshots are stable") {
    Temp temp;
    auto config = temp.config();
    config.max_file_size = 80;
    config.max_files = 2;
    std::atomic<int64_t> now{1700000000000000};
    LogStorage storage(config, [&] { return now.load(); });
    auto empty = storage.snapshot();
    REQUIRE(empty.status == 200);
    CHECK(empty.file->size == 0);
    auto append = [&](const std::string& text) {
        REQUIRE(storage.enqueue(std::make_shared<FileLogRecord>(FileLogRecord{LogLevel::Error, text, now.load()})));
        REQUIRE(until([&] { auto snap = storage.snapshot(); return snap.file && read(snap.file) == text; }));
    };
    append(std::string(50, 'a') + "\n");
    auto original = storage.snapshot();
    auto original_text = read(original.file);
    append(std::string(50, 'b') + "\n");
    now += 86400LL * 1000000;
    append(std::string(10, 'c') + "\n");
    auto latest = storage.snapshot();
    CHECK(latest.file->filename != original.file->filename);
    CHECK(read(original.file) == original_text);
    CHECK_FALSE(fs::exists(temp.path / original.file->filename));
    CHECK(std::distance(fs::directory_iterator(temp.path), fs::directory_iterator{}) == 2);
    storage.stop();
    CHECK(read(original.file) == original_text);
}
TEST_CASE("File threshold preserves warning independently from console threshold") {
    Temp temp;
    auto config = temp.config();
    config.file_level = LogLevel::Info;
    LogStorage storage(config);
    REQUIRE(storage.enqueue(std::make_shared<FileLogRecord>(FileLogRecord{LogLevel::Debug, "hidden\n", logNowMicros()})));
    REQUIRE(storage.enqueue(std::make_shared<FileLogRecord>(FileLogRecord{LogLevel::Warn, "warning\n", logNowMicros()})));
    REQUIRE(until([&] { auto snapshot = storage.snapshot(); return snapshot.file && read(snapshot.file) == "warning\n"; }));
    config.file_level = LogLevel::Error;
    CHECK_THROWS_AS(config.validate(), std::invalid_argument);
}
TEST_CASE("Blocked storage times out snapshots and cannot block event processing or RT producers") {
    Temp temp;
    std::atomic<bool> stalled{true}, entered{false};
    LogManager logs(temp.config(), [&] {
        entered = true;
        while (stalled) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return logNowMicros();
    });
    REQUIRE(until([&] { return entered.load(); }));
    auto snapshot = logs.snapshot(std::chrono::milliseconds(5));
    CHECK(snapshot.status == 503);
    auto producer = logs.createRtProducer();
    for (int batch = 0; batch < 40; ++batch) {
        for (int i = 0; i < 200; ++i) producer->tryPush({static_cast<uint64_t>(logNowMicros()), 0, LogEventType::Result, LogLevel::Warn, LogModule::Robot});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    CHECK(logs.metrics()["file_dropped"].get<uint64_t>() > 0);
    CHECK_FALSE(logs.query("", 1)["events"].empty());
    stalled = false;
    logs.stop();
}
TEST_CASE("Unavailable storage records failures while alerts continue") {
    Temp temp;
    auto config = temp.config();
    auto file = temp.path / "regular-file";
    std::ofstream(file) << "file";
    config.directory = file.string();
    LogManager logs(config);
    logs.submit(LogLevel::Error, "Test", "storage unavailable");
    REQUIRE(until([&] { return !logs.query("", 1)["events"].empty(); }));
    REQUIRE(until([&] { return logs.metrics()["file_failures"].get<uint64_t>() > 0; }));
    CHECK(logs.snapshot().status == 503);
}
TEST_CASE("Module logger instances are shared safely across concurrent initialization") {
    std::array<Logger::logger_ptr, 16> instances;
    std::vector<std::thread> threads;
    for (size_t i = 0; i < instances.size(); ++i) threads.emplace_back([&, i] { instances[i] = Logger::getInstance("SameModule"); });
    for (auto& thread : threads) thread.join();
    for (const auto& instance : instances) CHECK(instance == instances.front());
}

TEST_CASE("RT overflow becomes an observable producer gap without an SSE id") {
    Temp temp;
    LogManager logs(temp.config());
    auto producer = logs.createRtProducer();
    auto sub = logs.subscribe("");
    RtLogEvent event{1700000000000000, 7, LogEventType::CycleFailed, LogLevel::Error, LogModule::Robot, -2310};
    for (size_t i = 0; i < 100000; ++i) producer->tryPush(event);
    REQUIRE(until([&] { return logs.metrics()["rt_dropped"].get<uint64_t>() > 0; }));
    std::string frame;
    bool found = false;
    for (int i = 0; i < 4098; ++i) {
        if (!sub->wait(frame, std::chrono::milliseconds(100))) break;
        if (frame.find("producer_overflow") != std::string::npos) { found = true; break; }
    }
    CHECK(found);
    CHECK(frame.find("id:") == std::string::npos);
    CHECK(frame.find("\"count\":") != std::string::npos);
}
TEST_CASE("History byte limit evicts records before the count limit and replay retains available events") {
    Temp temp;
    LogManager logs(temp.config());
    auto zero = logs.query("", 1)["next_id"].get<std::string>();
    for (int batch = 0; batch < 26; ++batch) {
        for (int i = 0; i < 100; ++i) logs.submit(LogLevel::Warn, "Test", std::string(7900, 'x'));
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    REQUIRE(until([&] { return logs.metrics()["history_evicted"].get<uint64_t>() > 0; }));
    auto metrics = logs.metrics();
    CHECK(metrics["history_bytes"].get<size_t>() <= 16 * 1024 * 1024);
    CHECK(metrics["history_records"].get<size_t>() < 4096);
    auto sub = logs.subscribe(zero);
    std::string frame;
    size_t replayed = 0;
    for (;;) {
        REQUIRE(sub->wait(frame, std::chrono::milliseconds(10)));
        if (frame.empty()) break;
        if (frame.find("event: log") != std::string::npos) ++replayed;
    }
    CHECK(replayed > 500); // 补发不应被较小的客户端队列静默截去。
}
TEST_CASE("Invalid UTF-8 is normalized consistently in queries and serialized output") {
    Temp temp;
    LogManager logs(temp.config());
    logs.submit(LogLevel::Warn, "Test", std::string("bad byte: ") + char(0xff));
    REQUIRE(until([&] { return !logs.query("", 1)["events"].empty(); }));
    auto event = logs.query("", 1)["events"][0];
    CHECK_NOTHROW(event.dump());
    CHECK(event["message"].get<std::string>().find("\xef\xbf\xbd") != std::string::npos);
}
TEST_CASE("RT timestamps reuse an existing monotonic sample without a clock read") {
    Temp temp;
    LogManager logs(temp.config());
    auto producer = logs.createRtProducer();
    bindRtLogProducer(producer);
    setRtLogClock(1700000000000000, 500000);
    setRtLogCycle(123, 0);
    setRtLogMonotonicTime(578000);
    allocations = 0;
    count_allocations = true;
    emitLogEvent(LogEventType::CycleFailed, LogModule::Robot, -2310);
    count_allocations = false;
    bindRtLogProducer(nullptr);
    CHECK(allocations == 0);
    REQUIRE(until([&] { return !logs.query("", 1)["events"].empty(); }));
    auto event = logs.query("", 1)["events"][0];
    CHECK(event["timestamp"] == logTimestamp(1700000000078000));
    CHECK(event["message"].get<std::string>().find("123") != std::string::npos);
}

TEST_CASE("Filesystem write and flush errors stay isolated from alert processing") {
    Temp temp;
    struct LimitGuard {
        rlimit previous{};
        decltype(std::signal(SIGXFSZ, SIG_IGN)) handler;
        LimitGuard() : handler(std::signal(SIGXFSZ, SIG_IGN)) {
            if (getrlimit(RLIMIT_FSIZE, &previous) != 0) throw std::runtime_error("getrlimit failed");
            auto limited = previous;
            limited.rlim_cur = 128;
            if (setrlimit(RLIMIT_FSIZE, &limited) != 0) throw std::runtime_error("setrlimit failed");
        }
        ~LimitGuard() { setrlimit(RLIMIT_FSIZE, &previous); std::signal(SIGXFSZ, handler); }
    } guard;
    LogManager logs(temp.config());
    logs.submit(LogLevel::Error, "Test", std::string(4000, 'x'));
    REQUIRE(until([&] { return !logs.query("", 1)["events"].empty(); }));
    REQUIRE(until([&] { return logs.metrics()["file_failures"].get<uint64_t>() > 0; }));
    CHECK(logs.snapshot().status == 503);
}

TEST_CASE("Servo guard messages identify the comparison and numerical threshold") {
    Temp temp;
    LogManager logs(temp.config());
    const std::vector<std::pair<LogEventType, std::string>> cases{
        {LogEventType::ServoIkDiscontinuity, "previous_accepted_ik="},
        {LogEventType::ServoReferenceDeviation, "limited_reference="},
        {LogEventType::ServoFeedbackPosition, "servo_feedback_position_error:"},
        {LogEventType::ServoFeedbackVelocity, "servo_feedback_velocity_error:"}};
    for (const auto& item : cases)
        logs.submitEvent({1700000000000000, 12, item.first, LogLevel::Error, LogModule::MoveServo,
                          -2314, 1, .1, .02, .003});
    REQUIRE(until([&] { return logs.query("", 100)["events"].size() == cases.size(); }));
    const auto events = logs.query("", 100)["events"];
    for (size_t i = 0; i < cases.size(); ++i) {
        const auto text = events[i]["message"].get<std::string>();
        CHECK(text.find(cases[i].second) != std::string::npos);
        CHECK(text.find("joint 2") != std::string::npos);
        CHECK(text.find("limit=0.003000000") != std::string::npos);
        CHECK(text.find("0.080000000") != std::string::npos);
    }
}
