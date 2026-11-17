#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/diagnostics_http_server.hpp"
#include "src/log_journal.hpp"
#include <filesystem>
#include <fstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace {
using namespace rocos;
namespace fs = std::filesystem;
template <class F> bool until(F predicate, int timeout_ms = 5000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    do {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}
struct Fixture {
    fs::path directory;
    std::unique_ptr<LogManager> logs;
    std::unique_ptr<DiagnosticsHttpServer> server;
    int port;
    explicit Fixture(bool detailed = false) {
        char pattern[] = "/tmp/rocos-diagnostics-XXXXXX";
        auto path = mkdtemp(pattern);
        if (!path) throw std::runtime_error("mkdtemp failed");
        directory = path;
        LoggingConfig config;
        config.directory = path;
        config.file_level = LogLevel::Trace;
        config.console_level = LogLevel::Critical;
        config.max_file_size = 16 * 1024 * 1024;
        config.max_files = 1;
        logs = std::make_unique<LogManager>(config);
        if (detailed) logs->enableDetailedEvents({"test-left", "test-right"});
        DiagnosticsConfig diagnostics;
        diagnostics.host = "127.0.0.1";
        diagnostics.port = 0;
        server = std::make_unique<DiagnosticsHttpServer>(*logs, diagnostics);
        port = server->start();
    }
    ~Fixture() {
        server.reset();
        logs.reset();
        fs::remove_all(directory);
    }
    void alert(const std::string& message) {
        logs->submit(LogLevel::Warn, "Test", message);
        if (!until([&] { auto events = logs->query("", 1)["events"]; return !events.empty() && events.back()["message"] == message; }))
            throw std::runtime_error("Alert processing timed out");
    }
};
struct Socket {
    int fd{-1};
    explicit Socket(int port, const std::string& target, const std::string& extra = "") {
        fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) throw std::runtime_error("socket failed");
        timeval timeout{20, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) throw std::runtime_error("connect failed");
        const auto request = "GET " + target + " HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n" + extra + "\r\n";
        if (send(fd, request.data(), request.size(), MSG_NOSIGNAL) != static_cast<ssize_t>(request.size()))
            throw std::runtime_error("send failed");
    }
    ~Socket() { close(); }
    void close() { if (fd >= 0) { shutdown(fd, SHUT_RDWR); ::close(fd); fd = -1; } }
    std::string header() {
        std::string result;
        char c;
        while (result.size() < 16384 && recv(fd, &c, 1, 0) == 1) {
            result += c;
            if (result.size() >= 4 && result.compare(result.size() - 4, 4, "\r\n\r\n") == 0) break;
        }
        return result;
    }
    std::string through(const std::string& expected) {
        std::string result;
        char block[16384];
        while (result.find(expected) == std::string::npos) {
            auto count = recv(fd, block, sizeof(block), 0);
            if (count <= 0) break;
            result.append(block, count);
        }
        return result;
    }
};
}
TEST_CASE_FIXTURE(Fixture, "Diagnostics query is read-only validates cursors and streams raw text snapshots") {
    httplib::Client client("127.0.0.1", port);
    auto preflight = client.Options("/api/logs/stream", {
        {"Origin", "http://teach-pendant"},
        {"Access-Control-Request-Method", "GET"},
        {"Access-Control-Request-Headers", "Last-Event-ID"}});
    REQUIRE(preflight);
    CHECK(preflight->status == 204);
    CHECK(preflight->get_header_value("Access-Control-Allow-Methods").find("GET") != std::string::npos);
    CHECK(preflight->get_header_value("Access-Control-Allow-Headers").find("Last-Event-ID") != std::string::npos);
    auto empty = client.Get("/api/logs");
    REQUIRE(empty);
    CHECK(empty->status == 200);
    auto cursor = LogJson::parse(empty->body)["data"]["next_id"].get<std::string>();
    for (const auto& target : {"/api/logs?limit=0", "/api/logs?limit=501", "/api/logs?limit=-1", "/api/logs?limit=1x", "/api/logs?after=bad", "/api/logs/stream?after=bad"}) {
        auto response = client.Get(target);
        REQUIRE(response);
        CHECK(response->status == 400);
        CHECK(LogJson::parse(response->body)["code"] == 1001);
    }
    auto empty_file = client.Get("/api/logs/file/latest");
    REQUIRE(empty_file);
    CHECK(empty_file->status == 200);
    CHECK(empty_file->body.empty());
    CHECK(empty_file->get_header_value("Content-Length") == "0");
    logs->submit(LogLevel::Info, "Test", "ordinary file record");
    alert("visible alert");
    auto query = client.Get("/api/logs?after=" + cursor + "&limit=1");
    REQUIRE(query);
    auto event = LogJson::parse(query->body)["data"]["events"][0];
    CHECK(event.size() == 6);
    CHECK(event["level"] == "warn");
    CHECK(event["code"].is_null());
    auto response = client.Get("/api/logs/file/latest");
    REQUIRE(response);
    CHECK(response->status == 200);
    CHECK(response->get_header_value("Content-Type") == "text/plain; charset=utf-8");
    CHECK(response->get_header_value("Content-Disposition").find("attachment; filename=\"rocos-app_") == 0);
    CHECK(response->get_header_value("Content-Length") == std::to_string(response->body.size()));
    CHECK_FALSE(response->get_header_value("X-Rocos-Log-Snapshot-Time").empty());
    CHECK(response->get_header_value("Access-Control-Expose-Headers").find("Content-Disposition") != std::string::npos);
    CHECK(response->body.find("] [Test] [info] ordinary file record\n") != std::string::npos);
    CHECK(response->body.find("] [Test] [warn] visible alert\n") != std::string::npos);
    CHECK(response->body.find(event["id"].get<std::string>()) == std::string::npos);
    CHECK(response->body.find('\033') == std::string::npos);
}
TEST_CASE_FIXTURE(Fixture, "SSE quotas leave query workers available and release on disconnect") {
    std::vector<std::unique_ptr<Socket>> subscribers;
    for (int i = 0; i < 4; ++i) {
        auto socket = std::make_unique<Socket>(port, "/api/logs/stream");
        auto header = socket->header();
        CHECK(header.find("200 OK") != std::string::npos);
        CHECK(header.find("text/event-stream") != std::string::npos);
        CHECK(header.find("X-Rocos-SSE-Limit: 4") != std::string::npos);
        subscribers.push_back(std::move(socket));
    }
    httplib::Client client("127.0.0.1", port);
    client.set_read_timeout(2);
    auto rejected = client.Get("/api/logs/stream");
    REQUIRE(rejected);
    CHECK(rejected->status == 429);
    CHECK(rejected->get_header_value("Retry-After") == "5");
    CHECK(rejected->get_header_value("X-Rocos-SSE-Limit") == "4");
    CHECK(rejected->get_header_value("Content-Type") == "application/json");
    auto query = client.Get("/api/logs");
    REQUIRE(query);
    CHECK(query->status == 200);
    subscribers.front().reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    Socket replacement(port, "/api/logs/stream");
    CHECK(replacement.header().find("200 OK") != std::string::npos);
    alert("live warning");
    CHECK(replacement.through("live warning").find("event: log") != std::string::npos);
    auto begin = std::chrono::steady_clock::now();
    server->stop();
    CHECK(std::chrono::steady_clock::now() - begin < std::chrono::seconds(3));
}
TEST_CASE_FIXTURE(Fixture, "SSE Last-Event-ID takes precedence replays in order and heartbeats carry no ID") {
    auto zero = logs->query("", 1)["next_id"].get<std::string>();
    alert("first warning");
    auto first = logs->query("", 1)["next_id"].get<std::string>();
    alert("second warning");
    Socket stream(port, "/api/logs/stream?after=malformed", "Last-Event-ID: " + first + "\r\n");
    REQUIRE(stream.header().find("200 OK") != std::string::npos);
    auto replay = stream.through("second warning");
    CHECK(replay.find("first warning") == std::string::npos);
    CHECK(replay.find("event: log") != std::string::npos);
    auto events = logs->query(first, 1)["events"];
    CHECK(replay.find("id: " + events[0]["id"].get<std::string>()) != std::string::npos);
    auto heartbeat = stream.through(": heartbeat");
    CHECK(heartbeat.find(": heartbeat") != std::string::npos);
    CHECK(heartbeat.find("event:") == std::string::npos);
    CHECK(heartbeat.find("id:") == std::string::npos);
    Socket restarted(port, "/api/logs/stream?after=previous-session:1");
    REQUIRE(restarted.header().find("200 OK") != std::string::npos);
    auto gap = restarted.through("second warning");
    CHECK(gap.find("event: gap\ndata: {\"earliest_id\"") != std::string::npos);
    CHECK(gap.find("server_restarted") != std::string::npos);
    Socket from_zero(port, "/api/logs/stream?after=" + zero);
    REQUIRE(from_zero.header().find("200 OK") != std::string::npos);
    auto both = from_zero.through("second warning");
    CHECK(both.find("first warning") < both.find("second warning"));
}
TEST_CASE_FIXTURE(Fixture, "Slow snapshot download keeps fixed length across writes rotation and cleanup") {
    const std::string payload(7900, 'x');
    for (int batch = 0; batch < 30; ++batch) {
        for (int i = 0; i < 50; ++i) logs->submit(LogLevel::Info, "Test", payload);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(until([&] { auto snap = logs->snapshot(); return snap.file && snap.file->size > 8 * 1024 * 1024; }));
    std::atomic<bool> receiving{false}, release{false};
    std::string body;
    size_t content_length = 0;
    bool successful = false;
    struct ThreadGuard {
        std::atomic<bool>& release;
        std::thread thread;
        ~ThreadGuard() { release = true; if (thread.joinable()) thread.join(); }
    } download{release, std::thread([&] {
        httplib::Client client("127.0.0.1", port);
        client.set_read_timeout(10);
        auto result = client.Get("/api/logs/file/latest", [&](const httplib::Response& response) {
            content_length = std::stoull(response.get_header_value("Content-Length"));
            return response.status == 200;
        }, [&](const char* data, size_t size) {
            receiving = true;
            while (!release) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            body.append(data, size);
            return true;
        });
        successful = result && result->status == 200;
    })};
    REQUIRE(until([&] { return receiving.load(); }));
    httplib::Client query("127.0.0.1", port);
    auto rejected = query.Get("/api/logs/file/latest");
    REQUIRE(rejected);
    CHECK(rejected->status == 429);
    Socket stream(port, "/api/logs/stream");
    REQUIRE(stream.header().find("200 OK") != std::string::npos);
    alert("warning during download");
    CHECK(stream.through("warning during download").find("event: log") != std::string::npos);
    // 跨过 16 MiB 上限，max_files=1 删除旧目录项，下载仍使用原 fd。
    for (int batch = 0; batch < 30; ++batch) {
        for (int i = 0; i < 50; ++i) logs->submit(LogLevel::Info, "Test", payload);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    release = true;
    download.thread.join();
    CHECK(successful);
    CHECK(body.size() == content_length);
    CHECK(body.find("warning during download") == std::string::npos);
    CHECK_FALSE(body.empty());
    CHECK(body.back() == '\n');
    auto restored = query.Get("/api/logs/file/latest");
    REQUIRE(restored);
    CHECK(restored->status == 200);
}

TEST_CASE("Dual-arm journal preserves arm context across concurrent producers and legacy schema") {
    Fixture f(true);
    auto producer = [&](LogScope scope, int ecat_id) {
        ScopedLogContext context({scope, ecat_id});
        f.logs->submit(LogLevel::Info, "DualArm.Http", "POST robot/movej accepted", 0, 0,
                       {"POST robot/movej", "request-" + std::to_string(ecat_id), "accepted", 200, 1.5});
        auto* rt = f.logs->createRtProducer();
        bindRtLogProducer(rt);
        setRtLogEpoch(logNowMicros());
        emitRtLog(LogEventType::CycleFinished, LogModule::Robot, 0, -1, 0, 0, 0, LogLevel::Info);
        emitRtLog(LogEventType::EnteredError, LogModule::Robot, -1);
        bindRtLogProducer(nullptr);
        f.logs->releaseRtProducer(rt);
    };
    std::thread left([&] { producer(LogScope::Left, 0); });
    std::thread right([&] { producer(LogScope::Right, 1); });
    left.join(); right.join();
    CHECK(currentLogContext().scope == LogScope::Process);
    REQUIRE(until([&] { return f.logs->detailedEvents()->query("", 100)["events"].size() == 6; }));
    httplib::Client client("127.0.0.1", f.port);
    auto response = client.Get("/api/dualarm/logs?limit=100");
    REQUIRE(response); REQUIRE(response->status == 200);
    auto data = LogJson::parse(response->body)["data"];
    int left_count = 0, right_count = 0;
    for (const auto& event : data["events"]) {
        const bool is_left = event["arm"] == "left";
        CHECK(event["ecat_id"] == (is_left ? 0 : 1));
        CHECK(event["device_id"] == (is_left ? "test-left" : "test-right"));
        is_left ? ++left_count : ++right_count;
        if (event["category"] == "operation") CHECK(event["operation"]["outcome"] == "accepted");
    }
    CHECK(left_count == 3); CHECK(right_count == 3);
    auto legacy = f.logs->query("", 100)["events"];
    REQUIRE(legacy.size() == 2);
    for (const auto& event : legacy) { CHECK(event.size() == 6); CHECK(event["level"] == "error"); }
    Socket stream(f.port, "/api/dualarm/logs/stream?after=" + data["events"][0]["id"].get<std::string>());
    CHECK(stream.header().find("200 OK") != std::string::npos);
    CHECK(stream.through(data["events"].back()["id"]).find("\"device_id\"") != std::string::npos);
    stream.close();
    CHECK(client.Get("/api/dualarm/logs?limit=0")->status == 400);
    CHECK(client.Get("/api/dualarm/logs/stream?after=invalid")->status == 400);
    auto restarted = f.logs->detailedEvents()->query("previous-session:1", 100);
    CHECK(restarted["gap"]["reason"] == "server_restarted");
    auto file = client.Get("/api/logs/file/latest");
    REQUIRE(file); CHECK(file->body.find("arm=left ecat_id=0") != std::string::npos);
    CHECK(file->body.find("arm=right ecat_id=1") != std::string::npos);
    CHECK(file->body.find("request_id=request-0 outcome=accepted") != std::string::npos);
}
TEST_CASE("Dual-arm journal bounds history and replays gaps") {
    LogJournal journal;
    for (int i = 0; i < 4200; ++i) journal.publish({{"message", "entry"}});
    const auto data = journal.query("", 1);
    const auto id = data["next_id"].get<std::string>();
    const auto session = id.substr(0, id.find(':'));
    auto gap = journal.query(session + ":1", 500);
    CHECK(gap["gap"]["reason"] == "buffer_overrun");
    CHECK(gap["has_more"] == true);
    journal.publishGap(3);
    CHECK(journal.query(id, 10)["gap"]["count"] == 3);
    CHECK(journal.query(id, 10)["next_id"] != id);
    auto subscription = journal.subscribe(id);
    std::string frame;
    REQUIRE(subscription->wait(frame, std::chrono::milliseconds(100)));
    CHECK(frame.find("producer_overflow") != std::string::npos);
    journal.stop(); CHECK_FALSE(subscription->wait(frame, std::chrono::milliseconds(1)));
}
TEST_CASE_FIXTURE(Fixture, "Single-arm diagnostics does not expose the dual-arm journal") {
    httplib::Client client("127.0.0.1", port);
    CHECK(client.Get("/api/dualarm/logs")->status == 404);
}
