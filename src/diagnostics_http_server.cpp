#include "diagnostics_http_server.hpp"
#include "log_journal.hpp"
#include <charconv>
#include <unistd.h>

namespace rocos {
namespace {
void error(httplib::Response& response, int status, const std::string& message) {
    response.status = status;
    response.set_content(LogJson{{"success", false}, {"code", status == 400 ? 1001 : -3001},
                                 {"message", message}, {"data", nullptr}}.dump(), "application/json");
    if (status == 429) response.set_header("Retry-After", "5");
}
struct Quota {
    std::shared_ptr<std::atomic<size_t>> count;
    bool held{false};
    ~Quota() { if (held) --*count; }
};
std::shared_ptr<Quota> reserve(const std::shared_ptr<std::atomic<size_t>>& count, size_t limit) {
    auto lease = std::make_shared<Quota>();
    lease->count = count;
    auto current = count->load();
    while (current < limit) {
        if (count->compare_exchange_weak(current, current + 1)) { lease->held = true; return lease; }
    }
    return {};
}
std::string after(const httplib::Request& request) {
    const auto value = request.has_header("Last-Event-ID") ? request.get_header_value("Last-Event-ID") : request.get_param_value("after");
    if (value.size() > 128) throw std::invalid_argument("Log cursor is too long");
    return value;
}
} // namespace
void DiagnosticsConfig::validate() const {
    if (host.empty() || port < 0 || port > 65535 || max_sse_clients < 1 || max_sse_clients > 64 ||
        max_downloads < 1 || max_downloads > 16)
        throw std::invalid_argument("Invalid diagnostics configuration");
}
DiagnosticsHttpServer::DiagnosticsHttpServer(LogManager& logs, DiagnosticsConfig config)
    : logs_(logs), config_(std::move(config)), sse_count_(std::make_shared<std::atomic<size_t>>(0)),
      download_count_(std::make_shared<std::atomic<size_t>>(0)) {
    config_.validate();
    const auto workers = config_.max_sse_clients + config_.max_downloads + 4;
    server_.new_task_queue = [workers] { return new httplib::ThreadPool(workers, workers, 64); };
    server_.set_write_timeout(2);
    server_.set_read_timeout(2);
    server_.set_keep_alive_max_count(20);
    server_.set_default_headers({{"Access-Control-Allow-Origin", "*"},
        {"Access-Control-Expose-Headers", "Content-Disposition, X-Rocos-Log-Snapshot-Time, X-Rocos-SSE-Limit, Retry-After"}});
    routes();
}
DiagnosticsHttpServer::~DiagnosticsHttpServer() { stop(); }
int DiagnosticsHttpServer::start() {
    if (thread_.joinable()) throw std::logic_error("Diagnostics server already started");
    int port = config_.port;
    if (port == 0) port = server_.bind_to_any_port(config_.host);
    else if (!server_.bind_to_port(config_.host, port)) port = -1;
    if (port <= 0) throw std::runtime_error("Cannot bind diagnostics HTTP port");
    stopping_ = false;
    thread_ = std::thread([this] { server_.listen_after_bind(); });
    server_.wait_until_ready();
    return port;
}
void DiagnosticsHttpServer::stop() {
    stopping_ = true;
    {
        std::lock_guard<std::mutex> lock(subscriptions_mutex_);
        for (auto& weak : subscriptions_) if (auto subscription = weak.lock()) subscription->close();
        subscriptions_.clear();
    }
    server_.stop();
    if (thread_.joinable()) thread_.join();
}
void DiagnosticsHttpServer::routes() {
    std::vector<std::string> paths{"/api/logs", "/api/logs/stream", "/api/logs/file/latest"};
    if (logs_.detailedEvents()) { paths.push_back("/api/dualarm/logs"); paths.push_back("/api/dualarm/logs/stream"); }
    for (const auto& path : paths) {
        server_.Options(path, [](const httplib::Request&, httplib::Response& response) {
            response.status = 204;
            response.set_header("Access-Control-Allow-Methods", "GET, OPTIONS");
            response.set_header("Access-Control-Allow-Headers", "Last-Event-ID");
        });
    }
    for (const bool detailed : {false, true}) {
    if (detailed && !logs_.detailedEvents()) continue;
    const std::string base = detailed ? "/api/dualarm/logs" : "/api/logs";
    server_.Get(base, [this, detailed](const httplib::Request& request, httplib::Response& response) {
        if (stopping_) { error(response, 503, "Diagnostics service is stopping"); return; }
        try {
            size_t limit = 100;
            if (request.has_param("limit")) {
                auto text = request.get_param_value("limit");
                auto parsed = std::from_chars(text.data(), text.data() + text.size(), limit);
                if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) throw std::invalid_argument("Invalid limit");
            }
            auto data = detailed ? logs_.detailedEvents()->query(request.get_param_value("after"), limit) : logs_.query(request.get_param_value("after"), limit);
            response.set_content(LogJson{{"success", true}, {"code", 0}, {"message", "ok"}, {"data", data}}.dump(), "application/json");
        } catch (const std::invalid_argument& e) { error(response, 400, e.what()); }
    });
    server_.Get(base + "/stream", [this, detailed](const httplib::Request& request, httplib::Response& response) {
        response.set_header("X-Rocos-SSE-Limit", std::to_string(config_.max_sse_clients));
        if (stopping_) { error(response, 503, "Diagnostics service is stopping"); return; }
        try {
            auto cursor = after(request);
            // 在发送响应头以前校验并原子预占配额。
            if (!cursor.empty()) { if (detailed) logs_.detailedEvents()->query(cursor, 1); else logs_.query(cursor, 1); }
            auto lease = reserve(sse_count_, config_.max_sse_clients);
            if (!lease) { error(response, 429, "SSE connection limit reached"); return; }
            auto subscription = detailed ? logs_.detailedEvents()->subscribe(cursor) : logs_.subscribe(cursor);
            {
                std::lock_guard<std::mutex> lock(subscriptions_mutex_);
                if (stopping_) { subscription->close(); error(response, 503, "Diagnostics service is stopping"); return; }
                subscriptions_.erase(std::remove_if(subscriptions_.begin(), subscriptions_.end(),
                    [](const auto& weak) { return weak.expired(); }), subscriptions_.end());
                subscriptions_.push_back(subscription);
            }
            response.set_header("Cache-Control", "no-cache");
            response.set_header("X-Accel-Buffering", "no");
            auto heartbeat = std::make_shared<std::chrono::steady_clock::time_point>(std::chrono::steady_clock::now());
            auto connected = std::make_shared<bool>(false);
            response.set_chunked_content_provider("text/event-stream",
                [subscription, lease, heartbeat, connected](size_t, httplib::DataSink& sink) {
                    if (!*connected) { *connected = true; return sink.write(": connected\n\n", 13); }
                    std::string frame;
                    for (;;) {
                        if (!sink.is_writable() || !subscription->wait(frame, std::chrono::milliseconds(100))) return false;
                        if (!frame.empty()) return sink.write(frame.data(), frame.size());
                        if (std::chrono::steady_clock::now() - *heartbeat >= std::chrono::seconds(15)) {
                            *heartbeat = std::chrono::steady_clock::now();
                            return sink.write(": heartbeat\n\n", 13);
                        }
                    }
                }, [subscription, lease](bool) { subscription->close(); });
        } catch (const std::invalid_argument& e) { error(response, 400, e.what()); }
    });
    }
    server_.Get("/api/logs/file/latest", [this](const httplib::Request&, httplib::Response& response) {
        if (stopping_) { error(response, 503, "Diagnostics service is stopping"); return; }
        auto lease = reserve(download_count_, config_.max_downloads);
        if (!lease) { error(response, 429, "Download connection limit reached"); return; }
        auto snapshot = logs_.snapshot();
        if (snapshot.status != 200) { error(response, snapshot.status, "Log file snapshot is unavailable"); return; }
        auto file = snapshot.file;
        // 正文开始前尽可能发现读取错误；开始后错误仅通过连接中断报告。
        char probe;
        if (file->size && pread(file->fd, &probe, 1, 0) != 1) { error(response, 500, "Cannot read log file snapshot"); return; }
        response.set_header("Content-Disposition", "attachment; filename=\"" + file->filename + "\"");
        response.set_header("Cache-Control", "no-store");
        response.set_header("X-Rocos-Log-Snapshot-Time", file->timestamp);
        response.set_content_provider(file->size, "text/plain; charset=utf-8",
            [file, lease](size_t offset, size_t length, httplib::DataSink& sink) {
                char block[16384];
                const auto size = std::min({sizeof(block), length, file->size - offset});
                ssize_t count;
                do { count = pread(file->fd, block, size, static_cast<off_t>(offset)); } while (count < 0 && errno == EINTR);
                return count > 0 && sink.write(block, static_cast<size_t>(count));
            });
    });
}
} // namespace rocos
