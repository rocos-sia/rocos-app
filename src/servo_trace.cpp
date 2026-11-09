#include "servo_trace.hpp"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>

namespace rocos {
namespace {
thread_local ServoIkTrace* current_ik_trace = nullptr;
}
uint64_t ServoTraceNow() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
ServoIkTrace* CurrentServoIkTrace() { return current_ik_trace; }
ServoIkTraceScope::ServoIkTraceScope(ServoIkTrace* record)
    : previous_(current_ik_trace) {
    current_ik_trace = record;
}
ServoIkTraceScope::~ServoIkTraceScope() { current_ik_trace = previous_; }

struct ServoTrace::Impl {
    static constexpr size_t capacity = 4096;
    std::unique_ptr<ServoTraceRecord[]> ring{new ServoTraceRecord[capacity]};
    std::atomic<uint64_t> written{0}, read{0}, dropped{0};
    std::atomic<bool> done{false};
    uint64_t saved{0};
    bool io_error{false};
    std::string path;
    FILE* file{nullptr};
    std::thread writer;

    explicit Impl(const char* directory) {
        path = std::string(directory) + "/servo_" + std::to_string(getpid()) +
               "_" + std::to_string(ServoTraceNow());
        file = std::fopen((path + ".bin").c_str(), "wb");
        if (!file) throw std::runtime_error("Cannot open servo trace file");
        try {
            writer = std::thread([this] {
                while (!done.load(std::memory_order_acquire) ||
                       read.load() < written.load(std::memory_order_acquire)) {
                    const auto r = read.load(std::memory_order_relaxed);
                    if (r < written.load(std::memory_order_acquire)) {
                        if (std::fwrite(&ring[r % capacity],
                                        sizeof(ServoTraceRecord), 1, file) == 1)
                            ++saved;
                        else
                            io_error = true;
                        read.store(r + 1, std::memory_order_release);
                    } else {
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(1));
                    }
                }
            });
        } catch (...) {
            std::fclose(file);
            throw;
        }
    }
    ~Impl() {
        done.store(true, std::memory_order_release);
        if (writer.joinable()) writer.join();
        if (std::fclose(file) != 0) io_error = true;
        if (auto* meta = std::fopen((path + ".json").c_str(), "w")) {
            std::fprintf(meta,
                         "{\"version\":1,\"record_bytes\":%zu,\"records\":%llu,"
                         "\"dropped\":%llu,\"io_error\":%s}\n",
                         sizeof(ServoTraceRecord),
                         static_cast<unsigned long long>(saved),
                         static_cast<unsigned long long>(dropped.load()),
                         io_error ? "true" : "false");
            std::fclose(meta);
        }
    }
};
ServoTrace::ServoTrace(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
ServoTrace::~ServoTrace() = default;
std::unique_ptr<ServoTrace> ServoTrace::Create() {
    const auto* directory = std::getenv("ROCOS_SERVO_TRACE_DIR");
    if (!directory || !*directory) return nullptr;
    try {
        return std::unique_ptr<ServoTrace>(
            new ServoTrace(std::make_unique<Impl>(directory)));
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Servo trace disabled: %s\n", error.what());
        return nullptr;
    }
}
void ServoTrace::Push(const ServoTraceRecord& record) {
    const auto w = impl_->written.load(std::memory_order_relaxed);
    if (w - impl_->read.load(std::memory_order_acquire) >= Impl::capacity) {
        impl_->dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    impl_->ring[w % Impl::capacity] = record;
    impl_->written.store(w + 1, std::memory_order_release);
}
}  // namespace rocos
