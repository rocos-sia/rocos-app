#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <new>
#include <string>
#include <system_error>

namespace rocos {

// 独立诊断文件，不改变 UDP/API 布局。每个流只有一个生产者，停止后导出。
class ServoTimingTrace {
 public:
    static constexpr uint64_t kCapacity = 65536;
    struct UdpRecord {
        uint64_t message_id{}, receive_ns{}, locked_ns{}, ready_ns{}, published_ns{};
        // 1: 发布；2: 停止中拒绝；3: 乱序/重复拒绝。只记录已通过来源/会话检查的正序号包。
        uint64_t disposition{}, reserved[2]{};
    };
    struct ControlRecord {
        uint64_t cycle{}, entry_ns{}, locked_ns{}, consumed_ns{}, consumed_id{};
        uint64_t reference_id{}, applied_id{}, completed_ns{};
        int64_t result{};
        uint64_t reserved{};
    };
    struct alignas(64) Header {
        char magic[8]{};
        uint64_t version{}, capacity{}, udp_size{}, control_size{};
        uint64_t created_mono_ns{}, created_real_ns{};
        char session[40]{};
        alignas(64) std::atomic<uint64_t> udp_count{0};
        alignas(64) std::atomic<uint64_t> control_count{0};
    };
    static_assert(sizeof(UdpRecord) == 64 && sizeof(ControlRecord) == 80, "Trace layout");
    static_assert(sizeof(Header) == 256, "Trace header layout");
    static_assert(std::atomic<uint64_t>::is_always_lock_free, "Trace needs lock-free counters");

    static uint64_t Now() noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    static std::unique_ptr<ServoTimingTrace> Create(const std::string& session,
                                                   std::string& error) noexcept {
        const auto* enabled = std::getenv("ROCOS_SERVO_TIMING_TRACE");
        if (!enabled || std::strcmp(enabled, "1") != 0) return nullptr;
        try {
            auto trace = std::unique_ptr<ServoTimingTrace>(new ServoTimingTrace);
            const std::string directory = "/dev/shm/rocos-servo-timing";
            std::filesystem::create_directory(directory);
            trace->path_ = directory + "/servo_" + std::to_string(getpid()) + "_" +
                           session + ".bin";
            const int fd = open(trace->path_.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
            if (fd < 0) throw std::system_error(errno, std::generic_category(), "open trace");
            // 先预留 tmpfs 空间，防止运行期首次访问时才遇到 ENOSPC/SIGBUS。
            const int rc = posix_fallocate(fd, 0, kBytes);
            if (rc) {
                close(fd);
                unlink(trace->path_.c_str());
                throw std::system_error(rc, std::generic_category(), "reserve trace");
            }
            trace->memory_ = mmap(nullptr, kBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            const int map_error = errno;
            close(fd);
            if (trace->memory_ == MAP_FAILED) {
                unlink(trace->path_.c_str());
                throw std::system_error(map_error, std::generic_category(), "map trace");
            }
            // 在线程启动前预触页。mlock 失败时关闭诊断，不改变运动/保护结果。
            std::memset(trace->memory_, 0, kBytes);
            if (mlock(trace->memory_, kBytes) != 0) {
                const int lock_error = errno;
                unlink(trace->path_.c_str());
                throw std::system_error(lock_error, std::generic_category(), "mlock trace");
            }
            trace->header_ = new (trace->memory_) Header;
            auto* udp = reinterpret_cast<UdpRecord*>(static_cast<char*>(trace->memory_) + sizeof(Header));
            auto* control = reinterpret_cast<ControlRecord*>(static_cast<char*>(trace->memory_) +
                sizeof(Header) + sizeof(UdpRecord) * kCapacity);
            for (uint64_t i = 0; i < kCapacity; ++i) {
                new (&udp[i]) UdpRecord;
                new (&control[i]) ControlRecord;
            }
            auto& h = *trace->header_;
            h.version = 1;
            h.capacity = kCapacity;
            h.udp_size = sizeof(UdpRecord);
            h.control_size = sizeof(ControlRecord);
            h.created_mono_ns = Now();
            h.created_real_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            std::memcpy(h.session, session.data(), std::min(session.size(), sizeof(h.session) - 1));
            std::memcpy(h.magic, "RCSTIME1", 8);
            return trace;
        } catch (const std::exception& e) {
            try { error = e.what(); } catch (...) {}
            return nullptr;
        }
    }
    ~ServoTimingTrace() {
        if (memory_ != MAP_FAILED) {
            munlock(memory_, kBytes);
            munmap(memory_, kBytes);
        }
    }
    ServoTimingTrace(const ServoTimingTrace&) = delete;
    ServoTimingTrace& operator=(const ServoTimingTrace&) = delete;

    void PushUdp(const UdpRecord& record) noexcept {
        auto* records = reinterpret_cast<UdpRecord*>(static_cast<char*>(memory_) + sizeof(Header));
        records[udp_count_ % kCapacity] = record;
        header_->udp_count.store(++udp_count_, std::memory_order_release);
    }
    void PushControl(const ControlRecord& record) noexcept {
        auto* records = reinterpret_cast<ControlRecord*>(static_cast<char*>(memory_) +
            sizeof(Header) + sizeof(UdpRecord) * kCapacity);
        records[control_count_ % kCapacity] = record;
        header_->control_count.store(++control_count_, std::memory_order_release);
    }
    const std::string& Path() const { return path_; }

 private:
    ServoTimingTrace() = default;
    static constexpr size_t kBytes = sizeof(Header) +
        kCapacity * (sizeof(UdpRecord) + sizeof(ControlRecord));
    void* memory_{MAP_FAILED};
    Header* header_{nullptr};
    // 不共享每周期写索引，避免两个生产者争用缓存行。
    alignas(64) uint64_t udp_count_{0};
    alignas(64) uint64_t control_count_{0};
    std::string path_;
};
}  // namespace rocos
