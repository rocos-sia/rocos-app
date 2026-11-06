#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace rocos {
using std::size_t;

// 仅一个生产者和一个消费者；初始化、销毁与队列使用不得并发。
template <class T, size_t Capacity> class SpscQueue {
    static_assert(Capacity > 1, "SPSC requires at least two slots");
    static_assert(std::atomic<size_t>::is_always_lock_free, "SPSC index must be lock-free");
public:
    bool tryPush(const T& value) noexcept(std::is_nothrow_copy_assignable<T>::value) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = (head + 1) % Capacity;
        if (next == tail_.load(std::memory_order_acquire)) return false;
        slots_[head] = value;
        head_.store(next, std::memory_order_release);
        return true;
    }
    bool tryPop(T& value) noexcept(std::is_nothrow_copy_assignable<T>::value) {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        value = slots_[tail];
        tail_.store((tail + 1) % Capacity, std::memory_order_release);
        return true;
    }
private:
    std::array<T, Capacity> slots_{};
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

enum class LogLevel : uint8_t { Trace, Debug, Info, Warn, Error, Critical };
enum class LogModule : uint8_t { Robot, Executor, PositionController, Model, MoveJoint, Hardware, MoveServo };
enum class LogEventType : uint8_t {
    Result, MotionFailed, ControllerFailed, CommandFailed, CycleFailed, CycleFinished,
    PositionLimit, VelocityLimit, ProfileFailed, EnteredError, EnteredStopped, EnteredRunning, ThreadJoinSkipped, HardwareSizeMismatch, HardwareThreadLimit, ServoStartPoseInvalid, ServoCommunicationFault
};
struct RtLogEvent {
    uint64_t elapsed_us{0}; // 启动日历基准加周期偏移，入口不读取时钟。
    uint64_t cycle{0};
    LogEventType type{LogEventType::Result};
    LogLevel level{LogLevel::Error};
    LogModule module{LogModule::Robot};
    int32_t code{0};
    int32_t joint{-1}; // 对外英文使用从 1 开始的关节编号。
    double value{0}, lower{0}, upper{0};
};
static_assert(std::is_trivially_copyable<RtLogEvent>::value, "RT event must be a value record");
static_assert(sizeof(RtLogEvent) <= 64, "RT event size budget");

class RtLogProducer {
public:
    bool tryPush(const RtLogEvent& event) noexcept {
        if (queue_.tryPush(event)) return true;
        dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
private:
    friend class LogManager;
    SpscQueue<RtLogEvent, 1024> queue_;
    std::atomic<uint64_t> dropped_{0};
};
static_assert(std::atomic<uint64_t>::is_always_lock_free, "RT counters must be lock-free");

// 线程进入周期循环前绑定已预分配的生产者，调用点只访问 TLS 和固定大小值。
void bindRtLogProducer(RtLogProducer* producer) noexcept;
void setRtLogEpoch(uint64_t epoch_us) noexcept;
void setRtLogClock(uint64_t epoch_us, uint64_t monotonic_us) noexcept;
// 复用调用方已经采样的单调时间，不在日志入口读取时钟。
void setRtLogMonotonicTime(uint64_t monotonic_us) noexcept;
void setRtLogCycle(uint64_t cycle, uint64_t elapsed_us) noexcept;
bool emitRtLog(LogEventType type, LogModule module, int code,
               int joint = -1, double value = 0, double lower = 0, double upper = 0,
               LogLevel level = LogLevel::Error) noexcept;
bool hasRtLogProducer() noexcept;

} // namespace rocos
