#include "log_event.hpp"

namespace rocos {
namespace {
thread_local LogContext log_context{};
thread_local RtLogProducer* rt_producer = nullptr;
thread_local uint64_t rt_cycle = 0, rt_elapsed = 0, rt_epoch = 0, rt_origin = 0;
thread_local bool rt_clock_bound = false;
} // namespace
LogContext currentLogContext() noexcept { return log_context; }
ScopedLogContext::ScopedLogContext(LogContext context) noexcept : previous_(log_context) { log_context = context; }
ScopedLogContext::~ScopedLogContext() { log_context = previous_; }
void bindRtLogProducer(RtLogProducer* producer) noexcept { rt_producer = producer; rt_clock_bound = false; }
void setRtLogEpoch(uint64_t epoch_us) noexcept { rt_epoch = epoch_us; rt_clock_bound = false; }
void setRtLogClock(uint64_t epoch_us, uint64_t monotonic_us) noexcept {
    rt_epoch = epoch_us; rt_origin = monotonic_us; rt_elapsed = 0; rt_clock_bound = true;
}
void setRtLogMonotonicTime(uint64_t monotonic_us) noexcept {
    if (rt_clock_bound && monotonic_us >= rt_origin) rt_elapsed = monotonic_us - rt_origin;
}
void setRtLogCycle(uint64_t cycle, uint64_t elapsed_us) noexcept { rt_cycle = cycle; if (!rt_clock_bound) rt_elapsed = elapsed_us; }
bool hasRtLogProducer() noexcept { return rt_producer != nullptr; }
bool emitRtLog(LogEventType type, LogModule module, int code, int joint,
               double value, double lower, double upper, LogLevel level) noexcept {
    if (!rt_producer) return false;
    return rt_producer->tryPush({rt_epoch + rt_elapsed, rt_cycle, type, level, module, code, joint, value, lower, upper, currentLogContext()});
}
} // namespace rocos
