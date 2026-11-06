#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace rocos {
// 每个控制周期只记一次：突发包不能给未来周期充值，查询/重复包不计成功。
class ServoCommunication {
   public:
    void Reset() { *this = ServoCommunication{}; }
    void Record(bool fresh, uint64_t elapsed_cycles = 1) {
        const auto missed = elapsed_cycles > 0 ? elapsed_cycles - 1 : 0;
        for (uint64_t i = 0; i < std::min<uint64_t>(missed, 100); ++i) push(false);
        missing_ = std::min<uint64_t>(UINT32_MAX, missing_ + missed);
        push(fresh);
        missing_ = fresh ? 0 : std::min<uint64_t>(UINT32_MAX, missing_ + 1);
    }
    void RecordCycle(bool fresh, uint64_t elapsed_ns, uint64_t period_ns) {
        Record(fresh);
        ++control_cycles_;
        if (period_ns) {
            elapsed_periods_ += elapsed_ns / period_ns;
            remainder_ns_ += elapsed_ns % period_ns;
            elapsed_periods_ += remainder_ns_ / period_ns;
            remainder_ns_ %= period_ns;
        }
    }
    uint64_t ControlCycles() const { return control_cycles_; }
    uint64_t ElapsedPeriods() const { return elapsed_periods_; }
    double SuccessRate() const { return samples_ ? double(successes_) / samples_ : 0; }
    uint32_t MissingCycles() const { return missing_; }
    bool FullWindow() const { return samples_ == 100; }

   private:
    void push(bool success) {
        if (window_[index_]) --successes_;
        window_[index_] = success;
        if (success) ++successes_;
        index_ = (index_ + 1) % 100;
        samples_ = std::min(100u, samples_ + 1);
    }
    std::array<bool, 100> window_{};
    uint64_t control_cycles_{0}, elapsed_periods_{0}, remainder_ns_{0};
    uint32_t index_{0}, samples_{0}, successes_{0}, missing_{0};
};
}  // namespace rocos
