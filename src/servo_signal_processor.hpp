#pragma once

#include <memory>
#include <vector>

#include "model_interface.hpp"
#include "servo_settings.hpp"

namespace rocos {
class ServoTrace;
struct ServoTraceRecord;
// 每个伺服会话独立持有的指令状态，仅由控制线程推进。
class ServoSignalProcessor {
   public:
    ServoSignalProcessor();
    ~ServoSignalProcessor();
    Result Reset(ModelInterface* model, const std::vector<JointDerivativeLimits>& limits,
                 const JntArray& actual, double dt, const ServoSettings& settings,
                 const Frame& tool = Frame::Identity());
    Result Step(const Reference& target, bool stopping, JntArray& output);
    // 实际反馈范围/新鲜度由调用方验证；此处比较同周期的参考与反馈。
    Result CheckFeedback(const JntArray& actual, const JntArray& velocity) const;
    const JntArray& Position() const;
    const JntArray& Velocity() const;
    const JntArray& Acceleration() const;
    bool AtRest() const;

   private:
    Result stepImpl(const Reference& target, bool stopping, JntArray& output,
                    ServoTraceRecord* trace);
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::unique_ptr<ServoTrace> trace_;
    uint64_t trace_sequence_{0};
};
}  // namespace rocos
