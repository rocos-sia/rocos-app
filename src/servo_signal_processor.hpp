#pragma once

#include <memory>
#include <vector>

#include "model_interface.hpp"
#include "servo_settings.hpp"

namespace rocos {
// 每个伺服会话独立持有的指令状态，仅由控制线程推进。
class ServoSignalProcessor {
   public:
    ServoSignalProcessor();
    ~ServoSignalProcessor();
    Result Reset(ModelInterface* model, const std::vector<JointDerivativeLimits>& limits,
                 const JntArray& actual, double dt, const ServoSettings& settings,
                 const Frame& tool = Frame::Identity());
    Result Step(const Reference& target, bool stopping, JntArray& output);
    const JntArray& Position() const;
    const JntArray& Velocity() const;
    const JntArray& Acceleration() const;
    bool AtRest() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace rocos
