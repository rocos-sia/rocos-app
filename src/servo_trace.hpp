#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace rocos {
constexpr size_t kServoTraceJoints = 10;
// 仅诊断格式，不是 UDP/API 契约。v1 使用本机小端二进制布局。
#pragma pack(push, 1)
struct ServoIkTrace {
    int32_t result{std::numeric_limits<int32_t>::min()};
    int32_t backend_result{0}, count{0}, selected{-1};
    uint64_t solve_ns{0}, capture_ns{0};
    std::array<double, kServoTraceJoints> seed{}, output{};
    std::array<double, 16> target{};
    std::array<std::array<double, kServoTraceJoints>, 2> candidates{};
    std::array<double, 2> scores{};
};
struct ServoTraceRecord {
    uint64_t sequence{0}, time_ns{0}, elapsed_ns{0};
    uint32_t joints{0};
    int32_t result{std::numeric_limits<int32_t>::min()};
    uint8_t stopping{0};
    double dt{0}, alpha{0};
    std::array<double, kServoTraceJoints> q{}, filtered{}, v{}, a{};
    std::array<ServoIkTrace, 2> ik{};  // 保留旧格式：槽 0 不调用，槽 1 为滤波位姿 IK。
    std::array<double, kServoTraceJoints> goal{}, next{}, next_v{}, next_a{};
    uint32_t brake_mask{0};
};
#pragma pack(pop)

class ServoTrace {
   public:
    // 由 ROCOS_SERVO_TRACE_DIR 开启；只在创建会话时读取环境变量。
    static std::unique_ptr<ServoTrace> Create();
    ~ServoTrace();
    void Push(const ServoTraceRecord& record);

   private:
    struct Impl;
    explicit ServoTrace(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

uint64_t ServoTraceNow();
ServoIkTrace* CurrentServoIkTrace();
class ServoIkTraceScope {
   public:
    explicit ServoIkTraceScope(ServoIkTrace* record);
    ~ServoIkTraceScope();

   private:
    ServoIkTrace* previous_;
};
}  // namespace rocos
