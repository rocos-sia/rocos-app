#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "src/servo_timing_trace.hpp"
#include "test/doctest.h"

#include <fstream>

namespace {
struct TimingEnvironment {
    std::string previous;
    bool existed;
    TimingEnvironment() : existed(std::getenv("ROCOS_SERVO_TIMING_TRACE") != nullptr) {
        if (existed) previous = std::getenv("ROCOS_SERVO_TIMING_TRACE");
        setenv("ROCOS_SERVO_TIMING_TRACE", "1", 1);
    }
    ~TimingEnvironment() {
        if (existed) setenv("ROCOS_SERVO_TIMING_TRACE", previous.c_str(), 1);
        else unsetenv("ROCOS_SERVO_TIMING_TRACE");
    }
};
std::string session(const char* suffix) {
    return "offline_test_" + std::to_string(getpid()) + "_" + suffix;
}
}
TEST_CASE("Servo timing is opt in and does not create disabled captures") {
    TimingEnvironment env;
    unsetenv("ROCOS_SERVO_TIMING_TRACE");
    std::string error;
    CHECK_FALSE(rocos::ServoTimingTrace::Create(session("off"), error));
    CHECK(error.empty());
}
TEST_CASE("Servo timing pins rings and retains the latest complete records") {
    TimingEnvironment env;
    std::string error;
    auto trace = rocos::ServoTimingTrace::Create(session("ring"), error);
    INFO(error);
    REQUIRE(trace);
    const auto path = trace->Path();
    const auto n = rocos::ServoTimingTrace::kCapacity + 11;
    for (uint64_t i = 1; i <= n; ++i) {
        rocos::ServoTimingTrace::UdpRecord udp{};
        udp.message_id = i;
        udp.receive_ns = i * 1000;
        udp.locked_ns = udp.receive_ns + 10;
        udp.ready_ns = udp.locked_ns + 10;
        udp.published_ns = udp.ready_ns + 10;
        udp.disposition = 1;
        trace->PushUdp(udp);
        rocos::ServoTimingTrace::ControlRecord control{};
        control.cycle = i;
        control.consumed_id = control.reference_id = control.applied_id = i;
        control.result = i == n ? -1008 : 0;
        trace->PushControl(control);
    }
    trace.reset(); // 故障/对象销毁后数据仍可读取，无需实时线程 flush。
    std::ifstream file(path, std::ios::binary);
    std::array<char, 256> header{};
    REQUIRE(file.read(header.data(), header.size()));
    CHECK(std::string(header.data(), 8) == "RCSTIME1");
    uint64_t udp_count, control_count;
    std::memcpy(&udp_count, header.data() + 128, 8);
    std::memcpy(&control_count, header.data() + 192, 8);
    CHECK(udp_count == n);
    CHECK(control_count == n);
    rocos::ServoTimingTrace::UdpRecord first{};
    file.seekg(256 + (11 * sizeof(first)));
    REQUIRE(file.read(reinterpret_cast<char*>(&first), sizeof(first)));
    CHECK(first.message_id == 12);
    CHECK(first.receive_ns < first.locked_ns);
    CHECK(first.locked_ns < first.ready_ns);
    CHECK(first.ready_ns < first.published_ns);
    rocos::ServoTimingTrace::ControlRecord last{};
    file.seekg(256 + sizeof(first) * rocos::ServoTimingTrace::kCapacity + 10 * sizeof(last));
    REQUIRE(file.read(reinterpret_cast<char*>(&last), sizeof(last)));
    CHECK(last.cycle == n);
    CHECK(last.applied_id == n);
    CHECK(last.result == -1008);
    file.close();
    std::filesystem::remove(path);
}
TEST_CASE("Servo timing setup failure disables diagnostics without throwing") {
    TimingEnvironment env;
    std::string error;
    auto trace = rocos::ServoTimingTrace::Create(session("missing/child"), error);
    CHECK_FALSE(trace);
    CHECK_FALSE(error.empty());
}
