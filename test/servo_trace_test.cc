#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "src/servo_trace.hpp"

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <json.hpp>
#include <string>

#include "src/model.hpp"
#include "src/servo_signal_processor.hpp"
#include "test/doctest.h"

namespace {
struct TraceDirectory {
    std::filesystem::path path;
    std::string previous;
    bool had_previous;
    TraceDirectory() {
        const char* value = std::getenv("ROCOS_SERVO_TRACE_DIR");
        had_previous = value != nullptr;
        if (value) previous = value;
        path = std::filesystem::temp_directory_path() /
               ("rocos_trace_test_" + std::to_string(getpid()) + "_" +
                std::to_string(rocos::ServoTraceNow()));
        std::filesystem::create_directory(path);
        setenv("ROCOS_SERVO_TRACE_DIR", path.c_str(), 1);
    }
    ~TraceDirectory() {
        if (had_previous)
            setenv("ROCOS_SERVO_TRACE_DIR", previous.c_str(), 1);
        else
            unsetenv("ROCOS_SERVO_TRACE_DIR");
        std::filesystem::remove_all(path);
    }
};
}  // namespace
TEST_CASE("Servo trace scope restores the caller context") {
    rocos::ServoIkTrace a, b;
    CHECK(rocos::CurrentServoIkTrace() == nullptr);
    {
        rocos::ServoIkTraceScope outer(&a);
        CHECK(rocos::CurrentServoIkTrace() == &a);
        {
            rocos::ServoIkTraceScope inner(&b);
            CHECK(rocos::CurrentServoIkTrace() == &b);
        }
        CHECK(rocos::CurrentServoIkTrace() == &a);
    }
    CHECK(rocos::CurrentServoIkTrace() == nullptr);
}
TEST_CASE("Servo trace flushes complete records in order at shutdown") {
    TraceDirectory directory;
    auto trace = rocos::ServoTrace::Create();
    REQUIRE(trace);
    for (int i = 0; i < 200; ++i) {
        rocos::ServoTraceRecord record;
        record.sequence = i + 1;
        record.ik[1].count = 2;
        record.ik[1].selected = 1;
        record.ik[1].candidates[1][6] = i * .25;
        record.next[6] = i * .125;
        trace->Push(record);
    }
    trace.reset();
    for (const auto& entry :
         std::filesystem::directory_iterator(directory.path)) {
        if (entry.path().extension() == ".json") {
            std::ifstream file(entry.path());
            nlohmann::json meta;
            file >> meta;
            CHECK(meta["records"] == 200);
            CHECK(meta["dropped"] == 0);
            CHECK(meta["io_error"] == false);
            CHECK(meta["record_bytes"] == sizeof(rocos::ServoTraceRecord));
        } else {
            REQUIRE(std::filesystem::file_size(entry.path()) ==
                    200 * sizeof(rocos::ServoTraceRecord));
            std::ifstream file(entry.path(), std::ios::binary);
            for (int i = 0; i < 200; ++i) {
                rocos::ServoTraceRecord record;
                file.read(reinterpret_cast<char*>(&record), sizeof(record));
                REQUIRE(file.good());
                CHECK(record.sequence == i + 1);
                CHECK(record.ik[1].selected == 1);
                CHECK(record.ik[1].candidates[1][6] == i * .25);
                CHECK(record.next[6] == i * .125);
            }
        }
    }
}
TEST_CASE("Servo trace is disabled unless explicitly configured") {
    TraceDirectory directory;
    unsetenv("ROCOS_SERVO_TRACE_DIR");
    CHECK(rocos::ServoTrace::Create() == nullptr);
    setenv("ROCOS_SERVO_TRACE_DIR", "/nonexistent/rocos-trace", 1);
    CHECK(rocos::ServoTrace::Create() == nullptr);
}

TEST_CASE(
    "Servo trace records only the filtered Cartesian IK and final output") {
    TraceDirectory directory;
    rocos::Model model(TRACE_TEST_MODEL, "base_link", "link_7");
    rocos::JntArray q(7), output;
    rocos::Frame target;
    REQUIRE(model.ForwardKinematics(q, target) == rocos::Result::NoError);
    {
        rocos::ServoSignalProcessor processor;
        REQUIRE(processor.Reset(&model,
                                std::vector<rocos::JointDerivativeLimits>(
                                    7, {10, 150, 10, 0}),
                                q, .001, {}) == rocos::Result::NoError);
        REQUIRE(processor.Step(target, false, output) ==
                rocos::Result::NoError);
    }
    int records = 0;
    for (const auto& entry :
         std::filesystem::directory_iterator(directory.path)) {
        if (entry.path().extension() != ".bin") continue;
        std::ifstream file(entry.path(), std::ios::binary);
        rocos::ServoTraceRecord record;
        file.read(reinterpret_cast<char*>(&record), sizeof(record));
        REQUIRE(file.good());
        ++records;
        CHECK(record.result == 0);
        CHECK(record.ik[0].result == std::numeric_limits<int32_t>::min());
        CHECK(record.ik[0].count == 0);
        CHECK(record.ik[0].solve_ns == 0);
        {
            const auto& ik = record.ik[1];
            CHECK(ik.result == 0);
            REQUIRE(ik.count >= 1);
            REQUIRE(ik.count <= 2);
            REQUIRE(ik.selected >= 0);
            REQUIRE(ik.selected < ik.count);
            for (int i = 0; i < 7; ++i) {
                CHECK(ik.candidates[ik.selected][i] == ik.output[i]);
                CHECK(record.next[i] == output(i));
            }
        }
    }
    CHECK(records == 1);
}
