#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/frame_registry.hpp"
#include <atomic>
#include <filesystem>
#include <fstream>
#include <thread>
#include <limits>

namespace {
struct Fixture {
    std::filesystem::path directory;
    Fixture() {
        char pattern[] = "/tmp/rocos-frames-XXXXXX";
        auto path = mkdtemp(pattern);
        REQUIRE(path);
        directory = path;
    }
    ~Fixture() { std::filesystem::remove_all(directory); }
};
using rocos::Result;
}

TEST_CASE("TCP uses the selected immutable tool snapshot and rotates the complete tool transform") {
    rocos::FrameRegistry registry;
    const rocos::Frame flange(rocos::Rotation::RotZ(1.5707963267948966), rocos::Vector(1, 2, 3));
    const rocos::Frame tool(rocos::Rotation::RotX(0.4), rocos::Vector(0.01, 0, 0));
    const rocos::Frame other(rocos::Vector(0, 0.03, 0));
    REQUIRE(registry.set("tool1", tool, true) == Result::NoError);
    REQUIRE(registry.set("other", other, true) == Result::NoError);
    rocos::Frame tcp;
    REQUIRE(rocos::FrameRegistry::resolveTcp(*registry.snapshot(), flange, tcp) == Result::NoError);
    CHECK(rocos::Equal(tcp, flange, 1e-12));
    REQUIRE(registry.activate("tool1", true) == Result::NoError);
    const auto selected = registry.snapshot();
    REQUIRE(rocos::FrameRegistry::resolveTcp(*selected, flange, tcp) == Result::NoError);
    CHECK(tcp.p.x() == doctest::Approx(1));
    CHECK(tcp.p.y() == doctest::Approx(2.01));
    CHECK(rocos::Equal(tcp.M, flange.M * tool.M, 1e-12));
    REQUIRE(rocos::FrameRegistry::resolveTcp(*selected, flange, tcp, "other") == Result::NoError);
    CHECK(rocos::Equal(tcp, flange * other, 1e-12));
    CHECK(registry.snapshot()->active_tool == "tool1");
    REQUIRE(registry.remove("tool1", true) == Result::NoError);
    REQUIRE(rocos::FrameRegistry::resolveTcp(*selected, flange, tcp) == Result::NoError);
    CHECK(rocos::Equal(tcp, flange * tool, 1e-12));
    const auto before = tcp;
    CHECK(rocos::FrameRegistry::resolveTcp(*registry.snapshot(), flange, tcp, "missing") == Result::ResourceUnavailable);
    CHECK(rocos::FrameRegistry::resolveTcp(*selected, flange, tcp, "bad/name") == Result::IllegalParameter);
    auto invalid = flange; invalid.p[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK(rocos::FrameRegistry::resolveTcp(*selected, invalid, tcp) == Result::FkCalcFail);
    CHECK(rocos::Equal(tcp, before, 1e-12));
}

TEST_CASE_FIXTURE(Fixture, "Tools persist create edit activate remove and preserve object frames") {
    const auto path = (directory / "frames.yaml").string();
    rocos::FrameRegistry registry(path);
    const rocos::Frame tool(rocos::Rotation::RPY(0.2, -0.3, 0.4), rocos::Vector(0.01, 0.02, 0.15));
    REQUIRE(registry.set("fixture", rocos::Frame(), false) == Result::NoError);
    REQUIRE(registry.set("TCP_1", tool, true, true, true) == Result::NoError);
    CHECK(registry.set("TCP_1", tool, true, true) == Result::CallingConflictError);
    REQUIRE(registry.activate("TCP_1", true, true) == Result::NoError);
    REQUIRE(registry.set("TCP_renamed", tool, true, false, true, "TCP_1") == Result::NoError);
    CHECK(registry.snapshot()->active_tool == "TCP_renamed");
    CHECK(registry.set("TCP_1", tool, true, false, true, "TCP_renamed") == Result::NoError);
    const auto old_snapshot = registry.snapshot();
    rocos::FrameRegistry restarted(path);
    REQUIRE(restarted.load() == Result::NoError);
    CHECK(restarted.snapshot()->active_tool == "TCP_1");
    CHECK(restarted.snapshot()->objects.count("fixture") == 1);
    rocos::Frame read;
    REQUIRE(restarted.get("TCP_1", true, read) == Result::NoError);
    CHECK(rocos::Equal(tool, read, 1e-12));
    REQUIRE(registry.set("TCP_1", rocos::Frame(rocos::Vector(0, 0, 0.3)), true, false, true) == Result::NoError);
    CHECK(rocos::Equal(old_snapshot->tools.at("TCP_1"), tool, 1e-12));
    REQUIRE(registry.remove("TCP_1", true, true) == Result::NoError);
    CHECK(registry.snapshot()->active_tool.empty());
    REQUIRE(restarted.load() == Result::NoError);
    CHECK(restarted.names(true).empty());
    CHECK(restarted.names(false).size() == 1);
}

TEST_CASE_FIXTURE(Fixture, "A failed persistent write does not publish partial tool changes") {
    rocos::FrameRegistry registry((directory / "missing" / "frames.yaml").string());
    REQUIRE(registry.set("TCP", rocos::Frame(), true) == Result::NoError);
    const auto before = registry.snapshot();
    CHECK(registry.set("new", rocos::Frame(), true, false, true) == Result::ResourceUnavailable);
    CHECK(registry.remove("TCP", true, true) == Result::ResourceUnavailable);
    CHECK(registry.activate("TCP", true, true) == Result::ResourceUnavailable);
    CHECK(registry.snapshot() == before);
    CHECK(registry.activate("missing", true) == Result::ResourceUnavailable);
}

TEST_CASE_FIXTURE(Fixture, "Invalid frame files preserve the complete current registry") {
    const auto path = (directory / "frames.yaml").string();
    rocos::FrameRegistry registry(path);
    REQUIRE(registry.set("TCP", rocos::Frame(), true) == Result::NoError);
    REQUIRE(registry.activate("TCP", true) == Result::NoError);
    const auto before = registry.snapshot();
    for (const auto* content : {
        "tools: []", "active_tool: missing\ntools: {}", "tools: {bad/name: {}}",
        "tools: {TCP: {position: [0,0,0], orientation: [0,0,0,0]}}",
        "tools: {TCP: {position: [.nan,0,0], orientation: [0,0,0,1]}}"}) {
        std::ofstream(path) << content;
        CHECK(registry.load() != Result::NoError);
        CHECK(registry.snapshot() == before);
    }
    rocos::Frame malformed;
    malformed.M(0, 0) = 2;
    CHECK(registry.set("bad", malformed, true) == Result::IllegalParameter);
    CHECK(registry.set("bad/name", rocos::Frame(), true) == Result::IllegalParameter);
}

TEST_CASE("Frame snapshots remain coherent while tools are replaced") {
    rocos::FrameRegistry registry;
    REQUIRE(registry.set("TCP", rocos::Frame(), true) == Result::NoError);
    std::atomic<bool> done{false}, coherent{true};
    std::thread reader([&] {
        while (!done) {
            const auto snapshot = registry.snapshot();
            if (!snapshot->active_tool.empty() && !snapshot->tools.count(snapshot->active_tool)) coherent = false;
        }
    });
    for (int i = 0; i < 200; ++i) {
        registry.set("TCP", rocos::Frame(rocos::Vector(0, 0, i * 0.001)), true);
        registry.activate("TCP", true);
        registry.remove("TCP", true);
    }
    done = true;
    reader.join();
    CHECK(coherent);
}
