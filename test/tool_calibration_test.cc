#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/tool_calibration.hpp"
#include <limits>

namespace {
std::array<rocos::Frame, 6> fixture(const rocos::Frame& tool) {
    const rocos::Vector pivot(0.5, -0.2, 0.4);
    std::array<rocos::Frame, 6> points;
    points[0].M = rocos::Rotation::RPY(0.6, -0.3, 0.2);
    points[1].M = rocos::Rotation::RPY(-0.4, 0.7, -0.5);
    points[2].M = rocos::Rotation::RPY(0.3, 0.4, 0.8);
    points[3].M = rocos::Rotation::RPY(-0.2, -0.3, 0.4);
    for (int i = 0; i < 4; ++i) points[i].p = pivot - points[i].M * tool.p;
    points[4] = points[3]; points[5] = points[3];
    points[4].p = points[4].p + points[3].M * tool.M * rocos::Vector(0.07, 0, 0);
    points[5].p = points[5].p + points[3].M * tool.M * rocos::Vector(0, 0, 0.08);
    return points;
}
}

TEST_CASE("Six point calibration recovers a nontrivial TCP and right handed tool rotation") {
    const rocos::Frame expected(rocos::Rotation::RPY(0.4, -0.5, 1.2), rocos::Vector(0.03, -0.02, 0.18));
    rocos::ToolCalibrationSolution result;
    std::string error;
    REQUIRE(rocos::solveSixPointTool(fixture(expected), 0.001, result, error));
    CHECK(rocos::Equal(result.frame, expected, 1e-9));
    CHECK(result.rms_error < 1e-12);
    CHECK(result.max_error < 1e-12);
    CHECK(result.condition_number >= 1);
    CHECK(result.axis_angle_degrees == doctest::Approx(90));
}

TEST_CASE("Six point calibration measures pivot residual and rejects excessive error") {
    auto points = fixture(rocos::Frame(rocos::Vector(0, 0, 0.2)));
    points[0].p = points[0].p + rocos::Vector(0.0001, 0, 0);
    rocos::ToolCalibrationSolution result;
    std::string error;
    REQUIRE(rocos::solveSixPointTool(points, 0.001, result, error));
    CHECK(result.max_error > 0);
    CHECK_FALSE(rocos::solveSixPointTool(points, 1e-8, result, error));
    CHECK(error.find("Pivot error") != std::string::npos);
}

TEST_CASE("Six point calibration rejects repeated poses bad directions and changed orientation") {
    const auto valid = fixture(rocos::Frame(rocos::Vector(0, 0, 0.2)));
    rocos::ToolCalibrationSolution result;
    std::string error;
    SUBCASE("identical orientations") {
        auto points = valid;
        for (int i = 0; i < 4; ++i) points[i] = points[3];
        CHECK_FALSE(rocos::solveSixPointTool(points, 0.001, result, error));
        CHECK(error.find("orientations") != std::string::npos);
    }
    SUBCASE("collinear axes") {
        auto points = valid; points[5] = points[4];
        CHECK_FALSE(rocos::solveSixPointTool(points, 0.001, result, error));
        CHECK(error.find("collinear") != std::string::npos);
    }
    SUBCASE("short axis movement") {
        auto points = valid; points[4] = points[3];
        CHECK_FALSE(rocos::solveSixPointTool(points, 0.001, result, error));
        CHECK(error.find("1 mm") != std::string::npos);
    }
    SUBCASE("orientation changes after point four") {
        auto points = valid; points[4].M = points[3].M * rocos::Rotation::RotX(0.1);
        CHECK_FALSE(rocos::solveSixPointTool(points, 0.001, result, error));
        CHECK(error.find("orientation") != std::string::npos);
    }
    SUBCASE("nonfinite pose") {
        auto points = valid; points[0].p[0] = std::numeric_limits<double>::quiet_NaN();
        CHECK_FALSE(rocos::solveSixPointTool(points, 0.001, result, error));
    }
    SUBCASE("invalid tolerance") {
        CHECK_FALSE(rocos::solveSixPointTool(valid, 0, result, error));
        CHECK_FALSE(rocos::solveSixPointTool(valid, std::numeric_limits<double>::infinity(), result, error));
    }
}
