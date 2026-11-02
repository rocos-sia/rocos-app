// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <fstream>
#include <string>
#include <variant>
#include <vector>

#include <test/doctest.h>

#include "src/move_circle.hpp"

namespace {

struct DataPoint {
    int    step;
    double time;
    double px, py, pz;
    double qx, qy, qz, qw;
};

constexpr double kDt     = 0.001;
constexpr double kVLimit = 1.0;
constexpr double kALimit = 2.0;
constexpr double kJLimit = 10.0;

constexpr int kMaxSteps    = 10000;
constexpr int kPauseAtStep = 200;

const std::string kCsvDir = "/tmp/move_circle_csv/";

// ─── 辅助 ───

/// 圆心在 origin，法向 Z 轴（XY 平面圆弧）
rocos::Frame makeCenterFrame(double cx, double cy, double cz) {
    return rocos::Frame(rocos::Rotation::Identity(), rocos::Vector(cx, cy, cz));
}

/// 起始位姿：位置 (x,y,z)，单位姿态
rocos::Frame makeStartPose(double x, double y, double z) {
    return rocos::Frame(rocos::Rotation::Identity(), rocos::Vector(x, y, z));
}

DataPoint extractData(int step, double time, const rocos::Reference& ref) {
    const auto& f = std::get<rocos::Frame>(ref);

    DataPoint dp{};
    dp.step = step;
    dp.time = time;
    dp.px = f.p.x();  dp.py = f.p.y();  dp.pz = f.p.z();
    f.M.GetQuaternion(dp.qx, dp.qy, dp.qz, dp.qw);
    return dp;
}

void writeCsv(const std::string& filename, const std::vector<DataPoint>& data) {
    std::string full_path = kCsvDir + filename;
    std::ofstream ofs(full_path);
    ofs << "step,time,px,py,pz,qx,qy,qz,qw\n";
    for (const auto& dp : data) {
        ofs << dp.step << "," << dp.time << ","
            << dp.px << "," << dp.py << "," << dp.pz << ","
            << dp.qx << "," << dp.qy << "," << dp.qz << "," << dp.qw << "\n";
    }
}

}  // namespace

// ==========================================================================
// 测试辅助函数
// ==========================================================================

static void runNormal(rocos::MoveCircle& move,
                      std::vector<DataPoint>& data,
                      const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step; time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

static void runPauseResume(rocos::MoveCircle& move,
                           std::vector<DataPoint>& data,
                           const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    while (step < kPauseAtStep && step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step; time += kDt;
    }

    move.Pause();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r == rocos::Result::PlanFinished) break;
        ++step; time += kDt;
    }

    move.Resume();
    ++step; time += kDt;
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step; time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

static void runPauseStop(rocos::MoveCircle& move,
                         std::vector<DataPoint>& data,
                         const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    while (step < kPauseAtStep && step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step; time += kDt;
    }

    move.Pause();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r == rocos::Result::PlanFinished) break;
        ++step; time += kDt;
    }

    move.Stop();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r == rocos::Result::PlanFinished) break;
        ++step; time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

// ==========================================================================
// 正向：圆心 (0,0,0), R=0.5, theta=π (180° 半圆弧) XY 平面
// 起点 (0.5, 0, 0) → 逆时针半圆 → 终点 (-0.5, 0, 0)
// path_length = 0.5 × π ≈ 1.571
// ==========================================================================

TEST_CASE("MoveCircle 正向 — 正常执行") {
    auto start  = makeStartPose(0.5, 0, 0);
    auto center = makeCenterFrame(0, 0, 0);
    rocos::MoveCircle move(start, center, M_PI, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runNormal(move, data, "forward_normal.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(-0.5).epsilon(1e-3));
    CHECK(last.py == doctest::Approx( 0.0).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx( 0.0).epsilon(1e-3));
}

TEST_CASE("MoveCircle 正向 — 暂停继续") {
    auto start  = makeStartPose(0.5, 0, 0);
    auto center = makeCenterFrame(0, 0, 0);
    rocos::MoveCircle move(start, center, M_PI, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseResume(move, data, "forward_pause_resume.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(-0.5).epsilon(1e-3));
}

TEST_CASE("MoveCircle 正向 — 暂停后Stop") {
    auto start  = makeStartPose(0.5, 0, 0);
    auto center = makeCenterFrame(0, 0, 0);
    rocos::MoveCircle move(start, center, M_PI, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseStop(move, data, "forward_pause_stop.csv");

    const auto& last = data.back();
    CHECK(last.px > 0.45);  // 暂停点早，应仍在起点附近（终点在 -0.5）
}

// ==========================================================================
// 反向：起点 (-0.5, 0, 0), 圆心 (0,0,0), theta = -π (顺时针半圆)
// → 终点回到 (0.5, 0, 0)，path_length 同正向
// ==========================================================================

TEST_CASE("MoveCircle 反向 — 正常执行") {
    auto start  = makeStartPose(-0.5, 0, 0);
    auto center = makeCenterFrame(0, 0, 0);
    rocos::MoveCircle move(start, center, -M_PI, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runNormal(move, data, "reverse_normal.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx( 0.5).epsilon(1e-3));
    CHECK(last.py == doctest::Approx( 0.0).epsilon(1e-3));
}

TEST_CASE("MoveCircle 反向 — 暂停继续") {
    auto start  = makeStartPose(-0.5, 0, 0);
    auto center = makeCenterFrame(0, 0, 0);
    rocos::MoveCircle move(start, center, -M_PI, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseResume(move, data, "reverse_pause_resume.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(0.5).epsilon(1e-3));
}

TEST_CASE("MoveCircle 反向 — 暂停后Stop") {
    auto start  = makeStartPose(-0.5, 0, 0);
    auto center = makeCenterFrame(0, 0, 0);
    rocos::MoveCircle move(start, center, -M_PI, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseStop(move, data, "reverse_pause_stop.csv");

    const auto& last = data.back();
    CHECK(last.px < -0.45);  // 暂停点早，应仍在起点附近（终点在 0.5）
}
