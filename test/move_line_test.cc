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

#include "src/move_line.hpp"

namespace {

struct DataPoint {
    int    step;
    double time;
    double px, py, pz;      // 位置
    double qx, qy, qz, qw;  // 四元数
};

constexpr double kDt     = 0.001;
constexpr double kVLimit = 1.0;
constexpr double kALimit = 2.0;
constexpr double kJLimit = 10.0;

constexpr int kMaxSteps    = 10000;
constexpr int kPauseAtStep = 200;

const std::string kCsvDir = "/tmp/move_line_csv/";

// ─── 辅助 ───

/// 构造位姿：位置 (x,y,z) + 绕 Z 轴旋转 angle 弧度
rocos::Frame makeFrame(double x, double y, double z, double angle_z) {
    return rocos::Frame(rocos::Rotation::RotZ(angle_z), rocos::Vector(x, y, z));
}

/// 从已有 Reference 提取位姿数据
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

static void runNormal(rocos::MoveLine& move,
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
        ++step;
        time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

static void runPauseResume(rocos::MoveLine& move,
                           std::vector<DataPoint>& data,
                           const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    // 阶段 1：运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }

    // 阶段 2：Pause → 减速到停
    move.Pause();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r == rocos::Result::PlanFinished) break;
        ++step;
        time += kDt;
    }

    // 阶段 3：Resume → 继续到目标
    move.Resume();
    ++step;
    time += kDt;
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

static void runPauseStop(rocos::MoveLine& move,
                         std::vector<DataPoint>& data,
                         const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    // 阶段 1：运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }

    // 阶段 2：Pause → 减速到停
    move.Pause();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r == rocos::Result::PlanFinished) break;
        ++step;
        time += kDt;
    }

    // 阶段 3：Stop 确认
    move.Stop();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        data.push_back(extractData(step, time, ref));
        if (r == rocos::Result::PlanFinished) break;
        ++step;
        time += kDt;
    }

    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

// ==========================================================================
// 正向：起点 (0,0,0) 无旋转 → 终点 (0.4,0.3,0.5) + 绕Z旋转60°
// translation ≈ 0.707, rotation = π/3 ≈ 1.047
// r_length = 0.1*1.047 = 0.105, path_length = max(0.707, 0.105) = 0.707
// ==========================================================================

TEST_CASE("MoveLine 正向 — 正常执行") {
    auto start = makeFrame(0, 0, 0, 0);
    auto goal  = makeFrame(0.4, 0.3, 0.5, M_PI / 3.0);
    rocos::MoveLine move(start, goal, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runNormal(move, data, "forward_normal.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(0.4).epsilon(1e-3));
    CHECK(last.py == doctest::Approx(0.3).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx(0.5).epsilon(1e-3));
}

TEST_CASE("MoveLine 正向 — 暂停继续") {
    auto start = makeFrame(0, 0, 0, 0);
    auto goal  = makeFrame(0.4, 0.3, 0.5, M_PI / 3.0);
    rocos::MoveLine move(start, goal, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseResume(move, data, "forward_pause_resume.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(0.4).epsilon(1e-3));
    CHECK(last.py == doctest::Approx(0.3).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx(0.5).epsilon(1e-3));
}

TEST_CASE("MoveLine 正向 — 暂停后Stop") {
    auto start = makeFrame(0, 0, 0, 0);
    auto goal  = makeFrame(0.4, 0.3, 0.5, M_PI / 3.0);
    rocos::MoveLine move(start, goal, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseStop(move, data, "forward_pause_stop.csv");

    const auto& last = data.back();
    CHECK(last.px < 0.4);
}

// ==========================================================================
// 反向：起点 (0.4,0.3,0.5) + 绕Z 60° → 终点 (0,0,0) 无旋转
// path_length 相同 = 0.707，数据点数应与正向一致
// ==========================================================================

TEST_CASE("MoveLine 反向 — 正常执行") {
    auto start = makeFrame(0.4, 0.3, 0.5, M_PI / 3.0);
    auto goal  = makeFrame(0, 0, 0, 0);
    rocos::MoveLine move(start, goal, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runNormal(move, data, "reverse_normal.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(last.py == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx(0.0).epsilon(1e-3));
}

TEST_CASE("MoveLine 反向 — 暂停继续") {
    auto start = makeFrame(0.4, 0.3, 0.5, M_PI / 3.0);
    auto goal  = makeFrame(0, 0, 0, 0);
    rocos::MoveLine move(start, goal, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseResume(move, data, "reverse_pause_resume.csv");

    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(last.py == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx(0.0).epsilon(1e-3));
}

TEST_CASE("MoveLine 反向 — 暂停后Stop") {
    auto start = makeFrame(0.4, 0.3, 0.5, M_PI / 3.0);
    auto goal  = makeFrame(0, 0, 0, 0);
    rocos::MoveLine move(start, goal, kVLimit, kALimit, kJLimit, kDt);
    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<DataPoint> data;
    runPauseStop(move, data, "reverse_pause_stop.csv");

    const auto& last = data.back();
    CHECK(last.px > 0.0);
}
