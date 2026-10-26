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

#include "src/model.hpp"
#include "src/move_line_offline.hpp"

namespace {

struct CartesianPoint {
    int    step;
    double time;
    double px, py, pz;      // FK 后的笛卡尔位置
    double qx, qy, qz, qw;  // FK 后的四元数
    double dev_mm;           // 到直线的垂直距离 [mm]
};

constexpr double kDt     = 0.001;
constexpr double kVLimit = 1.0;
constexpr double kALimit = 2.0;
constexpr double kJLimit = 10.0;

constexpr int kMaxSteps    = 10000;
constexpr int kPauseAtStep = 150;

const std::string kCsvDir   = "/tmp/move_line_offline_csv/";
const std::string kUrdfPath = "config/robots/talon/robot.urdf";
const std::string kBaseLink = "base_link";
const std::string kTipLink  = "link_7";

// ─── 辅助 ───

/// 点到直线 (p_start → p_goal) 的垂直距离 [m]
double pointToLineDistance(const KDL::Vector& p,
                           const KDL::Vector& p_start,
                           const KDL::Vector& p_goal) {
    KDL::Vector dir = p_goal - p_start;
    double len2 = KDL::dot(dir, dir);
    if (len2 < 1e-12) return (p - p_start).Norm();  // 退化：起点=终点
    double t = KDL::dot(p - p_start, dir) / len2;
    KDL::Vector proj = p_start + dir * t;
    return (p - proj).Norm();
}

/// 从已有 JntArray 通过 FK 提取笛卡尔位姿
CartesianPoint extractCartesian(int step,
                                double time,
                                const rocos::JntArray& q,
                                rocos::Model& model,
                                const KDL::Vector& p_start,
                                const KDL::Vector& p_goal) {

    KDL::Frame fk;
    model.ForwardKinematics(q, fk);

    CartesianPoint cp{};
    cp.step = step;
    cp.time = time;
    cp.px = fk.p.x();
    cp.py = fk.p.y();
    cp.pz = fk.p.z();
    fk.M.GetQuaternion(cp.qx, cp.qy, cp.qz, cp.qw);

    cp.dev_mm = pointToLineDistance(fk.p, p_start, p_goal) * 1000.0;

    return cp;
}

void writeCsv(const std::string& filename,
              const std::vector<CartesianPoint>& data) {
    std::string full_path = kCsvDir + filename;
    std::ofstream ofs(full_path);
    ofs << "step,time,px,py,pz,qx,qy,qz,qw,dev_mm\n";
    for (const auto& cp : data) {
        ofs << cp.step << "," << cp.time << ","
            << cp.px << "," << cp.py << "," << cp.pz << ","
            << cp.qx << "," << cp.qy << "," << cp.qz << "," << cp.qw << ","
            << cp.dev_mm << "\n";
    }
}

void runNormal(rocos::MoveLineOffline& move,
               rocos::Model& model,
               const KDL::Vector& p_start,
               const KDL::Vector& p_goal,
               std::vector<CartesianPoint>& data,
               const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

/// 运行暂停→继续模式并收集数据
void runPauseResume(rocos::MoveLineOffline& move,
                    rocos::Model& model,
                    const KDL::Vector& p_start,
                    const KDL::Vector& p_goal,
                    std::vector<CartesianPoint>& data,
                    const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    // 阶段 1：运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }

    // 阶段 2：Pause → Jacobian 积分减速到停
    move.Pause();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r == rocos::Result::PlanFinished) break;
        ++step;
        time += kDt;
    }

    // 阶段 3：Resume → Jacobian 积分继续到终点
    move.Resume();
    ++step;
    time += kDt;
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }
    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

/// 运行暂停→停止模式并收集数据
void runPauseStop(rocos::MoveLineOffline& move,
                  rocos::Model& model,
                  const KDL::Vector& p_start,
                  const KDL::Vector& p_goal,
                  std::vector<CartesianPoint>& data,
                  const std::string& csv_name) {
    data.clear();
    int step = 0;
    double time = 0.0;

    // 阶段 1：运行到暂停点
    while (step < kPauseAtStep && step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r != rocos::Result::NoError) break;
        ++step;
        time += kDt;
    }

    // 阶段 2：Pause → Jacobian 积分减速到停
    move.Pause();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r == rocos::Result::PlanFinished) break;
        ++step;
        time += kDt;
    }

    // 阶段 3：Stop 确认
    move.Stop();
    while (step < kMaxSteps) {
        rocos::Reference ref;
        rocos::Result r = move.GenerateRef(ref);
        const auto& q = std::get<rocos::JntArray>(ref);
        data.push_back(extractCartesian(step, time, q, model, p_start, p_goal));
        if (r == rocos::Result::PlanFinished) break;
        ++step;
        time += kDt;
    }

    writeCsv(csv_name, data);
    MESSAGE(csv_name, " — ", data.size(), " data points");
}

}  // namespace

// ==========================================================================
// 测试夹具：加载 talon URDF 模型，从零位 FK 得到起始和目标任务
// ==========================================================================

struct TestFixture {
    rocos::Model model{kUrdfPath, kBaseLink, kTipLink};
    KDL::JntArray q_start{7};  // 初始关节角 [0, 60°, 0, 90°, 0, -60°, 0]

    KDL::Frame frame_start;
    KDL::Frame frame_goal;

    TestFixture() {
        // 从非零关节构型出发，保证 IK 远离奇异区
        q_start(0) = 0.0;
        q_start(1) = M_PI / 3.0;    // 60°
        q_start(2) = 0.0;
        q_start(3) = M_PI / 2.0;    // 90°
        q_start(4) = 0.0;
        q_start(5) = -M_PI / 3.0;   // -60°
        q_start(6) = 0.0;

        // FK → 起始帧
        REQUIRE(static_cast<int>(model.ForwardKinematics(q_start, frame_start)) == 0);

        // 目标帧：小偏移平移 + 小角度旋转（确保在 talon 工作空间内）
        frame_goal = frame_start;
        frame_goal.p += KDL::Vector(0.10, 0.05, 0.05);
        frame_goal.M = frame_start.M * KDL::Rotation::RotZ(M_PI / 6.0);
    }

    rocos::MoveLineOffline createMove() {
        return rocos::MoveLineOffline(frame_start, frame_goal,
                                      &model,
                                      kVLimit, kALimit, kJLimit, kDt);
    }
};

// ==========================================================================
// 正常执行：验证离开起点、到达终点、全轨迹在直线上
// ==========================================================================

TEST_CASE_FIXTURE(TestFixture, "MoveLineOffline 正常执行 — 全轨迹在直线上") {
    auto move = createMove();
    move.SetInitialJointPosition(q_start);

    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<CartesianPoint> data;
    runNormal(move, model, frame_start.p, frame_goal.p, data, "offline_normal.csv");

    // 验证终点到达
    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(frame_goal.p.x()).epsilon(1e-3));
    CHECK(last.py == doctest::Approx(frame_goal.p.y()).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx(frame_goal.p.z()).epsilon(1e-3));

    // 验证起点
    const auto& first = data.front();
    CHECK(first.px == doctest::Approx(frame_start.p.x()).epsilon(1e-3));
    CHECK(first.py == doctest::Approx(frame_start.p.y()).epsilon(1e-3));
    CHECK(first.pz == doctest::Approx(frame_start.p.z()).epsilon(1e-3));

    // 验证全轨迹在直线上：最大垂直偏差 < 0.5mm
    // （normal 模式下是预存关节轨迹，FK 误差仅来自 IK 求解残差）
    double max_dev = 0.0;
    for (const auto& cp : data) {
        if (cp.dev_mm > max_dev) max_dev = cp.dev_mm;
    }
    CHECK(max_dev < 0.5);
    MESSAGE("Normal: max line deviation = ", max_dev, " mm, ",
            data.size(), " total points");
}

// ==========================================================================
// 暂停继续：验证暂停减速、Resume 加速全在直线上，最后到达终点
// ==========================================================================

TEST_CASE_FIXTURE(TestFixture, "MoveLineOffline 暂停继续 — 暂停/恢复全程在直线上") {
    auto move = createMove();
    move.SetInitialJointPosition(q_start);

    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<CartesianPoint> data;
    runPauseResume(move, model, frame_start.p, frame_goal.p, data, "offline_pause_resume.csv");

    // 验证终点到达
    const auto& last = data.back();
    CHECK(last.px == doctest::Approx(frame_goal.p.x()).epsilon(1e-3));
    CHECK(last.py == doctest::Approx(frame_goal.p.y()).epsilon(1e-3));
    CHECK(last.pz == doctest::Approx(frame_goal.p.z()).epsilon(1e-3));

    // 验证全轨迹在直线上：即使暂停/恢复走 Jacobian 积分通道，
    // 所有FK位置仍应严格在起点到终点的直线上
    double max_dev = 0.0;
    for (const auto& cp : data) {
        if (cp.dev_mm > max_dev) max_dev = cp.dev_mm;
    }
    CHECK(max_dev < 0.5);
    MESSAGE("Pause+Resume: max line deviation = ", max_dev, " mm, ",
            data.size(), " total points");
}

// ==========================================================================
// 暂停停止：验证暂停后停止，未到终点，全程在直线上
// ==========================================================================

TEST_CASE_FIXTURE(TestFixture, "MoveLineOffline 暂停停止 — 停止后未到终点但仍在直线上") {
    auto move = createMove();
    move.SetInitialJointPosition(q_start);

    REQUIRE(move.Reset() == rocos::Result::NoError);

    std::vector<CartesianPoint> data;
    runPauseStop(move, model, frame_start.p, frame_goal.p, data, "offline_pause_stop.csv");

    // 验证未到终点
    const auto& last = data.back();
    KDL::Vector final_pos(last.px, last.py, last.pz);
    double dist_start = (final_pos - frame_start.p).Norm();
    double dist_goal  = (final_pos - frame_goal.p).Norm();
    CHECK(dist_goal > 0.01);  // 离终点至少 1cm
    CHECK(dist_start > 0.01); // 已离开起点至少 1cm（运行了 kPauseAtStep 步）

    // 验证全轨迹在直线上
    double max_dev = 0.0;
    for (const auto& cp : data) {
        if (cp.dev_mm > max_dev) max_dev = cp.dev_mm;
    }
    CHECK(max_dev < 0.5);
    MESSAGE("Pause+Stop: max line deviation = ", max_dev, " mm, ",
            data.size(), " total points");
}

// ==========================================================================
// 边界情况：无运动（起点=终点）
// ==========================================================================

TEST_CASE_FIXTURE(TestFixture, "MoveLineOffline 起点等于终点 — 直接返回 PlanFinished") {
    // 目标 = 起点
    auto move = rocos::MoveLineOffline(frame_start, frame_start,
                                       &model,
                                       kVLimit, kALimit, kJLimit, kDt);
    move.SetInitialJointPosition(q_start);
    CHECK(move.Reset() == rocos::Result::PlanFinished);
}

// ==========================================================================
// 边界情况：无初始关节位置 → 参数校验失败
// ==========================================================================

TEST_CASE_FIXTURE(TestFixture, "MoveLineOffline 无初始关节位置 — 参数校验失败") {
    auto move = createMove();
    // 不调用 SetInitialJointPosition
    CHECK(move.Reset() != rocos::Result::NoError);
}
