// MoveJog 规划层往复点动测试 — 隔离漂移到底是规划层还是控制层
//
// 模拟完整往复流程：
//   new MoveJog → FeedJog(+X) ×3s → Stop → PlanFinished
//   → new MoveJog → FeedJog(-X) ×3s → Stop → PlanFinished
//   → 重复 N 循环，每步记录 FK 末端位置
//
// 运行：./build/bin/move_jog_reciprocating_test [cycles] [phase_s]
//       默认 5 循环，3 秒/程

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

#include "src/model.hpp"
#include "src/move_jog.hpp"

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kDt       = 0.001;
constexpr double kTimeout  = 0.20;
constexpr double kJogSpeed = 0.03;
constexpr double kFeedPeriod = 0.05;

struct FkSample {
    double t{0.0};
    double px{0.0}, py{0.0}, pz{0.0};
    double q[7]{0.0};
};

bool isOk(rocos::Result rc) { return rc == rocos::Result::NoError; }

rocos::JntArray makeHomeJoints() {
    rocos::JntArray q(7);
    const double deg[7] = {0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0};
    for (int i = 0; i < 7; ++i) q(i) = deg[i] * kDegToRad;
    return q;
}

bool extractJointRef(const rocos::Reference& ref, rocos::JntArray& q) {
    if (!std::holds_alternative<rocos::JntArray>(ref)) return false;
    q = std::get<rocos::JntArray>(ref);
    return true;
}

std::string findUrdf() {
    for (const auto& c : {"robot.urdf", "config/robot.urdf", "config/robots/talon/robot.urdf"})
        if (std::filesystem::exists(c)) return c;
    return "robot.urdf";
}

/// 模拟一个方向的点动阶段，返回结束时 FK 位置 + 内部 q_integral
struct PhaseResult {
    rocos::JntArray q_final;
    rocos::Frame    fk_final;
    double          travel_mm{0.0};
    int             steps{0};
};

PhaseResult runPhase(rocos::Model& model,
                     const rocos::JntArray& q_start,
                     const rocos::Twist& direction,
                     double duration_s,
                     bool verbose = false) {
    PhaseResult pr;

    // 模拟 Robot::MoveJogging Branch 3：全新创建
    rocos::MoveJog jog(kDt, kTimeout, &model);
    jog.setInitialPosition(q_start);
    jog.Reset();
    jog.FeedJog(direction, kJogSpeed);

    const int total_steps = static_cast<int>(duration_s / kDt);
    const int feed_every = static_cast<int>(kFeedPeriod / kDt);
    rocos::JntArray q_cur = q_start;
    int step = 0;

    while (step < total_steps) {
        // 模拟 HTTP 周期喂入
        if (step > 0 && step % feed_every == 0) {
            auto rc = jog.FeedJog(direction, kJogSpeed);
            // 如果被拒绝（方向不变所以不会），忽略
            (void)rc;
        }

        rocos::Reference ref;
        auto rc = jog.GenerateRef(ref);
        if (rc == rocos::Result::PlanFinished) break;
        extractJointRef(ref, q_cur);
        ++step;
    }

    // 模拟 jog_stop() → 减速到 0
    jog.Stop();

    // 继续推进直到 PlanFinished
    int decel_steps = 0;
    const int max_decel = static_cast<int>(2.0 / kDt);  // 最多 2 秒减速
    while (decel_steps < max_decel) {
        rocos::Reference ref;
        auto rc = jog.GenerateRef(ref);
        if (rc == rocos::Result::PlanFinished) break;
        extractJointRef(ref, q_cur);
        ++decel_steps;
    }

    pr.q_final = q_cur;
    model.ForwardKinematics(q_cur, pr.fk_final);

    // 计算行程
    rocos::Frame fk_start;
    model.ForwardKinematics(q_start, fk_start);
    const double dx = pr.fk_final.p.x() - fk_start.p.x();
    const double dy = pr.fk_final.p.y() - fk_start.p.y();
    const double dz = pr.fk_final.p.z() - fk_start.p.z();
    pr.travel_mm = std::sqrt(dx*dx + dy*dy + dz*dz) * 1000.0;
    pr.steps = step + decel_steps;

    return pr;
}

}  // namespace

int main(int argc, char* argv[]) {
    const int cycles    = (argc > 1) ? std::stoi(argv[1]) : 5;
    const double phase_s = (argc > 2) ? std::stod(argv[2]) : 3.0;

    std::cout << "MoveJog 规划层往复点动测试 — " << cycles << " 循环, ±" << phase_s << "s"
              << std::endl;
    std::cout << "（纯规划层，不涉及 Robot/Executor/Controller/Hardware）" << std::endl;

    rocos::Model model(findUrdf(), "base_link", "link_7");
    const rocos::JntArray q_home = makeHomeJoints();

    rocos::Frame fk_home;
    model.ForwardKinematics(q_home, fk_home);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Home FK: (" << fk_home.p.x() << ", " << fk_home.p.y()
              << ", " << fk_home.p.z() << ")" << std::endl;

    rocos::Twist dir_pos_x, dir_neg_x;
    dir_pos_x.vel.x(1.0);
    dir_neg_x.vel.x(-1.0);

    std::vector<PhaseResult> results;
    rocos::JntArray q_current = q_home;

    for (int c = 0; c < cycles; ++c) {
        // +X phase
        auto r_pos = runPhase(model, q_current, dir_pos_x, phase_s);
        results.push_back(r_pos);
        q_current = r_pos.q_final;

        // -X phase
        auto r_neg = runPhase(model, q_current, dir_neg_x, phase_s);
        results.push_back(r_neg);
        q_current = r_neg.q_final;
    }

    // ── 分析 ──
    std::cout << "\n" << std::string(75, '=') << std::endl;
    std::cout << "  Planner Reciprocating Jog Results" << std::endl;
    std::cout << std::string(75, '=') << std::endl;
    std::cout << std::setw(10) << "Phase" << std::setw(12) << "Travel(mm)"
              << std::setw(12) << "Steps" << std::setw(28) << "End FK"
              << std::endl;
    std::cout << std::string(75, '-') << std::endl;

    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        const int cycle = static_cast<int>(i / 2) + 1;
        const char* dir = (i % 2 == 0) ? "+X" : "-X";
        std::cout << "  C" << cycle << " " << dir << "   "
                  << std::setw(10) << std::setprecision(4) << r.travel_mm
                  << std::setw(10) << r.steps
                  << "  (" << std::setprecision(6)
                  << r.fk_final.p.x() << ", "
                  << r.fk_final.p.y() << ", "
                  << r.fk_final.p.z() << ")"
                  << std::endl;
    }

    // 每个循环结束的漂移
    std::cout << "\n  Cycle-end drift from home:" << std::endl;
    std::cout << std::setw(8) << "Cycle" << std::setw(12) << "dX(mm)"
              << std::setw(12) << "dY(mm)" << std::setw(12) << "dZ(mm)"
              << std::setw(12) << "Drift(mm)" << std::endl;

    double accum_drift = 0.0;
    for (int c = 0; c < cycles; ++c) {
        const auto& r_neg = results[static_cast<size_t>(c * 2 + 1)];  // -X phase end
        const double dx = (r_neg.fk_final.p.x() - fk_home.p.x()) * 1000.0;
        const double dy = (r_neg.fk_final.p.y() - fk_home.p.y()) * 1000.0;
        const double dz = (r_neg.fk_final.p.z() - fk_home.p.z()) * 1000.0;
        const double drift = std::sqrt(dx*dx + dy*dy + dz*dz);
        std::cout << std::setw(6) << (c+1)
                  << std::setw(12) << std::setprecision(4) << dx
                  << std::setw(12) << dy
                  << std::setw(12) << dz
                  << std::setw(12) << drift << std::endl;
        accum_drift = drift;
    }

    // ── 判断 ──
    std::cout << "\n  规划层最终漂移: " << accum_drift << " mm" << std::endl;

    // 对称性检查
    double asym_total = 0.0;
    for (int c = 0; c < cycles; ++c) {
        const double pos_travel = results[static_cast<size_t>(c * 2)].travel_mm;
        const double neg_travel = results[static_cast<size_t>(c * 2 + 1)].travel_mm;
        const double asym = pos_travel - neg_travel;
        asym_total += asym;
        std::cout << "  C" << (c+1) << " 非对称: " << pos_travel << " - "
                  << neg_travel << " = " << asym << " mm" << std::endl;
    }

    std::cout << "\n  总非对称: " << asym_total << " mm" << std::endl;

    if (std::abs(accum_drift) < 0.1) {
        std::cout << "\n  ✅ 规划层漂移 < 0.1mm → 问题在控制层 (Controller/Executor)" << std::endl;
    } else {
        std::cout << "\n  ❌ 规划层漂移 ≥ 0.1mm → 问题在 MoveJog 规划层" << std::endl;
    }

    return 0;
}
