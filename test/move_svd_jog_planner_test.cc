// SVD 零空间点动离线规划层测试
//
// 本测试不创建 Robot，不启动控制线程，不连接硬件/仿真。
// 仅运行规划层：
//   MoveSvdJog::FeedSvdJog → MoveSvdJog::Update → MoveSvdJog::GenerateRef → Model::FK
// 验证 SVD 零空间点动过程中末端位姿保持不变。
//
// 运行方式（项目根目录）：
//   ./build/bin/move_svd_jog_planner_test

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "src/model.hpp"
#include "src/move_svd_jog.hpp"

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kDt       = 0.001;      // 规划步长
constexpr double kDuration = 2.0;        // 测试时长
constexpr double kTimeout  = 0.20;       // 喂入超时
constexpr double kSvdSpeed = 0.03;       // 零空间维度速度 (rad/s)
constexpr int    kSampleEvery = 50;      // 每隔多少步记录一次采样

// ---- 类型 ----

struct Pose {
    double px{0.0}, py{0.0}, pz{0.0};      // 位置
    double m00{0.0}, m01{0.0}, m02{0.0};    // 旋转矩阵（按列优先）
    double m10{0.0}, m11{0.0}, m12{0.0};
    double m20{0.0}, m21{0.0}, m22{0.0};
};

struct TestSample {
    double t{0.0};
    Pose   pose;
    double q_positions[7]{0.0};
};

// ---- 工具函数 ----

bool isOk(rocos::Result rc) {
    return rc == rocos::Result::NoError;
}

rocos::JntArray makeHomeJoints() {
    rocos::JntArray q(7);
    const double deg[7] = {0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0};
    for (int i = 0; i < 7; ++i) {
        q(i) = deg[i] * kDegToRad;
    }
    return q;
}

Pose frameToPose(const rocos::Frame& frame) {
    Pose p;
    p.px  = frame.p.x();
    p.py  = frame.p.y();
    p.pz  = frame.p.z();
    p.m00 = frame.M(0, 0); p.m01 = frame.M(0, 1); p.m02 = frame.M(0, 2);
    p.m10 = frame.M(1, 0); p.m11 = frame.M(1, 1); p.m12 = frame.M(1, 2);
    p.m20 = frame.M(2, 0); p.m21 = frame.M(2, 1); p.m22 = frame.M(2, 2);
    return p;
}

Pose computeFk(rocos::Model& model, const rocos::JntArray& q) {
    rocos::Frame frame;
    if (!isOk(model.ForwardKinematics(q, frame))) {
        std::cerr << "FK failed" << std::endl;
        return {};
    }
    return frameToPose(frame);
}

// ---- 位姿比较 ----

/// 位置位移 (欧氏距离)
double positionDelta(const Pose& a, const Pose& b) {
    const double dx = b.px - a.px;
    const double dy = b.py - a.py;
    const double dz = b.pz - a.pz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// 旋转矩阵偏差 (Frobenius 范数除以 3 取平均)
double rotationDelta(const Pose& a, const Pose& b) {
    const double dm00 = b.m00 - a.m00, dm01 = b.m01 - a.m01, dm02 = b.m02 - a.m02;
    const double dm10 = b.m10 - a.m10, dm11 = b.m11 - a.m11, dm12 = b.m12 - a.m12;
    const double dm20 = b.m20 - a.m20, dm21 = b.m21 - a.m21, dm22 = b.m22 - a.m22;

    const double frob_sq = dm00 * dm00 + dm01 * dm01 + dm02 * dm02 +
                           dm10 * dm10 + dm11 * dm11 + dm12 * dm12 +
                           dm20 * dm20 + dm21 * dm21 + dm22 * dm22;
    return std::sqrt(frob_sq) / 3.0;
}

// ---- URDF 查找 ----

std::string findUrdfPath() {
    const std::vector<std::string> candidates = {
        "robot.urdf",
        "config/robot.urdf",
        "config/robots/talon/robot.urdf",
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return "robot.urdf";
}

// ---- 从 Reference 中提取 JntArray ----

bool extractJointRef(const rocos::Reference& ref, rocos::JntArray& q) {
    if (!std::holds_alternative<rocos::JntArray>(ref)) {
        return false;
    }
    q = std::get<rocos::JntArray>(ref);
    return true;
}

// ---- 主测试流程 ----

int runTest() {
    const std::string urdf_path = findUrdfPath();
    std::cout << "URDF: " << urdf_path << std::endl;

    // ── 1. 创建 Model ──
    rocos::Model model(urdf_path, "base_link", "link_7");

    const rocos::JntArray q_home = makeHomeJoints();

    // ── 2. 计算初始末端位姿 ──
    const Pose initial_pose = computeFk(model, q_home);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n初始关节角 (deg): ";
    for (int i = 0; i < 7; ++i) {
        std::cout << q_home(i) / kDegToRad << " ";
    }
    std::cout << "\n初始末端位置: [" << initial_pose.px << ", "
              << initial_pose.py << ", " << initial_pose.pz << "]" << std::endl;

    // ── 3. 创建 MoveSvdJog ──
    // 7 轴机械臂，零空间维度 = 7 - 6 = 1
    constexpr int kNullDim = 7 - 6;

    rocos::MoveSvdJog svd_jog(kDt, kTimeout, &model);
    svd_jog.setInitialPosition(q_home);

    rocos::Result rc = svd_jog.Reset();
    if (!isOk(rc)) {
        std::cerr << "MoveSvdJog Reset failed, rc=" << static_cast<int>(rc) << std::endl;
        return 1;
    }

    // ── 4. 运行 SVD 零空间点动 2 秒 ──
    const int steps = static_cast<int>(kDuration / kDt);
    std::vector<TestSample> samples;
    rocos::JntArray q_current = q_home;

    // 记录初始采样
    samples.push_back({0.0, initial_pose,
                       {q_home(0), q_home(1), q_home(2), q_home(3),
                        q_home(4), q_home(5), q_home(6)}});

    for (int step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) * kDt;

        // 喂入零空间维度速度（7轴只有1个冗余维度）
        rc = svd_jog.FeedSvdJog({kSvdSpeed});
        if (!isOk(rc)) {
            std::cerr << "FeedSvdJog failed at t=" << t
                      << ", rc=" << static_cast<int>(rc) << std::endl;
            return 1;
        }


        rocos::Reference ref;
        rc = svd_jog.GenerateRef(ref);
        if (!isOk(rc)) {
            std::cerr << "GenerateRef failed at t=" << t
                      << ", rc=" << static_cast<int>(rc) << std::endl;
            return 1;
        }

        if (!extractJointRef(ref, q_current)) {
            std::cerr << "Reference is not JntArray at t=" << t << std::endl;
            return 1;
        }

        // 定期采样 FK
        if (step % kSampleEvery == 0 || step == steps) {
            const Pose fk = computeFk(model, q_current);
            samples.push_back({t, fk,
                               {q_current(0), q_current(1), q_current(2),
                                q_current(3), q_current(4), q_current(5), q_current(6)}});
        }
    }

    // ── 5. 结果分析 ──
    const Pose final_pose = samples.back().pose;
    const double pos_delta   = positionDelta(initial_pose, final_pose);
    const double rot_delta   = rotationDelta(initial_pose, final_pose);

    // 末端位姿不应变化（零空间运动的本质属性）
    constexpr double kPosTolerance = 1e-4;   // 位置容许误差 (m)
    constexpr double kRotTolerance = 1e-4;   // 旋转容许误差

    const bool pos_pass = pos_delta < kPosTolerance;
    const bool rot_pass = rot_delta < kRotTolerance;
    const bool test_pass = pos_pass && rot_pass;

    // 计算关节变化量
    double max_joint_delta_rad = 0.0;
    for (int i = 0; i < 7; ++i) {
        const double d = std::abs(q_current(i) - q_home(i));
        if (d > max_joint_delta_rad) max_joint_delta_rad = d;
    }

    std::cout << "\n═══════════════════════════════════════════════════" << std::endl;
    std::cout << "  SVD 零空间点动离线规划测试结果" << std::endl;
    std::cout << "═══════════════════════════════════════════════════" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  运行时长:             " << kDuration << " s" << std::endl;
    std::cout << "  零空间维度:           " << kNullDim << " (7 - 6)" << std::endl;
    std::cout << "  SVD 速度:             " << kSvdSpeed << " rad/s" << std::endl;

    std::cout << "\n  末端位置初始:         [" << initial_pose.px << ", "
              << initial_pose.py << ", " << initial_pose.pz << "]" << std::endl;
    std::cout << "  末端位置终止:         [" << final_pose.px << ", "
              << final_pose.py << ", " << final_pose.pz << "]" << std::endl;
    std::cout << "  位置偏差 (m):         " << pos_delta
              << "  限值: " << kPosTolerance
              << "  [" << (pos_pass ? "✓ PASS" : "✗ FAIL") << "]" << std::endl;

    std::cout << "\n  旋转偏差 (avg):       " << rot_delta
              << "        限值: " << kRotTolerance
              << "  [" << (rot_pass ? "✓ PASS" : "✗ FAIL") << "]" << std::endl;

    std::cout << "\n  关节最大变化 (rad):   " << max_joint_delta_rad
              << " (" << max_joint_delta_rad / kDegToRad << " deg)"
              << std::endl;

    // 打印关节变化明细
    std::cout << "\n  关节角变化 (deg):" << std::endl;
    for (int i = 0; i < 7; ++i) {
        const double delta = (q_current(i) - q_home(i)) / kDegToRad;
        std::cout << "    J" << i << ": " << std::setw(10) << delta << " deg" << std::endl;
    }

    std::cout << "\n  采集样本数:           " << samples.size() << std::endl;
    std::cout << "═══════════════════════════════════════════════════" << std::endl;

    if (test_pass) {
        std::cout << "\n  ✅ 测试通过 — 零空间点动未改变末端位姿"
                  << std::endl;
    } else {
        std::cout << "\n  ❌ 测试失败 — 末端位姿超出容许范围" << std::endl;
    }

    return test_pass ? 0 : 1;
}

}  // namespace

int main() {
    return runTest();
}
