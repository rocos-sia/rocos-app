// =============================================================================
// hardware_enable_test.cc — 真实硬件上使能 + joint_2 增量运动测试
//
// 测试内容：
//   1. Hardware 初始化（连接 EtherCAT 共享内存，等待 OP）
//   2. 设置 CSP 模式，读取当前位置
//   3. 依次上使能所有关节
//   4. joint_2 执行 +10° 增量运动（梯形速度规划: 加速→匀速→减速）
//   5. 读取运动后各关节位置
//   6. 依次下使能
//
// 用法：
//   ./build/bin/hardware_enable_test [ecat_id] [config_path]
//
// 重要：运行前必须先启动 EtherCAT 主站，等待总线进入 OP 状态！
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include "src/hardware.hpp"

// ==========================================================================
// 梯形速度规划器（T 型曲线）
// ==========================================================================
struct TrapProfile {
    double t_acc;       // 加速时间 [s]
    double t_cruise;    // 匀速时间 [s]
    double t_decel;     // 减速时间 [s]
    double v_peak;      // 实际峰值速度 [rad/s]
    double d_acc;       // 加速段位移 [rad]

    /// @brief 计算梯形曲线参数
    /// @param dist  位移绝对值 [rad]
    /// @param v_max 最大允许速度 [rad/s]
    /// @param acc   加速度 [rad/s²]
    static TrapProfile compute(double dist, double v_max, double acc) {
        TrapProfile p{};
        double d_full = v_max * v_max / acc;  // 能否达到全速所需的最小位移
        if (dist >= d_full) {
            p.t_acc   = v_max / acc;
            p.d_acc   = 0.5 * acc * p.t_acc * p.t_acc;
            p.t_cruise = (dist - 2.0 * p.d_acc) / v_max;
            p.t_decel  = p.t_acc;
            p.v_peak   = v_max;
        } else {
            // 三角曲线（位移不够达到全速）
            p.t_acc   = std::sqrt(dist / acc);
            p.d_acc   = 0.5 * acc * p.t_acc * p.t_acc;
            p.t_cruise = 0.0;
            p.t_decel  = p.t_acc;
            p.v_peak   = acc * p.t_acc;
        }
        return p;
    }

    /// @brief 时刻 t 的位置（从起点算，沿正方向）
    double posAt(double t) const {
        if (t < 0.0)       return 0.0;
        if (t >= total())  return d_acc * 2.0 + v_peak * t_cruise;
        if (t < t_acc)
            return 0.5 * v_peak / t_acc * t * t;  // 用 v_peak/t_acc 避免浮点误差
        double t1 = t - t_acc;
        if (t1 < t_cruise)
            return d_acc + v_peak * t1;
        double t2 = t1 - t_cruise;
        return d_acc + v_peak * t_cruise + v_peak * t2 - 0.5 * (v_peak / t_decel) * t2 * t2;
    }

    double total() const { return t_acc + t_cruise + t_decel; }
};

// ==========================================================================
// 辅助函数
// ==========================================================================
static std::string resolveConfig(const std::string& name) {
    std::string path = "../../config/" + name;
    std::ifstream t(path); if (t.good()) return path;
    t.open("config/" + name); if (t.good()) return "config/" + name;
    t.open(name); if (t.good()) return name;
    return name;
}
static double rad2deg(double r) { return r * 180.0 / M_PI; }

// ==========================================================================
// main
// ==========================================================================
int main(int argc, char* argv[]) {
    int ecat_id = 0;
    std::string config_path = "hardware_talon_config.yaml";
    if (argc > 1) ecat_id = std::stoi(argv[1]);
    if (argc > 2) config_path = argv[2];
    config_path = resolveConfig(config_path);

    // 运动参数
    const double kMaxVel = 0.5;           // rad/s
    const double kAcc    = 2.0;           // rad/s²
    const double kDelta  = 0.174532925;   // +10° (10 * π / 180)
    const int    kCycleUs = 1000;         // 控制周期 1ms

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     ROCOS Hardware 上使能 + joint_2 增量运动测试            ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║  ecat_id = " << std::setw(47) << std::left << ecat_id << "║\n";
    std::cout << "║  config  = " << std::setw(47) << std::left << config_path << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    // ===== 0. 预检查 =====
    {
        auto* ec = rocos::EcatConfig::getInstance(ecat_id);
        if (ec && ec->ecatBus) {
            int st = ec->getBusCurrentState();
            const char* ss = "???";
            switch (st) { case 1:ss="INIT";break;case 2:ss="PREOP";break;
                          case 4:ss="SAFEOP";break;case 8:ss="OP";break;
                          case 3:ss="BOOTSTRAP";break; }
            std::cout << "[0/6] 主站: " << ss << "(" << st << "), 从站: " << ec->getSlaveNum() << std::endl;
            if (st < 4) {
                std::cout << "  ⚠️  主站未到 SAFEOP/OP，等 master 进入 OP 再试\n";
                return EXIT_FAILURE;
            }
        }
    }

    // ===== 1. 初始化 =====
    std::cout << "[1/6] 初始化 Hardware ..." << std::endl;
    rocos::Hardware* hw = nullptr;
    try { hw = new rocos::Hardware(config_path, ecat_id); }
    catch (const std::exception& e) {
        std::cerr << "  ❌ " << e.what() << std::endl; return EXIT_FAILURE;
    }
    const auto& cfg = hw->getConfig();
    size_t n = cfg.drives.size();
    std::cout << "  ✅ " << n << " 个驱动器\n";

    const rocos::Drive* j2 = hw->findDriveByName("joint_2");
    if (!j2) { std::cerr << "  ❌ 找不到 joint_2\n"; delete hw; return EXIT_FAILURE; }
    int j2id = j2->id;
    std::cout << "  joint_2 → id=" << j2id << "\n\n";

    // ===== 2. CSP 模式 + 读位置 =====
    std::cout << "[2/6] CSP 模式，当前位置:\n";
    hw->SetMode(static_cast<int8_t>(rocos::ModeOfOperation::CyclicSynchronousPositionMode));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "  ┌──────┬───────────────┬───────────────┬───────────────┐\n";
    std::cout << "  │  ID  │  Joint Name   │  Position(rad)│  Position(deg)│\n";
    std::cout << "  ├──────┼───────────────┼───────────────┼───────────────┤\n";
    for (size_t i = 0; i < n; ++i) {
        int id = cfg.drives[i].id;
        double p = hw->GetJointPosition(id);
        std::cout << "  │ " << std::setw(4) << id << " │ "
                  << std::setw(13) << cfg.drives[i].joint_name << " │ "
                  << std::setw(13) << std::fixed << std::setprecision(6) << p << " │ "
                  << std::setw(13) << rad2deg(p) << " │\n";
    }
    std::cout << "  └──────┴───────────────┴───────────────┴───────────────┘\n\n";

    // ===== 3. 上使能 =====
    std::cout << "[3/6] 上使能所有关节 ..." << std::endl;
    for (size_t i = 0; i < n; ++i) {
        int id = cfg.drives[i].id;
        double b4 = hw->GetJointPosition(id);
        std::cout << "  " << cfg.drives[i].joint_name << " (id=" << id << ") ... ";
        hw->SetJointEnabled(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        double af = hw->GetJointPosition(id);
        std::cout << (std::abs(af - b4) < 0.05 ? "✅" : "⚠️")
                  << " Δ=" << std::abs(af - b4) << " rad\n";
    }
    std::cout << std::endl;

    // ===== 4. joint_2 增量 +10° 梯形运动 =====
    std::cout << "[4/6] joint_2 增量 +10° ..." << std::endl;
    double start_pos = hw->GetJointPosition(j2id);
    double target    = start_pos + kDelta;
    std::cout << "  " << start_pos << " rad → " << target << " rad (+"
              << rad2deg(kDelta) << "°)\n";

    auto profile = TrapProfile::compute(kDelta, kMaxVel, kAcc);
    int64_t steps = static_cast<int64_t>(profile.total() * 1e6 / kCycleUs);
    std::cout << "  梯形: t_acc=" << profile.t_acc << "s, t_cruise=" << profile.t_cruise
              << "s, t_decel=" << profile.t_decel << "s, total=" << profile.total()
              << "s, v_peak=" << profile.v_peak << " rad/s, " << steps << " steps\n";

    auto t0 = std::chrono::steady_clock::now();
    for (int64_t i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) * kCycleUs * 1e-6;
        double pos = start_pos + profile.posAt(t);
        hw->SetJointPosition(j2id, pos);

        auto next = t0 + std::chrono::microseconds(static_cast<int64_t>((i + 1) * kCycleUs));
        std::this_thread::sleep_until(next);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    double final_pos = hw->GetJointPosition(j2id);
    double err = std::abs(final_pos - target);
    std::cout << "  终点: " << final_pos << " rad, 误差: " << err
              << " rad (" << rad2deg(err) << "°)\n\n";

    // ===== 5. 运动后位置 =====
    std::cout << "[5/6] 运动后各关节位置:\n";
    std::cout << "  ┌──────┬───────────────┬───────────────┬───────────────┐\n";
    std::cout << "  │  ID  │  Joint Name   │  Position(rad)│  Position(deg)│\n";
    std::cout << "  ├──────┼───────────────┼───────────────┼───────────────┤\n";
    auto qf = hw->GetPosition();
    for (size_t i = 0; i < n; ++i) {
        int id = cfg.drives[i].id;
        std::cout << "  │ " << std::setw(4) << id << " │ "
                  << std::setw(13) << cfg.drives[i].joint_name << " │ "
                  << std::setw(13) << std::fixed << std::setprecision(6) << qf(static_cast<int>(i)) << " │ "
                  << std::setw(13) << rad2deg(qf(static_cast<int>(i))) << " │\n";
    }
    std::cout << "  └──────┴───────────────┴───────────────┴───────────────┘\n\n";

    // ===== 6. 下使能 =====
    std::cout << "[6/6] 下使能所有关节 ..." << std::endl;
    for (size_t i = 0; i < n; ++i) {
        std::cout << "  " << cfg.drives[i].joint_name
                  << " (id=" << cfg.drives[i].id << ") ... ";
        hw->SetJointDisabled(cfg.drives[i].id);
        std::cout << "✅\n";
    }
    std::cout << std::endl;

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  测试完成 — joint_2 增量 +" << rad2deg(kDelta)
              << "°, 到达误差 " << rad2deg(err) << "°"
              << std::setw(28) << "║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";

    delete hw;
    return (err < 0.01745) ? EXIT_SUCCESS : EXIT_FAILURE;  // 误差 < 1° 算成功
}
