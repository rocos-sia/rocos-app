
// =============================================================================
// mode_switch_test.cc — 位置/力矩模式切换测试
//
// 测试内容：
//   1. Hardware 初始化，CSP 模式上使能
//   2. joint_2 在 CSP 下移动到目标位置
//   3. 切换到 CST（力矩模式），joint_2 施加力矩
//   4. 切回 CSP，joint_2 回到目标位置
//
// 用法：
//   ./build/bin/mode_switch_test [ecat_id] [config_path]
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

    const int    kCycleUs = 1000;          // 控制周期 1ms
    const double kCSPMove = 0.174532925;   // +10°
    const double kCSTTorque = -36.0;         // Nm (力矩模式施加值)
    const double kCSTDuration = 2.0;       // 力矩模式持续时间 [s]
    const double kMaxVel = 0.5;            // rad/s
    const double kAcc = 2.0;               // rad/s²

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     ROCOS 位置/力矩模式切换测试                              ║\n";
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
                std::cout << "  ⚠️  主站未到 SAFEOP/OP\n";
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

    // ===== 2. CSP 模式 + 上使能 + 读位置 =====
    std::cout << "[2/6] CSP 模式上使能，初始位置:\n";
    hw->SetMode(static_cast<int8_t>(
        rocos::ModeOfOperation::CyclicSynchronousPositionMode));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (size_t i = 0; i < n; ++i)
        hw->SetJointEnabled(cfg.drives[i].id);
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

    // ===== 3. CSP: joint_2 移动到 +10° =====
    std::cout << "[3/6] CSP: joint_2 移动到 +10° ..." << std::endl;
    double start_pos = hw->GetJointPosition(j2id);
    double target = start_pos + kCSPMove;
    std::cout << "  " << start_pos << " rad → " << target
              << " rad (+" << rad2deg(kCSPMove) << "°)\n";

    // 简单梯形插补
    double t_acc = kMaxVel / kAcc;
    double d_acc = 0.5 * kAcc * t_acc * t_acc;
    double t_cruise = (kCSPMove - 2.0 * d_acc) / kMaxVel;
    if (t_cruise < 0) {
        t_acc = std::sqrt(kCSPMove / kAcc);
        d_acc = 0.5 * kAcc * t_acc * t_acc;
        t_cruise = 0.0;
    }
    double total_t = 2.0 * t_acc + t_cruise;
    int64_t steps = static_cast<int64_t>(total_t * 1e6 / kCycleUs);

    auto t0 = std::chrono::steady_clock::now();
    double frac;
    for (int64_t i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) * kCycleUs * 1e-6;
        
        if (t < t_acc)
            frac = 0.5 * kAcc * t * t / kCSPMove;
        else if (t < t_acc + t_cruise)
            frac = (d_acc + kMaxVel * (t - t_acc)) / kCSPMove;
        else
            frac = 1.0 - 0.5 * kAcc * (total_t - t) * (total_t - t) / kCSPMove;
        hw->SetJointPosition(j2id, start_pos + kCSPMove * frac);
      
        auto next = t0 + std::chrono::microseconds(
            static_cast<int64_t>((i + 1) * kCycleUs));
        std::this_thread::sleep_until(next);
    }
      std::cout << "send JointPosition(joint_2) = "
                  << start_pos + kCSPMove * frac << " rad\n";
        std::cout << "JointPosition(joint_2) = "
                  << hw->GetJointPosition(j2id) << " rad\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    double csp_end = hw->GetJointPosition(j2id);
    std::cout << "  CSP 到位: " << csp_end << " rad ("
              << rad2deg(csp_end) << "°)\n\n";

    // ===== 4. 切换到 CST（力矩模式）=====
    std::cout << "[4/6] 切换到 CST 力矩模式，施加 "
              << kCSTTorque << " Nm 持续 " << kCSTDuration << "s ..." << std::endl;
    hw->SetJointMode(j2id, static_cast<int8_t>(
        rocos::ModeOfOperation::CyclicSynchronousTorqueMode));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    double pre_cst = hw->GetJointPosition(j2id);
    hw->SetJointTorque(j2id, kCSTTorque);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(kCSTDuration * 1000)));
    double post_cst = hw->GetJointPosition(j2id);
    double cst_delta = post_cst - pre_cst;
    std::cout << "  CST 前: " << pre_cst << " rad, CST 后: " << post_cst
              << " rad, 偏移: " << cst_delta << " rad ("
              << rad2deg(cst_delta) << "°)\n";
    // 仿真环境下施加力矩应导致关节移动
    std::cout << (std::abs(cst_delta) > 0.001 ? "  ✅ 关节在力矩下产生了位移"
                                              : "  ⚠️  关节未明显位移")
              << "\n\n";

    // ===== 5. 切回 CSP =====
    std::cout << "[5/6] 切回 CSP 模式 ..." << std::endl;
     double cur_pos = hw->GetJointPosition(j2id);
    hw->SetJointPosition(j2id, cur_pos);
    hw->SetJointMode(j2id, static_cast<int8_t>(
        rocos::ModeOfOperation::CyclicSynchronousPositionMode));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 先将目标位置同步为当前实际位置，防止模式切换瞬间飞车
    cur_pos = hw->GetJointPosition(j2id);
    hw->SetJointPosition(j2id, cur_pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double csp_return = hw->GetJointPosition(j2id);
    std::cout << "  CSP 复位后: " << csp_return << " rad\n\n";

    // ===== 6. 下使能 =====
    std::cout << "[6/6] 下使能所有关节 ..." << std::endl;
    for (size_t i = 0; i < n; ++i) {
        hw->SetJointDisabled(cfg.drives[i].id);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cout << "  ✅ 全部下使能\n\n";

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  测试完成 — CSP→CST→CSP 模式切换                           ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";

    delete hw;
    return EXIT_SUCCESS;
}