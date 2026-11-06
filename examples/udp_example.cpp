/**
 * @file udp_example.cpp
 * @brief UDP 伺服运动控制示例 —— 测试 1kHz 实时通信
 *
 * 用法: udp_example [host] [http_port] [udp_port]
 * 示例: udp_example localhost 8080 8081
 *
 * 工作流程:
 *   1. 查询 RCI；若未开启，自动申请普通 HTTP 控制权并开启 RCI
 *   2. 无额外凭据接入 RCI，自动取得 UDP 会话 ID（关节空间）
 *   3. 通过 UDP 以 1kHz 发送 MotionGeneratorCommand，接收 RobotState
 *   4. 每秒输出通信统计（实际频率、RTT、丢包率、服务器成功率）
 *   5. 收到 SIGINT 后通过 HTTP 停止伺服模式并安全退出
 *
 * 前置条件:
 *   - rocosAppMain 已启动（使用已准备好的硬件或离线模拟主站配置）
 *   - 机械臂已 enable: curl -X POST localhost:8080/api/robot/enable
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <httplib.h>
#include <json.hpp>

#include "../src/servo_type.hpp"

// ============================================================================
// 全局标志 —— SIGINT 信号处理
// ============================================================================

namespace {

volatile sig_atomic_t g_running = 1;

void signalHandler(int /*signo*/) {
    g_running = 0;
}

}  // namespace

// ============================================================================
// main
// ============================================================================

int main(int argc, char* argv[]) {
    // ---- 解析命令行参数 ----
    const std::string host      = (argc >= 2) ? argv[1] : "localhost";
    const int         http_port = (argc >= 3) ? std::stoi(argv[2]) : 8080;
    const int         udp_port  = (argc >= 4) ? std::stoi(argv[3]) : 8081;

    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║   UDP 伺服运动控制示例 (1kHz)                 ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    std::cout << "║ HTTP: " << host << ":" << http_port
              << "  |  UDP: " << host << ":" << udp_port << "\n";
    std::cout << "╚══════════════════════════════════════════════╝\n\n";

    // 注册信号处理
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cerr << "[ERROR] 无法注册 SIGINT 信号处理器\n";
        return 1;
    }

    // ---- Step 1: 通过 HTTP 启动伺服模式 ----
    httplib::Client http(host, http_port);
    http.set_connection_timeout(5);
    http.set_read_timeout(5);
    std::array<uint8_t, 16> servo_session{};
    const auto status_response = http.Get("/api/robot/rci/status");
    if (!status_response) {
        std::cerr << "[ERROR] 无法查询 RCI 状态\n";
        return 1;
    }
    const auto status = nlohmann::json::parse(status_response->body);
    if (!status.value("success", false)) {
        std::cerr << "[ERROR] RCI 状态不可用\n";
        return 1;
    }
    if (!status["data"].value("enabled", false)) {
        const std::string client_id = "udp-example-" + std::to_string(getpid());
        const nlohmann::json request = {{"client_id", client_id},
                                        {"client_name", "UDP example"}};
        const auto acquired = http.Post("/api/control/acquire", request.dump(),
                                        "application/json");
        if (!acquired) return 1;
        const auto owner = nlohmann::json::parse(acquired->body);
        if (!owner.value("success", false)) {
            std::cerr
                << "[ERROR] 已有其他控制者，请由上位机开启 RCI 后再连接\n";
            return 1;
        }
        const httplib::Headers headers = {
            {"X-Rocos-Control-Token",
             owner["data"]["token"].get<std::string>()},
            {"X-Rocos-Client-Id", client_id}};
        const auto enabled = http.Post("/api/robot/rci/enable", headers, "{}",
                                       "application/json");
        // RCI 接入与 HTTP 租约独立，程序无需续期上位机控制权。
        http.Post("/api/control/release", headers, "{}", "application/json");
        if (!enabled ||
            !nlohmann::json::parse(enabled->body).value("success", false)) {
            std::cerr << "[ERROR] 开启 RCI "
                         "失败，请确认机器人已使能、停止且没有运行脚本\n";
            return 1;
        }
    }

    {
        nlohmann::json body;
        body["port"] = udp_port;
        body["mode"] = "joint";
        auto res =
            http.Post("/api/robot/rci/start", body.dump(), "application/json");

        if (!res) {
            std::cerr << "[ERROR] HTTP 连接失败 — rocosAppMain 是否已启动？\n";
            return 1;
        }

        auto json = nlohmann::json::parse(res->body);
        if (!json.value("success", false)) {
            std::cerr << "[ERROR] 启动伺服失败: "
                      << json.value("message", "unknown")
                      << " (code=" << json.value("code", -1) << ")\n";
            std::cerr << "[HINT]  请确认机械臂已 enable: "
                      << "curl -X POST " << host << ":" << http_port
                      << "/api/robot/enable\n";
            return 1;
        }
        if (json["data"].value("protocol_version", 0) != SERVO_PROTOCOL_VERSION)
            return 1;
        const auto session_hex =
            json["data"].value("session_id", std::string{});
        if (session_hex.size() != 32) return 1;
        for (size_t i = 0; i < servo_session.size(); ++i)
            servo_session[i] = static_cast<uint8_t>(
                std::stoul(session_hex.substr(i * 2, 2), nullptr, 16));
        std::cout << "[INFO] ✓ 伺服模式已启动, 端口=" << json["data"]["port"]
                  << "\n";
    }

    // ---- Step 3: 初始化 UDP socket ----
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[ERROR] 无法创建 UDP socket\n";
        http.Post("/api/robot/rci/stop", "", "application/json");
        return 1;
    }

    // 解析服务器地址
    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port   = htons(static_cast<uint16_t>(udp_port));
    // 如果 host 是 "localhost"，解析为 127.0.0.1
    if (host == "localhost") {
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    } else if (inet_pton(AF_INET, host.c_str(),
                         &server_addr.sin_addr) != 1) {
        std::cerr << "[ERROR] 无法解析主机名: " << host << "\n";
        close(sockfd);
        http.Post("/api/robot/rci/stop", "", "application/json");
        return 1;
    }

    // connect UDP socket 以便使用 send/recv 代替 sendto/recvfrom
    if (connect(sockfd, reinterpret_cast<struct sockaddr*>(&server_addr),
                sizeof(server_addr)) < 0) {
        std::cerr << "[ERROR] UDP connect 失败\n";
        close(sockfd);
        http.Post("/api/robot/rci/stop", "", "application/json");
        return 1;
    }

    // ---- Step 4: 获取初始机器人状态（先用长超时探测，再恢复短超时）----
    MotionGeneratorCommand cmd{};
    cmd.session_id=servo_session;
    RobotState             state{};

    // 探测阶段：超时设 200ms，等待 UDP 线程就绪并回包
    {
        struct timeval tv {};
        tv.tv_sec  = 0;
        tv.tv_usec = 200000;  // 200ms
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
    }

    bool probe_ok = false;
    for (int attempt = 0; attempt < 10 && !probe_ok; ++attempt) {
        // message_id = 0 → 服务端识别为纯查询包，只回复状态，不更新运动指令
        cmd.message_id = 0;
        send(sockfd, &cmd, sizeof(cmd), 0);

        char    buf[sizeof(RobotState)];
        ssize_t n = recv(sockfd, buf, sizeof(buf), 0);
        if (n == static_cast<ssize_t>(sizeof(RobotState))) {
            std::memcpy(&state, buf, sizeof(RobotState));
            probe_ok = true;
        }
    }

    if (probe_ok) {
        std::cout << "[INFO] ✓ 获取初始状态, server_msg_id="
                  << state.message_id << "\n";
        std::cout << "[INFO]   关节位置 q = [";
        for (int i = 0; i < static_cast<int>(MAX_DOF); ++i) {
            std::cout << std::fixed << std::setprecision(4) << state.q[i];
            if (i < static_cast<int>(MAX_DOF) - 1) std::cout << ", ";
        }
        std::cout << "]\n";
    } else {
        std::cerr << "[ERROR] 探测失败：无法获取初始关节位置，请检查伺服是否正常启动\n";
        close(sockfd);
        http.Post("/api/robot/rci/stop", "", "application/json");
        return 1;
    }

    // 以当前位置作为 sin 运动基准，同时初始化 cmd.q_c
    // 避免探测包的零位指令被控制器在首个有效指令到来前执行
    double q_init[MAX_DOF]{};
    int    actual_dof = 0;  // 从探测响应推断实际关节数
    for (int i = 0; i < static_cast<int>(MAX_DOF); ++i) {
        q_init[i]  = state.q[i];
        cmd.q_c[i] = state.q[i];
        // state.q 由服务端填充到 joint_count 位，之后为 0；取最后一个非全零 index
        if (state.q[i] != 0.0 || i == 0) {
            actual_dof = i + 1;
        }
    }
    // 若全为 0（仿真零位），至少用 MAX_DOF 作回退
    if (actual_dof == 0) actual_dof = static_cast<int>(MAX_DOF);

    // 恢复 500us 超时 —— 控制循环非阻塞 recv
    {
        struct timeval tv {};
        tv.tv_sec  = 0;
        tv.tv_usec = 500;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
    }

    // ---- Sin 运动参数 ----
    // 每个关节使用不同频率，确保在任意时刻各关节指令值都明显不同
    // 频率间隔 0.1 Hz，关节 i 的频率 = kSinBaseFreq + i * kSinFreqStep
    constexpr double kSinAmplitude = 0.1;      // 幅值 [rad]，约 5.7°
    constexpr double kSinBaseFreq  = 0.2;      // 关节 0 的频率 [Hz]
    constexpr double kSinFreqStep  = 0.1;      // 相邻关节频率差 [Hz]
    constexpr double kTwoPi        = 6.283185307179586;

    std::cout << "[INFO] Sin 运动参数: 幅值=" << kSinAmplitude
              << " rad, 基础频率=" << kSinBaseFreq
              << " Hz, 关节间频率差=" << kSinFreqStep << " Hz\n";
    std::cout << "[INFO] 实际关节数 (actual_dof)=" << actual_dof << "\n\n";

    // ---- Step 5: 1kHz 伺服控制循环 ----
    constexpr int    kTargetHz        = 1000;
    constexpr auto   kCycleDuration   = std::chrono::microseconds(1000);
    constexpr int    kReportInterval  = 1000;    // 每秒统计一次
    constexpr double kLateThresholdUs = 300.0;   // 超过此值视为周期滞后

    std::cout << "[INFO] 开始 1kHz 控制循环 ... (Ctrl+C 停止)\n";
    std::cout << "[INFO] 注：首次循环可能因冷缓存而偏慢，运行几秒后趋于稳定\n\n";

    // 统计变量
    int64_t total_loops   = 0;
    int64_t send_errors   = 0;
    int64_t recv_missed   = 0;   // recv 返回 ≤0 的次数
    int64_t recv_bad_size = 0;   // 收到但大小不匹配的畸形包
    int64_t late_cycles   = 0;
    double  sum_loop_us   = 0.0;
    double  last_rtt_us   = 0.0;
    double  min_rtt_us    = 1e9;
    double  max_rtt_us    = 0.0;

    uint64_t cmd_msg_id = 0;  // 控制包从 1 开始（第一次 ++cmd_msg_id = 1）

    auto loop_start  = std::chrono::steady_clock::now();
    auto next_wakeup = loop_start;
    auto last_report = loop_start;
    int  loops_since_report = 0;

    while (g_running) {
        auto cycle_begin = std::chrono::steady_clock::now();

        // ---- 计算当前运动时间 [s] ----
        double t_s = static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_begin - loop_start).count()) * 1e-6;

        // ---- 生成 sin 关节指令 ----
        // 各关节使用不同频率，保证任意时刻值明显不同，便于验证通信
        for (int i = 0; i < static_cast<int>(MAX_DOF); ++i) {
            const double freq = kSinBaseFreq + kSinFreqStep * static_cast<double>(i);
            cmd.q_c[i] = q_init[i] + kSinAmplitude * std::sin(kTwoPi * freq * t_s);
        }

        // ---- 客户端诊断：每秒打印一次指令，确认 sin 值随时间变化 ----
        if (total_loops % kReportInterval == 0 && total_loops > 0) {
            std::cout << "[CMD]  t=" << std::fixed << std::setprecision(2) << t_s
                      << "s q=[";
            for (int i = 0; i < actual_dof; ++i) {
                std::cout << std::setprecision(4) << cmd.q_c[i];
                if (i < actual_dof - 1) std::cout << ",";
            }
            std::cout << "]\n";
        }

        // ---- 周期滞后检测 ----
        auto lag = std::chrono::duration_cast<std::chrono::microseconds>(
                       cycle_begin - next_wakeup).count();
        if (lag > static_cast<int64_t>(kLateThresholdUs)) {
            ++late_cycles;
            // 累积落后过多时重置调度基准，避免追赶导致更严重的滞后
            if (lag > static_cast<int64_t>(kCycleDuration.count() * 5)) {
                next_wakeup = cycle_begin;
            }
        }

        // ---- 发送 MotionGeneratorCommand ----
        cmd.message_id = ++cmd_msg_id;
        auto   send_t = std::chrono::steady_clock::now();
        ssize_t sent  = send(sockfd, &cmd, sizeof(cmd), 0);

        if (sent != static_cast<ssize_t>(sizeof(MotionGeneratorCommand))) {
            ++send_errors;
        }

        // ---- 接收 RobotState ----
        char    recv_buf[sizeof(RobotState)];
        ssize_t recvd = recv(sockfd, recv_buf, sizeof(recv_buf), 0);
        auto    recv_t = std::chrono::steady_clock::now();

        if (recvd == static_cast<ssize_t>(sizeof(RobotState))) {
            std::memcpy(&state, recv_buf, sizeof(RobotState));

            last_rtt_us = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    recv_t - send_t).count());
            if (last_rtt_us < min_rtt_us) min_rtt_us = last_rtt_us;
            if (last_rtt_us > max_rtt_us) max_rtt_us = last_rtt_us;
        } else if (recvd <= 0) {
            ++recv_missed;
        } else {
            ++recv_bad_size;
        }

        // ---- 记录循环耗时 ----
        auto   cycle_end = std::chrono::steady_clock::now();
        double loop_us   = static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_begin).count());
        sum_loop_us += loop_us;

        ++total_loops;
        ++loops_since_report;

        // ---- 每秒输出统计 ----
        if (total_loops % kReportInterval == 0) {
            auto   now       = std::chrono::steady_clock::now();
            double elapsed_s = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    now - last_report).count()) / 1e6;
            double actual_hz  = (elapsed_s > 0.0)
                                    ? static_cast<double>(loops_since_report) / elapsed_s
                                    : 0.0;
            double avg_loop   = (loops_since_report > 0)
                                    ? sum_loop_us / static_cast<double>(loops_since_report)
                                    : 0.0;
            double loss_rate  = (total_loops > 0)
                                    ? static_cast<double>(recv_missed) / static_cast<double>(total_loops) * 100.0
                                    : 0.0;
            double srv_success = state.control_command_success_rate * 100.0;

            std::cout << std::fixed << std::setprecision(1)
                      << "[STAT] "
                      << "频率: " << actual_hz << " Hz"
                      << " | 循环耗时: " << std::setprecision(0) << avg_loop << " us"
                      << " | RTT: " << last_rtt_us << " us"
                      << " | 丢包: " << std::setprecision(2) << loss_rate << "%"
                      << " | 服务器成功率: " << srv_success << "%"
                      << " | 滞后: " << late_cycles
                      << "\n";

            last_report         = now;
            loops_since_report  = 0;
            sum_loop_us         = 0.0;
        }

        // ---- 等待下一周期 ----
        next_wakeup += kCycleDuration;
        auto now = std::chrono::steady_clock::now();
        if (next_wakeup < now) {
            next_wakeup = now + kCycleDuration;
        }
        std::this_thread::sleep_until(next_wakeup);
    }

    // ---- Step 6: 安全退出 ----
    std::cout << "\n[INFO] 正在停止伺服控制 ...\n";

    // 关闭 UDP socket
    close(sockfd);

    {
        auto res = http.Post("/api/robot/rci/stop",
                             "", "application/json");
        if (res) {
            auto json = nlohmann::json::parse(res->body);
            std::cout << "[INFO] ✓ " << json.value("message", "") << "\n";
        } else {
            std::cout << "[WARN] HTTP 停止请求失败，伺服可能仍在运行\n";
        }
    }

    // ---- 最终报告 ----
    double total_elapsed = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - loop_start).count()) / 1e6;
    double avg_hz = (total_elapsed > 0.0)
                        ? static_cast<double>(total_loops) / total_elapsed
                        : 0.0;
    double loss_rate_final = (total_loops > 0)
        ? static_cast<double>(recv_missed) / static_cast<double>(total_loops) * 100.0
        : 0.0;

    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║  最终统计报告                                  ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n"
              << "║ 运行时间:     " << std::setw(12) << std::fixed
              << std::setprecision(2) << total_elapsed << " 秒\n"
              << "║ 总循环数:     " << std::setw(12) << total_loops << "\n"
              << "║ 平均频率:     " << std::setw(12) << std::setprecision(1)
              << avg_hz << " Hz\n"
              << "║ 发送错误:     " << std::setw(12) << send_errors << "\n"
              << "║ 接收丢失:     " << std::setw(12) << recv_missed
              << " (" << std::setprecision(1) << loss_rate_final << "%)\n"
              << "║ 畸形包:       " << std::setw(12) << recv_bad_size << "\n"
              << "║ 周期滞后:     " << std::setw(12) << late_cycles << "\n"
              << "║ 最小 RTT:     " << std::setw(11) << std::setprecision(0)
              << min_rtt_us << " us\n"
              << "║ 最大 RTT:     " << std::setw(11) << max_rtt_us << " us\n"
              << "╚══════════════════════════════════════════════╝\n";

    // 达标判定：平均频率 ≥ 99% 目标即视为通过
    double score = (avg_hz / static_cast<double>(kTargetHz)) * 100.0;
    if (score >= 99.0) {
        std::cout << "\n✅ 1kHz 伺服通信测试通过! (频率达标率: "
                  << std::setprecision(1) << score << "%)\n";
    } else if (score >= 90.0) {
        std::cout << "\n⚠️  频率略低于目标 (达标率: "
                  << std::setprecision(1) << score
                  << "%)，建议检查系统负载或调度策略\n";
    } else {
        std::cout << "\n❌ 1kHz 伺服通信测试未达标 (达标率: "
                  << std::setprecision(1) << score << "%)\n";
    }

    return (score >= 90.0) ? 0 : 1;
}
