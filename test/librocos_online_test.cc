#include <librocos/robot.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
}  // namespace

// 手动在线测试，启动前需准备并使能 App 和仿真。UDP 报文由外部抓包记录。
int main(int argc, char** argv) {
    try {
        if (argc != 5)
            throw std::invalid_argument(
                "Usage: librocos_online_test seconds period amplitude sessions");
        const double seconds = std::stod(argv[1]);
        const double period = std::stod(argv[2]);
        const double amplitude = std::stod(argv[3]);
        const int sessions = std::stoi(argv[4]);
        if (!std::isfinite(seconds) || !std::isfinite(period) || !std::isfinite(amplitude) ||
            seconds <= 0 || period <= 0 || std::abs(amplitude) > 0.1 || sessions <= 0)
            throw std::invalid_argument("Invalid duration, period, amplitude or session count");
        if (std::abs(seconds / period - std::round(seconds / period)) > 1e-8)
            throw std::invalid_argument("Duration must contain complete periods");
        std::signal(SIGINT, interrupt);
        std::signal(SIGTERM, interrupt);
        librocos::Robot robot("127.0.0.1");
        const auto initial_state = robot.readOnce();
        if (!initial_state.enabled || initial_state.robot_state != "STOPPED" ||
            initial_state.work_mode != "position")
            throw std::runtime_error("Expected enabled position-mode robot in STOPPED");
        robot.enableRci();
        librocos::JointArray initial{};
        for (int session = 0; session < sessions && !interrupted; ++session) {
            double elapsed = 0;
            std::size_t samples = 0;
            const auto started = std::chrono::steady_clock::now();
            robot.control([&](const librocos::RobotState& state, librocos::Duration dt) {
                if (session == 0 && dt.toNSec() == 0) initial = state.q_d;
                elapsed += dt.toSec();
                ++samples;
                const double phase = 2 * std::acos(-1.0) * std::min(elapsed, seconds) / period;
                const double wave = (1 - std::cos(phase)) / 2;
                librocos::JointPositions command(initial, state.dof);
                for (std::size_t joint = 0; joint < state.dof; ++joint) {
                    // 复用示例的平滑余弦往返形状，交替方向，端点前三阶导数为零。
                    const double direction = joint % 2 ? -1.0 : 1.0;
                    command.q[joint] += direction * amplitude * wave * wave;
                    if (std::abs(state.q[joint] - initial[joint]) > 0.15)
                        throw std::runtime_error("Online test joint excursion exceeds 0.15 rad");
                }
                if (interrupted || elapsed >= seconds + 0.5)
                    return librocos::MotionFinished(command);
                return command;
            });
            const auto status = robot.rciStatus();
            if (status.control_active || status.robot_state != "STOPPED" || !status.last_control ||
                status.last_control->fault_code)
                throw std::runtime_error("Session did not stop cleanly");
            const double wall =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            std::cout << "session=" << session + 1 << " samples=" << samples
                      << " sample_elapsed_s=" << elapsed << " wall_s=" << wall
                      << " final_fault=" << status.last_control->fault_code << std::endl;
        }
        return interrupted ? 2 : 0;
    } catch (const std::exception& error) {
        std::cerr << "Online test failed: " << error.what() << std::endl;
        if (const auto* control = dynamic_cast<const librocos::ControlException*>(&error)) {
            if (control->status)
                std::cerr << "Final state=" << control->status->robot_state
                          << " fault_code=" << control->code() << std::endl;
        }
        return 1;
    }
}
