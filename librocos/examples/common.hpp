#pragma once

#include <librocos/robot.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>

namespace example {
inline volatile std::sig_atomic_t interrupted = 0;
inline void interrupt(int) { interrupted = 1; }

struct Arguments {
    std::string host{"127.0.0.1"};
    librocos::Options options;
    std::size_t joint{};
    double amplitude{0.01};
    double duration{4.0};
};

inline Arguments arguments(int argc, char** argv) {
    if (argc > 7)
        throw std::invalid_argument(
            "Usage: example [host [http_port [udp_port [joint_index [amplitude_rad "
            "[duration_s]]]]]]");
    Arguments result;
    auto port = [](const char* value) {
        const auto number = std::stoul(value);
        if (!number || number > 65535) throw std::invalid_argument("Port outside 1..65535");
        return static_cast<std::uint16_t>(number);
    };
    if (argc > 1) result.host = argv[1];
    if (argc > 2) result.options.http_port = port(argv[2]);
    if (argc > 3) result.options.udp_port = port(argv[3]);
    if (argc > 4) result.joint = std::stoul(argv[4]);
    if (argc > 5) result.amplitude = std::stod(argv[5]);
    if (argc > 6) result.duration = std::stod(argv[6]);
    if (!std::isfinite(result.amplitude) || !std::isfinite(result.duration) || result.duration <= 0)
        throw std::invalid_argument("Amplitude must be finite and duration must be positive");
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    return result;
}

inline void prepare(librocos::Robot& robot, const Arguments& args) {
    const auto state = robot.readOnce();
    if (args.joint >= state.dof) throw std::invalid_argument("Joint index exceeds robot DOF");
    if (!state.enabled || state.robot_state != "STOPPED")
        throw std::runtime_error(
            "Enable the prepared robot and wait for STOPPED before running motion examples");
    std::cout << "Joint " << args.joint << ", displacement " << args.amplitude << " rad, duration "
              << args.duration << " s. Press Enter to start.\n";
    if (std::cin.get() == std::char_traits<char>::eof())
        throw std::runtime_error("Motion requires an Enter confirmation on stdin");
    if (interrupted) throw std::runtime_error("Interrupted before motion");
    robot.enableRci();
}

inline void pointToPoint(librocos::Robot& robot, const Arguments& args, double displacement,
                         const librocos::JointArray* absolute_target = nullptr) {
    librocos::JointArray initial{}, target{};
    double elapsed = 0;
    robot.control([&](const librocos::RobotState& state, librocos::Duration period) {
        if (period.toNSec() == 0) {
            initial = state.q_d;
            target = absolute_target ? *absolute_target : initial;
            if (!absolute_target) target[args.joint] += displacement;
        }
        elapsed += period.toSec();
        const double u = std::min(1.0, elapsed / args.duration);
        // 七次时间标定，端点速度、加速度和 jerk 均为零。
        const double blend = std::pow(u, 4) * (35 + u * (-84 + u * (70 - 20 * u)));
        librocos::JointPositions command(initial, state.dof);
        for (std::size_t joint = 0; joint < state.dof; ++joint)
            command.q[joint] += (target[joint] - initial[joint]) * blend;
        if (interrupted || elapsed >= args.duration + 0.25)
            return librocos::MotionFinished(command);
        return command;
    });
}

inline int report(const std::exception& error) {
    std::cerr << "librocos: " << error.what() << '\n';
    if (const auto* control = dynamic_cast<const librocos::ControlException*>(&error)) {
        if (control->status)
            std::cerr << "Controller state: " << control->status->robot_state << '\n';
    }
    return 1;
}
}  // namespace example
