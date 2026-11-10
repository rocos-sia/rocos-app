#include "common.hpp"

int main(int argc, char** argv) {
    try {
        const auto args = example::arguments(argc, argv);
        librocos::Robot robot(args.host, args.options);
        example::prepare(robot, args);
        librocos::JointArray initial{};
        double elapsed = 0;
        robot.control([&](const librocos::RobotState& state, librocos::Duration period) {
            if (period.toNSec() == 0) initial = state.q_d;
            elapsed += period.toSec();
            const double phase = 2 * std::acos(-1.0) * std::min(elapsed / args.duration, 1.0);
            const double wave = (1 - std::cos(phase)) / 2;
            librocos::JointPositions command(initial, state.dof);
            command.q[args.joint] += args.amplitude * wave * wave;
            if (example::interrupted || elapsed >= args.duration + 0.25)
                return librocos::MotionFinished(command);
            return command;
        });
        std::cout << "Motion finished, state=" << robot.rciStatus().robot_state << '\n';
        return 0;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
