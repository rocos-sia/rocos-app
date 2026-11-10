#include "common.hpp"

int main(int argc, char** argv) {
    try {
        auto args = example::arguments(argc, argv);
        args.amplitude = 0;
        librocos::Robot robot(args.host, args.options);
        example::prepare(robot, args);
        librocos::JointArray hold{};
        double elapsed = 0, maximum_period = 0, minimum_rate = 1;
        std::size_t callbacks = 0;
        robot.control([&](const librocos::RobotState& state, librocos::Duration period) {
            if (period.toNSec() == 0) hold = state.q_d;
            elapsed += period.toSec();
            maximum_period = std::max(maximum_period, period.toSec());
            if (state.applied_message_id)
                minimum_rate = std::min(minimum_rate, state.control_command_success_rate);
            ++callbacks;
            librocos::JointPositions command(hold, state.dof);
            if (example::interrupted || elapsed >= args.duration)
                return librocos::MotionFinished(command);
            return command;
        });
        std::cout << "Callbacks=" << callbacks << ", elapsed=" << elapsed
                  << " s, maximum sample interval=" << maximum_period * 1000
                  << " ms, minimum command reception ratio=" << minimum_rate << '\n';
        return 0;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
