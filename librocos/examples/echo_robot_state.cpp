#include "common.hpp"

int main(int argc, char** argv) {
    try {
        const auto args = example::arguments(argc, argv);
        librocos::Robot robot(args.host, args.options);
        robot.read([](const librocos::RobotState& state) {
            std::cout << state.robot_state << " q=";
            for (std::size_t i = 0; i < state.dof; ++i) std::cout << ' ' << state.q[i];
            std::cout << '\n';
            return !example::interrupted;
        });
        return 0;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
