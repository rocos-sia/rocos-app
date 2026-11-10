#include "common.hpp"

int main(int argc, char** argv) {
    try {
        const auto args = example::arguments(argc, argv);
        librocos::Robot robot(args.host, args.options);
        example::prepare(robot, args);
        example::pointToPoint(robot, args, args.amplitude);
        std::cout << "Motion finished, state=" << robot.rciStatus().robot_state << '\n';
        return 0;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
