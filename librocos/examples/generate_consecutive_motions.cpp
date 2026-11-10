#include "common.hpp"

int main(int argc, char** argv) {
    try {
        const auto args = example::arguments(argc, argv);
        librocos::Robot robot(args.host, args.options);
        example::prepare(robot, args);
        const auto origin = robot.readOnce().q;
        for (int motion = 0; motion < 4 && !example::interrupted; ++motion) {
            auto target = origin;
            if (motion % 2 == 0) target[args.joint] += args.amplitude;
            example::pointToPoint(robot, args, 0, &target);
            std::cout << "Completed motion " << motion + 1 << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
