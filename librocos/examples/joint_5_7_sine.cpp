#include <optional>
#include <string_view>
#include <thread>
#include <vector>

#include "common.hpp"
#include "joint_5_7_sine_trajectory.hpp"

namespace {
using Trajectory = joint_sine::Joint5To7Sine;
using WallClock = std::chrono::system_clock;
constexpr double kStoppedVelocity = 0.005;
constexpr auto kPeriod = std::chrono::milliseconds(1);
constexpr char kUsage[] =
    "Usage: librocos_joint_5_7_sine [host [http_port [udp_port]]] [--arm left|right] "
    "[--duration seconds] [--start-at unix_seconds]";

void checkStart(const librocos::RobotState& state) {
    if (state.dof != Trajectory::kDof || !state.enabled || state.work_mode != "position" ||
        (state.robot_state != "STOPPED" && state.robot_state != "RCI"))
        throw std::runtime_error("Prepare an enabled seven-axis arm in position mode and STOPPED or RCI");
    for (std::size_t joint = 0; joint < state.dof; ++joint)
        if (std::abs(state.dq[joint]) > kStoppedVelocity)
            throw std::runtime_error("Wait for all joints to settle before starting");
    (void)Trajectory(state.q);
}

WallClock::time_point parseStartTime(const std::string& value) {
    std::size_t consumed = 0;
    const auto seconds = std::stoll(value, &consumed);
    const auto maximum = std::chrono::duration_cast<std::chrono::seconds>(
                             WallClock::time_point::max().time_since_epoch()).count();
    if (consumed != value.size() || seconds <= 0 || seconds > maximum)
        throw std::invalid_argument("--start-at expects an integer Unix timestamp in seconds");
    return WallClock::time_point(std::chrono::seconds(seconds));
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--help") {
            std::cout << kUsage << '\n';
            return 0;
        }
        std::vector<char*> connection_args{argv[0]};
        std::optional<double> requested_duration;
        std::optional<WallClock::time_point> start_at;
        for (int i = 1; i < argc; ++i) {
            if (std::string_view(argv[i]) == "--start-at") {
                if (start_at || ++i == argc)
                    throw std::invalid_argument("Specify --start-at unix_seconds once");
                start_at = parseStartTime(argv[i]);
                continue;
            }
            if (std::string_view(argv[i]) != "--duration") {
                connection_args.push_back(argv[i]);
                continue;
            }
            if (requested_duration || ++i == argc)
                throw std::invalid_argument("Specify --duration seconds once");
            std::size_t consumed = 0;
            const std::string value = argv[i];
            requested_duration = std::stod(value, &consumed);
            if (consumed != value.size())
                throw std::invalid_argument("--duration expects seconds");
        }
        const auto duration = requested_duration.value_or(Trajectory::kDuration);
        (void)Trajectory(librocos::JointArray{}, duration);

        const int count = static_cast<int>(connection_args.size());
        const bool has_arm = count >= 3 && std::string_view(connection_args[count - 2]) == "--arm";
        if (count - (has_arm ? 2 : 0) > 4) throw std::invalid_argument(kUsage);
        auto args = example::arguments(count, connection_args.data());
        args.options.control_period = kPeriod;
        librocos::Robot robot(args.host, args.options);
        checkStart(robot.readOnce());
        if (start_at && *start_at <= WallClock::now())
            throw std::invalid_argument("--start-at must be in the future after startup checks");

        std::cout << "J1-J4 hold. J5-J7: -30 to +30 degrees around joint zero, "
                  << Trajectory::kNominalVelocity << " rad/s nominal peak, "
                  << Trajectory::kMaxVelocity << " rad/s monitored limit, "
                  << duration << " s, following the 1 ms App feedback cycle. ";
        if (start_at) {
            std::cout << "Waiting for shared start time "
                      << std::chrono::duration_cast<std::chrono::seconds>(
                             start_at->time_since_epoch()).count()
                      << ".\n" << std::flush;
            auto now = WallClock::now();
            while (!example::interrupted && now < *start_at) {
                std::this_thread::sleep_until(
                    std::min(*start_at, now + std::chrono::milliseconds(100)));
                now = WallClock::now();
            }
        } else {
            std::cout << "Press Enter to start.\n" << std::flush;
            if (std::cin.get() == std::char_traits<char>::eof())
                throw std::runtime_error("Motion cancelled before start");
        }
        if (example::interrupted)
            throw std::runtime_error("Motion cancelled before start");
        checkStart(robot.readOnce());
        robot.enableRci();

        std::optional<Trajectory> trajectory;
        std::uint64_t start_cycle = 0;
        const double step = std::chrono::duration<double>(kPeriod).count();
        robot.control([&](const librocos::RobotState& state, librocos::Duration) {
            if (!trajectory) {
                trajectory.emplace(state.q_d, duration);
                start_cycle = state.cycle;
            }
            const double elapsed = (state.cycle - start_cycle) * step;
            for (std::size_t joint = Trajectory::kFirstMovingJoint; joint < state.dof; ++joint) {
                if (std::abs(state.dq[joint]) > Trajectory::kMaxVelocity ||
                    std::abs(state.dq_d[joint]) > Trajectory::kMaxVelocity)
                    throw std::runtime_error("J" + std::to_string(joint + 1) +
                                             " exceeded 0.5 rad/s");
            }
            auto command = trajectory->command(elapsed);
            if (example::interrupted || elapsed >= trajectory->duration())
                return librocos::MotionFinished(command);
            return command;
        });
        std::cout << (example::interrupted ? "J5-J7 test stopped.\n" : "J5-J7 sine completed.\n");
        return 0;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
