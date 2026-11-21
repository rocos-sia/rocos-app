#include <gflags/gflags.h>

#include <csignal>
#include <type_traits>

#include "excitation_trajectory.hpp"
#include "friction_sweep_trajectory.hpp"

DEFINE_string(hardware, "config/robots/talon/hardware.yaml", "Hardware YAML (no Robot)");
DEFINE_int32(id, 0, "EtherCAT shared-memory instance ID");
DEFINE_string(trajectory, "trajectory.json", "Studio trajectory.json or adjacent trajectory.csv");
DEFINE_string(friction_protocol, "", "Studio friction_protocol.json for single-joint friction acquisition");
DEFINE_string(output, "measurements.csv", "New measured q/dq/tau CSV; never overwritten");
DEFINE_int32(loops, 4, "Excitation repetitions");
DEFINE_double(time_scale, 1.0, "Trajectory speed factor in (0,1]");
DEFINE_double(max_cycle_gap, 0.020, "Maximum control cycle interval, seconds");
DEFINE_double(tracking_error, 0.2, "Maximum measured position error to last command, rad");
DEFINE_bool(check_only, false, "Validate files without connecting hardware");

namespace {
using namespace rocos;
using namespace rocos::experiment;
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
void checkSignal() { require(!interrupted, "Interrupted by signal"); }

struct Samples {
    bool friction;
    size_t count = 0, width;
    std::vector<double> values;
    Samples(size_t capacity, size_t joints, bool recordSweeps)
        : friction(recordSweeps), width((friction ? 7 : 2) + 3 * joints),
          values(capacity * width) {}
    void append(double time, int loop, const Feedback& state,
                const SweepReference& reference = {}) {
        require((count + 1) * width <= values.size(), "Sample buffer exhausted");
        double* row = values.data() + count++ * width;
        *row++ = time;
        if (friction) {
            *row++ = reference.time;
            *row++ = reference.repeat;
            *row++ = reference.pair;
            *row++ = reference.sweep;
            *row++ = static_cast<int>(reference.phase);
            *row++ = reference.velocity;
        } else {
            *row++ = loop;
        }
        for (Eigen::Index j = 0; j < state.q.size(); ++j) {
            *row++ = state.q[j];
            *row++ = state.dq[j];
            *row++ = state.tau[j];
        }
    }
    void save(FILE* file, size_t n) const {
        std::fputs(friction ? "time,reference_time,repeat,pair,sweep,phase,target_velocity"
                           : "time,loop", file);
        for (size_t j = 1; j <= n; ++j) std::fprintf(file, ",q_%zu,dq_%zu,tau_%zu", j, j, j);
        std::fputc('\n', file);
        for (size_t i = 0; i < count; ++i) {
            const double* row = values.data() + i * width;
            for (size_t k = 0; k < width; ++k) {
                if (friction && k == 5) {
                    const char* phases[] = {"ramp", "steady", "dwell"};
                    std::fprintf(file, ",%s", phases[static_cast<size_t>(row[k])]);
                } else {
                    std::fprintf(file, k ? ",%.17g" : "%.17g", row[k]);
                }
            }
            std::fputc('\n', file);
        }
        finishOutput(file);
    }
};

template <typename Trajectory>
void run(Hardware& hardware, const Trajectory& trajectory, double dt, Samples& samples) {
    constexpr bool friction = std::is_same_v<Trajectory, FrictionSweepTrajectory>;
    bindAndValidate(hardware, trajectory.drives, FLAGS_id, false);
    CycleClock cycle(hardware, FLAGS_max_cycle_gap);
    Feedback state(trajectory.names.size());
    if constexpr (friction) {
        cycle.wait();
        checkSignal();
        for (const auto& drive : trajectory.drives)
            require(hardware.GetJointState(drive.id) == JntState::DISABLED,
                    "Stop other controllers and disable joint before friction acquisition: " +
                        drive.joint_name);
        readFeedback(hardware, trajectory.drives, state);
        require(state.dq.cwiseAbs().maxCoeff() <= 0.02,
                "Robot must be stationary before friction approach");
    }
    DisableGuard guard(hardware, trajectory.drives);
    guard.prepare();
    hardware.SetMode(static_cast<int8_t>(ModeOfOperation::CyclicSynchronousPositionMode));
    cycle.reset();
    cycle.wait();
    readFeedback(hardware, trajectory.drives, state);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot must be stationary before enabling");
    JntArray target(trajectory.names.size());
    target.data = state.q;
    JntArray lastSent = target;
    auto sendPosition = [&] {
        validatePositionCommand(target, lastSent, trajectory.drives, dt);
        hardware.SetPosition(target);
        lastSent = target;
    };
    sendPosition();
    cycle.wait();
    checkSignal();
    hardware.SetEnabled();
    checkEnabled(hardware);
    cycle.reset();

    auto feedback = [&] {
        cycle.wait();
        checkSignal();
        checkEnabled(hardware);
        readFeedback(hardware, trajectory.drives, state);
        require((state.q - target.data).cwiseAbs().maxCoeff() <= FLAGS_tracking_error,
                "Position tracking error exceeded");
    };

    const Eigen::VectorXd start = lastSent.data;
    const Eigen::VectorXd displacement = trajectory.centers - start;
    const auto progress = [&] {
        if constexpr (friction) {
            const FrictionApproach approach(displacement, trajectory.drives, dt);
            return [approach](double time) { return approach.progress(time); };
        } else {
            const double duration = approachDuration(start, trajectory);
            return [duration](double time) {
                const double u = std::min(time / duration, 1.0);
                return u * u * u * (10 + u * (-15 + 6 * u));
            };
        }
    }();
    for (size_t step = 1; ; ++step) {
        feedback();
        const double blend = progress(step * dt);
        target.data = blend >= 1 ? trajectory.centers : (start + blend * displacement).eval();
        sendPosition();
        if (blend >= 1) break;
    }

    constexpr double startingHoldSeconds = friction ? 2.0 : 0.2;
    auto epoch = Clock::now();
    do {
        feedback();
        sendPosition();
    } while (std::chrono::duration<double>(Clock::now() - epoch).count() < startingHoldSeconds);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot did not settle at trajectory start");
    if constexpr (friction)
        require((state.q - trajectory.centers).cwiseAbs().maxCoeff() <= 0.02,
                "Robot did not reach friction initial_pose");

    // 每个主站通知推进一个轨迹步。CSV 时间与反馈仍取实测值。
    const auto recordingStart = Clock::now();
    const auto elapsed = [&] {
        return std::chrono::duration<double>(Clock::now() - recordingStart).count();
    };
    if constexpr (friction) {
        SweepReference sent = trajectory.evaluate(0, target);
        const double samplePeriod = std::max(dt, 1 / trajectory.sampleRate);
        for (size_t step = 1; ; ++step) {
            feedback();
            // 刚收到的反馈对应上一条命令；实验标记始终随该命令保存。
            if (sent.time + dt * 1e-9 >= samples.count * samplePeriod)
                samples.append(elapsed(), 0, state, sent);
            const double t = std::min(trajectory.duration, step * dt);
            sent = trajectory.evaluate(t, target);
            sendPosition();
            if (t >= trajectory.duration) break;
        }
    } else {
        const double trajectoryStep = dt * FLAGS_time_scale;
        for (int loop = 1; loop <= FLAGS_loops; ++loop) {
            for (size_t step = 1; ; ++step) {
                feedback();
                samples.append(elapsed(), loop, state);
                const double t = std::min(trajectory.duration, step * trajectoryStep);
                trajectory.evaluate(t, FLAGS_time_scale, target);
                sendPosition();
                if (t >= trajectory.duration) break;
            }
        }
    }
    epoch = Clock::now();
    do {
        feedback();
        sendPosition();
    } while (std::chrono::duration<double>(Clock::now() - epoch).count() < 0.2);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot did not settle at trajectory end");
    guard.stop();
}
template <typename Trajectory>
int acquire(const HardwareConfig& config, const std::string& path) {
    constexpr bool friction = std::is_same_v<Trajectory, FrictionSweepTrajectory>;
    const Trajectory trajectory(path, config);
    if constexpr (!friction) trajectory.validate(FLAGS_time_scale, 0.001);
    if (FLAGS_check_only) {
        std::cout << (friction ? "Friction protocol" : "Trajectory")
                  << " and Hardware YAML validated (no hardware connection)\n";
        if constexpr (friction)
            std::cout << "Joint: " << trajectory.names[trajectory.joint]
                      << "; duration: " << trajectory.duration << " s; initial_pose [rad]: "
                      << trajectory.centers.transpose() << '\n';
        return 0;
    }

    auto csv = createOutput(FLAGS_output);
    auto metadataFile = createOutput(FLAGS_output + ".meta.yaml");
    YAML::Node metadata;
    metadata["joint_names"] = trajectory.names;
    metadata["urdf_hash"] = trajectory.urdfHash;
    metadata["hardware"] = std::filesystem::absolute(FLAGS_hardware).string();
    metadata[friction ? "friction_protocol" : "trajectory"] =
        std::filesystem::absolute(path).string();
    if constexpr (friction) {
        metadata["joint"] = trajectory.names[trajectory.joint];
        metadata["joint_index"] = trajectory.joint;
        metadata["repeats_requested"] = trajectory.repeats;
        metadata["reference_duration_seconds"] = trajectory.duration;
        metadata["requested_sample_rate_hz"] = trajectory.sampleRate;
        metadata["approach_speed_limit_rad_s"] = kFrictionApproachSpeed;
    } else {
        metadata["time_scale"] = FLAGS_time_scale;
        metadata["loops_requested"] = FLAGS_loops;
    }
    metadata["position_unit"] = "rad";
    metadata["velocity_unit"] = "rad/s";
    metadata["torque_unit"] = "Nm (requires calibrated hardware torque_per_unit)";
    metadata["torque_source"] = "Hardware::GetJointTorque; no additional conversion";
    for (const auto& drive : trajectory.drives) {
        YAML::Node joint;
        joint["name"] = drive.joint_name;
        joint["id"] = drive.id;
        joint["pos_per_unit"] = drive.transform.pos_per_unit;
        joint["vel_per_unit"] = drive.transform.vel_per_unit;
        joint["load_torque_per_unit"] = drive.transform.load_torque_per_unit;
        joint["ratio"] = drive.transform.ratio;
        joint["offset_pos_cnt"] = drive.transform.offset_pos_cnt;
        joint["torque_per_unit"] = drive.transform.torque_per_unit;
        metadata["drives"].push_back(joint);
    }
    Hardware hardware(config, FLAGS_id,
                      static_cast<uint32_t>(std::ceil(FLAGS_max_cycle_gap * 1000)));
    const double dt = hardware.GetDt() * 1e-6;
    require(dt > 0 && dt <= FLAGS_max_cycle_gap, "Invalid hardware period");
    double capacity;
    if constexpr (friction) {
        const double samplePeriod = std::max(dt, 1 / trajectory.sampleRate);
        metadata["nominal_record_period_seconds"] = samplePeriod;
        capacity = std::ceil(trajectory.duration / samplePeriod) + 2;
    } else {
        trajectory.validate(FLAGS_time_scale, dt);
        capacity = std::ceil(trajectory.duration * FLAGS_loops / FLAGS_time_scale / dt * 1.5) +
                   FLAGS_loops;
    }
    require(std::isfinite(capacity) && capacity * trajectory.names.size() <= 2000000,
            friction ? "Friction recording too large; reduce the protocol duration or sample rate"
                     : "Recording too large; reduce loops/duration or increase time_scale");
    Samples samples(static_cast<size_t>(capacity), trajectory.names.size(), friction);
    std::string failure;
    try {
        run(hardware, trajectory, dt, samples);
    } catch (const std::exception& error) {
        failure = error.what();
    }
    // run 的 guard 已先下使能，再进行文件 I/O。
    samples.save(csv.get(), trajectory.names.size());
    metadata["period_seconds"] = dt;
    metadata["samples"] = samples.count;
    metadata["completed"] = failure.empty();
    metadata["failure"] = failure;
    YAML::Emitter emitter;
    emitter << metadata;
    require(emitter.good(), "Cannot serialize metadata");
    std::fprintf(metadataFile.get(), "%s\n", emitter.c_str());
    finishOutput(metadataFile.get());
    if (!failure.empty()) throw std::runtime_error(failure);
    std::cout << "Saved " << samples.count << " measured samples to " << FLAGS_output << '\n';
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        require(std::isfinite(FLAGS_tracking_error) && FLAGS_tracking_error > 0,
                "Invalid tracking_error");
        require(std::isfinite(FLAGS_max_cycle_gap) && FLAGS_max_cycle_gap > 0 &&
                    FLAGS_max_cycle_gap <= 60,
                "Invalid max_cycle_gap");
        const auto config = Hardware::LoadConfigFromYAML(FLAGS_hardware);
        if (!FLAGS_friction_protocol.empty()) {
            for (const char* flag : {"trajectory", "loops", "time_scale"})
                require(gflags::GetCommandLineFlagInfoOrDie(flag).is_default,
                        std::string("--") + flag + " cannot be used with --friction_protocol");
            return acquire<FrictionSweepTrajectory>(config, FLAGS_friction_protocol);
        }
        require(FLAGS_loops > 0 && FLAGS_loops <= 1000, "loops must be in [1,1000]");
        return acquire<ExcitationTrajectory>(config, FLAGS_trajectory);
    } catch (const std::exception& error) {
        std::cerr << "Dynamics identification acquisition: " << error.what() << '\n';
        return 1;
    }
}
