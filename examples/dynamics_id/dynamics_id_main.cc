#include <gflags/gflags.h>

#include <csignal>

#include "excitation_trajectory.hpp"

DEFINE_string(hardware, "config/robots/talon/hardware.yaml", "Hardware YAML (no Robot)");
DEFINE_int32(id, 0, "EtherCAT shared-memory instance ID");
DEFINE_string(trajectory, "trajectory.json", "Studio trajectory.json or adjacent trajectory.csv");
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
    size_t count = 0, width;
    std::vector<double> values;
    Samples(size_t capacity, size_t joints) : width(2 + 3 * joints), values(capacity * width) {}
    void append(double time, int loop, const Feedback& state) {
        require((count + 1) * width <= values.size(), "Sample buffer exhausted");
        double* row = values.data() + count++ * width;
        *row++ = time;
        *row++ = loop;
        for (Eigen::Index j = 0; j < state.q.size(); ++j) {
            *row++ = state.q[j];
            *row++ = state.dq[j];
            *row++ = state.tau[j];
        }
    }
    void save(FILE* file, size_t n) const {
        std::fprintf(file, "time,loop");
        for (size_t j = 1; j <= n; ++j) std::fprintf(file, ",q_%zu,dq_%zu,tau_%zu", j, j, j);
        std::fputc('\n', file);
        for (size_t i = 0; i < count; ++i) {
            for (size_t k = 0; k < width; ++k)
                std::fprintf(file, k ? ",%.17g" : "%.17g", values[i * width + k]);
            std::fputc('\n', file);
        }
        finishOutput(file);
    }
};

void run(Hardware& hardware, const ExcitationTrajectory& trajectory, Samples& samples) {
    DisableGuard guard(hardware);
    bindAndValidate(hardware, trajectory.drives, FLAGS_id, false);
    guard.prepare();
    hardware.SetMode(static_cast<int8_t>(ModeOfOperation::CyclicSynchronousPositionMode));
    CycleClock cycle(hardware, FLAGS_max_cycle_gap);
    Feedback state(trajectory.names.size());
    cycle.wait();
    readFeedback(hardware, trajectory.drives, state);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot must be stationary before enabling");
    JntArray target(trajectory.names.size());
    target.data = state.q;
    JntArray lastSent = target;
    auto sendPosition = [&] {
        validatePositionCommand(target, lastSent, trajectory.drives, trajectory.velocity,
                                hardware.GetDt() * 1e-6);
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

    const Eigen::VectorXd start = state.q;
    const double approach = approachDuration(start, trajectory);
    auto epoch = Clock::now();
    double elapsed = 0;
    do {
        feedback();
        elapsed = std::chrono::duration<double>(Clock::now() - epoch).count();
        const double u = std::clamp(elapsed / approach, 0.0, 1.0);
        const double blend = u * u * u * (10 + u * (-15 + 6 * u));
        target.data = start + blend * (trajectory.centers - start);
        sendPosition();
    } while (elapsed < approach);

    epoch = Clock::now();
    do {
        feedback();
        sendPosition();
    } while (std::chrono::duration<double>(Clock::now() - epoch).count() < 0.2);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot did not settle at trajectory start");

    // 每轮独立推进轨迹时间，CSV 时间跨轮持续递增；记录实际反馈而非参考导数。
    const auto recordingStart = Clock::now();
    for (int loop = 1; loop <= FLAGS_loops; ++loop) {
        const auto loopStart = Clock::now();
        double t = 0;
        do {
            feedback();
            const auto now = Clock::now();
            samples.append(std::chrono::duration<double>(now - recordingStart).count(), loop,
                           state);
            t = std::chrono::duration<double>(now - loopStart).count() * FLAGS_time_scale;
            trajectory.evaluate(t, FLAGS_time_scale, target);
            sendPosition();
        } while (t < trajectory.duration);
    }
    epoch = Clock::now();
    do {
        feedback();
        sendPosition();
    } while (std::chrono::duration<double>(Clock::now() - epoch).count() < 0.2);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot did not settle at trajectory end");
    guard.stop();
}
}  // namespace

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        require(FLAGS_loops > 0 && FLAGS_loops <= 1000, "loops must be in [1,1000]");
        require(std::isfinite(FLAGS_tracking_error) && FLAGS_tracking_error > 0,
                "Invalid tracking_error");
        require(std::isfinite(FLAGS_max_cycle_gap) && FLAGS_max_cycle_gap > 0 &&
                    FLAGS_max_cycle_gap <= 60,
                "Invalid max_cycle_gap");
        const auto config = Hardware::LoadConfigFromYAML(FLAGS_hardware);
        const ExcitationTrajectory trajectory(FLAGS_trajectory, config);
        trajectory.validate(FLAGS_time_scale, 0.001);
        if (FLAGS_check_only) {
            std::cout << "Trajectory and Hardware YAML validated (no hardware connection)\n";
            return 0;
        }

        auto csv = createOutput(FLAGS_output);
        auto metadataFile = createOutput(FLAGS_output + ".meta.yaml");
        YAML::Node metadata;
        metadata["joint_names"] = trajectory.names;
        metadata["urdf_hash"] = trajectory.urdfHash;
        metadata["hardware"] = std::filesystem::absolute(FLAGS_hardware).string();
        metadata["trajectory"] = std::filesystem::absolute(FLAGS_trajectory).string();
        metadata["time_scale"] = FLAGS_time_scale;
        metadata["loops_requested"] = FLAGS_loops;
        metadata["position_unit"] = "rad";
        metadata["velocity_unit"] = "rad/s";
        metadata["torque_unit"] = "Nm (requires calibrated hardware torque_per_unit)";
        metadata["torque_source"] = "Hardware::GetJointTorque; no additional conversion";
        for (const auto& drive : trajectory.drives) {
            YAML::Node joint;
            joint["name"] = drive.joint_name;
            joint["id"] = drive.id;
            joint["cnt_per_unit"] = drive.transform.cnt_per_unit;
            joint["ratio"] = drive.transform.ratio;
            joint["offset_pos_cnt"] = drive.transform.offset_pos_cnt;
            joint["torque_per_unit"] = drive.transform.torque_per_unit;
            metadata["drives"].push_back(joint);
        }
        Hardware hardware(config, FLAGS_id,
                          static_cast<uint32_t>(std::ceil(FLAGS_max_cycle_gap * 1000)));
        const double dt = hardware.GetDt() * 1e-6;
        require(dt > 0 && dt <= FLAGS_max_cycle_gap, "Invalid hardware period");
        trajectory.validate(FLAGS_time_scale, dt);
        const double capacity =
            std::ceil(trajectory.duration * FLAGS_loops / FLAGS_time_scale / dt * 1.5) +
            FLAGS_loops;
        require(std::isfinite(capacity) && capacity * trajectory.names.size() <= 2000000,
                "Recording too large; reduce loops/duration or increase time_scale");
        Samples samples(static_cast<size_t>(capacity), trajectory.names.size());
        std::string failure;
        try {
            run(hardware, trajectory, samples);
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
    } catch (const std::exception& error) {
        std::cerr << "Dynamics identification acquisition: " << error.what() << '\n';
        return 1;
    }
}
