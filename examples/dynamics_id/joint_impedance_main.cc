#include <gflags/gflags.h>

#include <csignal>

#include "experiment_common.hpp"
#include "identified_joint_impedance.hpp"

DEFINE_string(config, "config/identified_impedance.yaml", "Identified impedance YAML");
DEFINE_int32(id, 0, "EtherCAT shared-memory instance ID");
DEFINE_bool(check_only, false, "Validate configuration/models without connecting hardware");
DEFINE_double(duration, 0, "Hold duration in seconds; 0 runs until SIGINT/SIGTERM");
DEFINE_double(max_cycle_gap, 0.020, "Maximum control cycle interval, seconds");
DEFINE_string(output, "impedance.csv", "New diagnostic CSV containing the last 512 cycles");

namespace {
using namespace rocos;
using namespace rocos::experiment;
volatile std::sig_atomic_t stopped = 0;
void signalHandler(int) { stopped = 1; }

struct History {
    static constexpr size_t capacity = 512;
    size_t count = 0, width;
    std::vector<double> values;
    explicit History(size_t n) : width(4 + 8 * n), values(capacity * width, 0) {}
    double* append(double time, double dt, const Feedback& state, const Eigen::VectorXd& target,
                   const Eigen::VectorXd& sent) {
        double* row = values.data() + (count++ % capacity) * width;
        std::fill(row, row + width, 0);
        row[0] = time;
        row[1] = dt;
        for (Eigen::Index j = 0; j < state.q.size(); ++j) {
            double* joint = row + 4 + 8 * j;
            joint[0] = state.q[j];
            joint[1] = state.dq[j];
            joint[2] = state.tau[j];
            joint[3] = target[j];
            joint[7] = sent[j];
        }
        return row;
    }
    void save(FILE* file, size_t n) const {
        std::fprintf(file, "time,dt,computed,written");
        for (size_t j = 1; j <= n; ++j)
            std::fprintf(
                file,
                ",q_%zu,dq_%zu,tau_%zu,target_%zu,rigid_%zu,friction_%zu,requested_%zu,sent_%zu", j,
                j, j, j, j, j, j, j);
        std::fputc('\n', file);
        const size_t first = count > capacity ? count - capacity : 0;
        for (size_t i = first; i < count; ++i) {
            const double* row = values.data() + (i % capacity) * width;
            for (size_t k = 0; k < width; ++k) std::fprintf(file, k ? ",%.17g" : "%.17g", row[k]);
            std::fputc('\n', file);
        }
        finishOutput(file);
    }
};

void run(Hardware& hardware, IdentifiedJointImpedanceController& controller,
         const std::vector<Drive>& drives, History& history) {
    DisableGuard guard(hardware);
    bindAndValidate(hardware, drives, FLAGS_id, true);
    guard.prepare();
    const auto& config = controller.config();
    CycleClock cycle(hardware, FLAGS_max_cycle_gap);
    Feedback state(drives.size());
    cycle.wait();
    readFeedback(hardware, drives, state);
    require(state.dq.cwiseAbs().maxCoeff() <= 0.02, "Robot must be stationary");
    const Eigen::VectorXd target = state.q;
    JntArray command(drives.size());
    command.data = controller.computeTorque(state.q, state.dq, target);
    require((command.data.cwiseAbs().array() <= config.torqueLimit.array()).all(),
            "Initial compensation exceeds torque limits");
    hardware.SetMode(static_cast<int8_t>(ModeOfOperation::CyclicSynchronousTorqueMode));
    hardware.SetTorque(command);  // Nm，Hardware 完成唯一一次驱动比例换算。
    cycle.wait();
    require(!stopped, "Interrupted before enabling");
    hardware.SetEnabled();
    checkEnabled(hardware);
    cycle.reset();
    const auto start = Clock::now();
    while (!stopped) {
        const double dt = cycle.wait();
        if (stopped) break;
        checkEnabled(hardware);
        readFeedback(hardware, drives, state);
        const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        double* row = history.append(elapsed, dt, state, target, command.data);
        if (FLAGS_duration > 0 && elapsed >= FLAGS_duration) break;
        // 目标更新和模型计算由同一线程执行；默认保持启动姿态。
        const auto& requested = controller.computeTorque(state.q, state.dq, target);
        row[2] = 1;
        for (size_t j = 0; j < drives.size(); ++j) {
            row[4 + 8 * j + 4] = controller.rigidTorque()[j];
            row[4 + 8 * j + 5] = controller.frictionTorque()[j];
            row[4 + 8 * j + 6] = requested[j];
        }
        limitTorque(requested, config.torqueLimit, config.torqueRateLimit, dt, command.data);
        hardware.SetTorque(command);
        row[3] = 1;  // 表示已写 PDO，不声称驱动器已执行该命令。
        for (size_t j = 0; j < drives.size(); ++j) row[4 + 8 * j + 7] = command(j);
    }
    guard.stop();
}
}  // namespace

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        require(std::isfinite(FLAGS_duration) && FLAGS_duration >= 0, "Invalid duration");
        require(std::isfinite(FLAGS_max_cycle_gap) && FLAGS_max_cycle_gap > 0 &&
                    FLAGS_max_cycle_gap <= 60,
                "Invalid max_cycle_gap");
        IdentifiedJointImpedanceController controller(FLAGS_config);
        const auto& config = controller.config();
        const auto hardwareConfig = Hardware::LoadConfigFromYAML(config.hardware);
        const auto drives = orderedDrives(hardwareConfig, config.jointNames);
        if (FLAGS_check_only) {
            std::cout << "Identified model and Hardware YAML validated (no hardware connection)\n";
            if (config.needsTemperature)
                std::cout << "Model requires measured temperatures; hold executable has no "
                             "temperature source\n";
            return 0;
        }
        require(!config.needsTemperature,
                "This hold executable has no temperature input; supply measured temperatures to "
                "the controller");
        History history(drives.size());
        auto file = createOutput(FLAGS_output);
        std::cout << "CST hold: q rad, dq rad/s, torque Nm; velocity from Hardware\n";
        for (const auto& d : drives)
            std::cout << d.joint_name << " -> drive " << d.id
                      << ", torque_per_unit=" << d.transform.torque_per_unit << '\n';
        std::string failure;
        {
            Hardware hardware(hardwareConfig, FLAGS_id,
                              static_cast<uint32_t>(std::ceil(FLAGS_max_cycle_gap * 1000)));
            try {
                run(hardware, controller, drives, history);
            } catch (const std::exception& error) {
                failure = error.what();
            }
        }
        // 停机后落盘，控制循环只更新固定容量诊断缓存。
        history.save(file.get(), drives.size());
        if (!failure.empty()) throw std::runtime_error(failure);
        std::cout << "Hold stopped; last " << std::min(history.count, History::capacity)
                  << " cycles saved to " << FLAGS_output << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Identified impedance: " << error.what() << '\n';
        return 1;
    }
}
