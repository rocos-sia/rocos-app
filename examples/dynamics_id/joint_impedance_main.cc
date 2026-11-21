#include <gflags/gflags.h>

#include <csignal>

#include "experiment_common.hpp"
#include "identified_joint_impedance.hpp"

DEFINE_string(config, "config/identified_impedance.yaml", "Identified impedance YAML");
DEFINE_int32(id, 0, "EtherCAT shared-memory instance ID");
DEFINE_bool(check_only, false, "Validate configuration/models without connecting hardware");
DEFINE_double(duration, 0, "CST hold duration in seconds after the CSP start; 0 runs until SIGINT/SIGTERM");
DEFINE_double(max_cycle_gap, 0.020, "Maximum control cycle interval, seconds");
DEFINE_string(output, "impedance.csv", "New diagnostic CSV containing the last 512 cycles");

namespace {
using namespace rocos;
using namespace rocos::experiment;
volatile std::sig_atomic_t stopped = 0;
void signalHandler(int) { stopped = 1; }

constexpr auto csp = ModeOfOperation::CyclicSynchronousPositionMode;
constexpr auto cst = ModeOfOperation::CyclicSynchronousTorqueMode;
constexpr auto cspHoldTime = std::chrono::seconds(5);
constexpr auto settleWindow = std::chrono::milliseconds(500);
constexpr auto modeTimeout = std::chrono::milliseconds(250);
constexpr double stationarySpeed = 0.02;  // rad/s
constexpr double stationaryPositionError = 0.02;  // rad
constexpr uint16_t followsTorqueCommand = 1u << 12;

struct ModeFeedback {
    PdoEntry<int8_t> mode;
    PdoEntry<uint16_t> status;

    bool matches(ModeOfOperation expected) const {
        return mode.read() == static_cast<int8_t>(expected) &&
               (expected != cst || (status.read() & followsTorqueCommand));
    }
};

struct History {
    static constexpr size_t capacity = 512;
    size_t count = 0, width;
    std::vector<double> values;
    Eigen::VectorXd cspHoldTorque;
    explicit History(size_t n)
        : width(4 + 8 * n), values(capacity * width, 0), cspHoldTorque(Eigen::VectorXd::Zero(n)) {}
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
        for (size_t j = 1; j <= n; ++j) std::fprintf(file, ",csp_hold_tau_%zu", j);
        std::fputc('\n', file);
        const size_t first = count > capacity ? count - capacity : 0;
        for (size_t i = first; i < count; ++i) {
            const double* row = values.data() + (i % capacity) * width;
            for (size_t k = 0; k < width; ++k) std::fprintf(file, k ? ",%.17g" : "%.17g", row[k]);
            for (size_t j = 0; j < n; ++j) std::fprintf(file, ",%.17g", cspHoldTorque[j]);
            std::fputc('\n', file);
        }
        finishOutput(file);
    }
};

void checkHandoffFeedback(const Feedback& state, const IdentifiedJointImpedanceConfig& config) {
    for (size_t j = 0; j < config.jointNames.size(); ++j) {
        if (std::abs(state.dq[j]) > config.velocityLimit[j])
            throw std::runtime_error("Safety velocity limit exceeded during mode handoff: " +
                                     config.jointNames[j] + ", dq=" + std::to_string(state.dq[j]));
        if (state.q[j] < config.lower[j] || state.q[j] > config.upper[j])
            throw std::runtime_error("Position exceeds joint limits during mode handoff: " +
                                     config.jointNames[j]);
    }
}

void run(Hardware& hardware, IdentifiedJointImpedanceController& controller,
         const std::vector<Drive>& drives, History& history) {
    bindAndValidate(hardware, drives, FLAGS_id, true);
    auto& bus = *EcatConfig::getInstance(FLAGS_id);
    std::vector<ModeFeedback> modeFeedback;
    modeFeedback.reserve(drives.size());
    for (const auto& drive : drives) {
        require(bool(PdoEntry<int32_t>::output(bus, drive.id, drive.outputs.target_position)),
                "Missing CSP target position PDO: " + drive.joint_name);
        auto mode = PdoEntry<int8_t>::input(bus, drive.id, "Mode of operation display");
        require(bool(mode), "Missing mode display PDO (0x6061): " + drive.joint_name);
        modeFeedback.push_back({mode, PdoEntry<uint16_t>::input(bus, drive.id, drive.inputs.status_word)});
    }
    auto allInMode = [&](ModeOfOperation mode) {
        return std::all_of(modeFeedback.begin(), modeFeedback.end(),
                           [mode](const ModeFeedback& feedback) { return feedback.matches(mode); });
    };

    DisableGuard guard(hardware, drives);
    guard.prepare();
    const auto& config = controller.config();
    CycleClock cycle(hardware, FLAGS_max_cycle_gap);
    Feedback state(drives.size());
    cycle.wait();
    readFeedback(hardware, drives, state);
    checkHandoffFeedback(state, config);
    require(state.dq.cwiseAbs().maxCoeff() <= stationarySpeed, "Robot must be stationary");
    JntArray cspTarget(drives.size());
    cspTarget.data = state.q;
    hardware.SetPosition(cspTarget);
    hardware.SetMode(static_cast<int8_t>(csp));
    auto modeDeadline = Clock::now() + modeTimeout;
    do {
        cycle.wait();
        require(!stopped, "Interrupted before enabling");
        require(Clock::now() <= modeDeadline, "CSP mode was not confirmed before enabling");
    } while (!allInMode(csp));
    hardware.SetEnabled();
    checkEnabled(hardware);
    cycle.reset();

    const auto cspStart = Clock::now();
    size_t holdSamples = 0;
    bool settled = true;
    do {
        cycle.wait();
        require(!stopped, "Interrupted during CSP hold");
        checkEnabled(hardware);
        require(allInMode(csp), "Drive left CSP during startup hold");
        readFeedback(hardware, drives, state);
        checkHandoffFeedback(state, config);
        hardware.SetPosition(cspTarget);
        if (Clock::now() - cspStart >= cspHoldTime - settleWindow) {
            settled = settled && state.dq.cwiseAbs().maxCoeff() <= stationarySpeed &&
                      (state.q - cspTarget.data).cwiseAbs().maxCoeff() <= stationaryPositionError;
            history.cspHoldTorque += state.tau;
            ++holdSamples;
        }
    } while (Clock::now() - cspStart < cspHoldTime);
    require(holdSamples > 0 && settled, "Robot did not settle in CSP before torque handoff");
    history.cspHoldTorque /= static_cast<double>(holdSamples);

    const Eigen::VectorXd target = state.q;
    const Eigen::VectorXd zeroVelocity = Eigen::VectorXd::Zero(target.size());
    const Eigen::VectorXd noTemperature;
    JntArray command(drives.size());
    command.data = controller.computeTorque(target, zeroVelocity, target, noTemperature, false);
    require((command.data.cwiseAbs().array() <= config.torqueLimit.array()).all(),
            "Initial compensation exceeds torque limits");
    std::cout << "CSP holding torque and CST model preload (Nm)\n";
    for (size_t j = 0; j < drives.size(); ++j)
        std::cout << drives[j].joint_name << ": " << history.cspHoldTorque[j] << " -> "
                  << command(j) << '\n';

    require(!stopped, "Interrupted before torque handoff");
    hardware.SetTorque(command);
    // 排除计算期间积压的周期通知，先让主站完整发送预置，再请求切模式。
    const auto preloadCycle = bus.getTimestamp();
    modeDeadline = Clock::now() + modeTimeout;
    auto observedCycle = preloadCycle;
    unsigned freshCycles = 0;
    do {
        cycle.wait();
        require(!stopped, "Interrupted before torque handoff");
        checkEnabled(hardware);
        require(allInMode(csp), "Drive left CSP before torque handoff");
        readFeedback(hardware, drives, state);
        checkHandoffFeedback(state, config);
        require(state.dq.cwiseAbs().maxCoeff() <= stationarySpeed &&
                    (state.q - target).cwiseAbs().maxCoeff() <= stationaryPositionError,
                "Robot moved before torque handoff");
        require(Clock::now() <= modeDeadline, "No fresh EtherCAT cycle after torque preload");
        const auto currentCycle = bus.getTimestamp();
        if (currentCycle != observedCycle) {
            observedCycle = currentCycle;
            ++freshCycles;
        }
    } while (freshCycles < 2);

    hardware.SetMode(static_cast<int8_t>(cst));
    modeDeadline = Clock::now() + modeTimeout;
    do {
        cycle.wait();
        require(!stopped, "Interrupted during CST handoff");
        checkEnabled(hardware);
        readFeedback(hardware, drives, state);
        checkHandoffFeedback(state, config);
        require(Clock::now() <= modeDeadline, "CST mode or target torque following was not confirmed");
    } while (!allInMode(cst));

    const auto start = Clock::now();
    double dt = hardware.GetDt() * 1e-6;
    while (!stopped) {
        const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        double* row = history.append(elapsed, dt, state, target, command.data);
        if (FLAGS_duration > 0 && elapsed >= FLAGS_duration) break;
        // 目标更新和模型计算由同一线程执行；默认保持启动姿态。
        const auto& requested = controller.computeTorque(state.q, state.dq, target, noTemperature);
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
        dt = cycle.wait();
        if (stopped) break;
        checkEnabled(hardware);
        require(allInMode(cst), "Drive left CST or stopped following target torque");
        readFeedback(hardware, drives, state);
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
            std::cout << "Startup plan: 5 s CSP hold, pure gravity preload, then CST friction_scale="
                      << config.frictionScale << '\n';
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
        std::cout << "CSP hold (5 s) -> pure gravity preload -> CST friction_scale="
                  << config.frictionScale << '\n';
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
