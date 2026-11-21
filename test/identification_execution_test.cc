#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <sys/wait.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include "src/hardware.hpp"
#include "test/doctest.h"

namespace {
struct OfflineMaster {
    static constexpr double simulatedCstSpeed = 0.1;
    static int nextId() {
        static int id = 600000 + getpid() * 10;
        return id++;
    }
    const int id = nextId();
    rocos::SharedMemoryConfig master{id};
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / ("rocos-identification-" + std::to_string(id));
    bool enabled = false, positionMoved = false, torqueWritten = false;
    std::vector<int32_t> targetsBeforeGap;
    bool cspEnabled = false, disabledBeforeCst = false, cstRequested = false;
    double secondsBeforeCst = 0;
    int16_t torqueBeforeCst = 0, firstCstTorque = 0;

    OfflineMaster() {
        REQUIRE(std::filesystem::create_directory(directory));
        REQUIRE(master.createSharedMemory());
        REQUIRE(master.createPdDataMemoryProvider(32, 32));
        auto& bus = *master.ecatBus;
        bus.slave_num = 2;
        bus.dt = 1000;
        bus.current_state = 8;
        for (int i = 0; i < 2; ++i) {
            auto& slave = bus.slaves[i];
            slave.id = i;
            slave.input_var_num = 5;
            slave.output_var_num = 4;
            descriptor(slave.input_vars[0], "status", i * 16, 2);
            descriptor(slave.input_vars[1], "position", i * 16 + 2, 4);
            descriptor(slave.input_vars[2], "velocity", i * 16 + 6, 4);
            descriptor(slave.input_vars[3], "torque", i * 16 + 10, 2);
            descriptor(slave.input_vars[4], "Mode of operation display", i * 16 + 12, 1);
            descriptor(slave.output_vars[0], "control", i * 16, 2);
            descriptor(slave.output_vars[1], "mode", i * 16 + 2, 1);
            descriptor(slave.output_vars[2], "target", i * 16 + 3, 4);
            descriptor(slave.output_vars[3], "target_torque", i * 16 + 7, 2);
            master.setSlaveInputVarValue<uint16_t>(i, 0, 0x40);
            master.setSlaveInputVarValue<int32_t>(i, 1, i ? -500 : 1000);
            master.setSlaveInputVarValue<int32_t>(i, 2, 0);
            master.setSlaveInputVarValue<int16_t>(i, 3, i ? -100 : 50);
        }
        REQUIRE(master.publishReady());
    }
    ~OfflineMaster() { std::filesystem::remove_all(directory); }
    static void descriptor(rocos::PdVar& v, const char* name, int offset, int size) {
        std::strcpy(v.name, name);
        v.offset = offset;
        v.size = size;
    }
    int run(bool impedance, bool loseCycles = false, bool delayPositionCycle = false,
            bool rejectCst = false) {
        const std::string output = (directory / "measurements.csv").string();
        std::vector<std::string> args;
        if (impedance) {
            args = {IMPEDANCE_EXECUTABLE, "--config=" IDENTIFICATION_TEST_DATA "/controller.yaml",
                    "--duration=.08"};
        } else {
            args = {ACQUISITION_EXECUTABLE, "--hardware=" IDENTIFICATION_TEST_DATA "/hardware.yaml",
                    "--trajectory=" IDENTIFICATION_TEST_DATA "/trajectory.json", "--loops=1"};
        }
        args.push_back("--id=" + std::to_string(id));
        args.push_back("--output=" + output);
        args.push_back("--max_cycle_gap=.2");
        std::vector<char*> argv;
        for (auto& arg : args) argv.push_back(arg.data());
        argv.push_back(nullptr);
        std::fflush(nullptr);
        const pid_t child = fork();
        REQUIRE(child >= 0);
        if (child == 0) {
            if (!std::freopen((directory / "process.log").c_str(), "w", stdout)) _exit(126);
            dup2(fileno(stdout), STDERR_FILENO);
            execv(argv[0], argv.data());
            _exit(127);
        }
        const auto start = std::chrono::steady_clock::now();
        auto enabledAt = start;
        int status = 0, activeCycles = 0;
        pid_t finished = 0;
        while ((finished = waitpid(child, &status, WNOHANG)) == 0) {
            for (int i = 0; i < 2; ++i) {
                const auto control = master.getSlaveOutputVarValue<uint16_t>(i, 0);
                uint16_t driveStatus = 0x40;
                if ((control & 0x8f) == 0x06) driveStatus = 0x21;
                if ((control & 0x8f) == 0x07) driveStatus = 0x23;
                if ((control & 0x8f) == 0x0f) driveStatus = 0x27;
                const auto requestedMode = master.getSlaveOutputVarValue<int8_t>(i, 1);
                const auto mode = rejectCst && requestedMode == 10 ? int8_t{8} : requestedMode;
                master.setSlaveInputVarValue<int8_t>(i, 4, mode);
                const auto torque = master.getSlaveOutputVarValue<int16_t>(i, 3);
                if (impedance && i == 0 && !cstRequested) {
                    if (driveStatus == 0x27 && requestedMode == 8) {
                        cspEnabled = true;
                        torqueBeforeCst = torque;
                    }
                    if (cspEnabled && driveStatus != 0x27) disabledBeforeCst = true;
                    if (driveStatus == 0x27 && requestedMode == 10) {
                        cstRequested = true;
                        secondsBeforeCst = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - enabledAt).count();
                        firstCstTorque = torque;
                    }
                }
                const bool followsTorque = driveStatus == 0x27 && mode == 10;
                master.setSlaveInputVarValue<uint16_t>(i, 0,
                                                        driveStatus | (followsTorque ? 0x1000 : 0));
                if (impedance && i == 1) {
                    // joint_2 has 50000 counts/rad after the hardware ratio.
                    master.setSlaveInputVarValue<int32_t>(i, 2,
                        followsTorque ? static_cast<int32_t>(simulatedCstSpeed * 50000) : 0);
                }
                if (driveStatus == 0x27) {
                    if (!enabled) enabledAt = std::chrono::steady_clock::now();
                    enabled = true;
                    if (mode == 8) {
                        const auto target = master.getSlaveOutputVarValue<int32_t>(i, 2);
                        positionMoved |= target != (i ? -500 : 1000);
                        master.setSlaveInputVarValue<int32_t>(i, 1, target);
                    }
                    if (mode == 10) torqueWritten |= torque != 0;
                }
            }
            if (enabled) ++activeCycles;
            if (delayPositionCycle && activeCycles == 200) {
                // 小于 max_cycle_gap，但墙钟轨迹推进将产生超速步进。
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
                for (int i = 0; i < 2; ++i)
                    targetsBeforeGap.push_back(master.getSlaveOutputVarValue<int32_t>(i, 2));
            }
            if (!loseCycles || activeCycles < 10) {
                rocos::atomicStore(master.ecatBus->timestamp, static_cast<long>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count()));
                REQUIRE(master.notifyClients());
            }
            if (std::chrono::steady_clock::now() - start > std::chrono::seconds(12)) {
                kill(child, SIGKILL);
                waitpid(child, &status, 0);
                FAIL("Experiment failed to exit within offline timeout");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::ifstream log(directory / "process.log");
        const std::string message{std::istreambuf_iterator<char>(log), {}};
        INFO(message);
        REQUIRE(finished == child);
        REQUIRE(WIFEXITED(status));
        for (int i = 0; i < 2; ++i) CHECK(master.getSlaveInputVarValue<uint16_t>(i, 0) == 0x40);
        return WEXITSTATUS(status);
    }
};
}  // namespace

TEST_CASE_FIXTURE(OfflineMaster, "Hardware-only acquisition writes measured Nm in model order") {
    REQUIRE(run(false) == 0);
    CHECK(enabled);
    CHECK(positionMoved);
    const auto meta = YAML::LoadFile((directory / "measurements.csv.meta.yaml").string());
    CHECK(meta["completed"].as<bool>());
    CHECK(meta["samples"].as<int>() > 100);
    std::ifstream csv(directory / "measurements.csv");
    std::string line;
    std::getline(csv, line);
    CHECK(line == "time,loop,q_1,dq_1,tau_1,q_2,dq_2,tau_2");
    double previous = -1;
    while (std::getline(csv, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream input(line);
        double t, loop, q1, v1, tau1, q2, v2, tau2;
        REQUIRE(bool(input >> t >> loop >> q1 >> v1 >> tau1 >> q2 >> v2 >> tau2));
        CHECK(t > previous);
        CHECK(tau1 == doctest::Approx(5));
        CHECK(tau2 == doctest::Approx(-5));
        REQUIRE(v1 == 0);
        REQUIRE(v2 == 0);  // 非零位置零点不影响速度。
        previous = t;
    }
}

TEST_CASE_FIXTURE(OfflineMaster, "Impedance preloads gravity in CSP and applies configured friction in CST") {
    REQUIRE(run(true) == 0);
    CHECK(enabled);
    CHECK(cspEnabled);
    CHECK(cstRequested);
    CHECK_FALSE(disabledBeforeCst);
    CHECK(secondsBeforeCst >= 5.0);
    CHECK(torqueBeforeCst == firstCstTorque);
    CHECK(firstCstTorque != 50);  // CSP feedback is recorded, not copied to the CST target.
    CHECK(torqueWritten);
    std::ifstream csv(directory / "measurements.csv");
    std::string line;
    std::getline(csv, line);
    CHECK(line.find("csp_hold_tau_1,csp_hold_tau_2") != std::string::npos);
    bool checked = false, checkedFriction = false;
    while (std::getline(csv, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream input(line);
        std::vector<double> row;
        double value;
        while (input >> value) row.push_back(value);
        REQUIRE(row.size() == 22);
        CHECK(row[20] == doctest::Approx(5.0));
        CHECK(row[21] == doctest::Approx(-5.0));
        if (row[3] == 1) {
            CHECK(row[9] == 0);
            CHECK(firstCstTorque == static_cast<int16_t>(row[8] * 10));
            CHECK(master.getSlaveOutputVarValue<int16_t>(0, 3) ==
                  static_cast<int16_t>(row[11] * 10));
            if (row[13] == doctest::Approx(simulatedCstSpeed)) {
                const double v = row[13];
                const double expected = 0.7 * ((0.2 + 0.03 * std::abs(row[16]) +
                    0.1 * std::exp(-std::pow(v / 0.15, 2))) * std::tanh(v / 0.05) + 0.08 * v);
                CHECK(row[17] == doctest::Approx(expected));
                checkedFriction = true;
            }
            checked = true;
        }
    }
    CHECK(checked);
    CHECK(checkedFriction);
}

TEST_CASE_FIXTURE(OfflineMaster, "Impedance aborts if CST mode feedback never acknowledges") {
    CHECK(run(true, false, false, true) == 1);
    CHECK(cspEnabled);
    CHECK(cstRequested);
    CHECK_FALSE(torqueWritten);
    std::ifstream log(directory / "process.log");
    const std::string message{std::istreambuf_iterator<char>(log), {}};
    CHECK(message.find("CST mode or target torque following was not confirmed") != std::string::npos);
}

TEST_CASE_FIXTURE(OfflineMaster,
                  "Position step violation rejects PDO writes and disables acquisition") {
    // joint_1 从 0.5 rad 过渡到中心，避开零位静止过渡。
    master.setSlaveInputVarValue<int32_t>(0, 1, 101000);
    REQUIRE(run(false, false, true) == 1);
    CHECK(enabled);
    REQUIRE(targetsBeforeGap.size() == 2);
    for (int i = 0; i < 2; ++i)
        CHECK(master.getSlaveOutputVarValue<int32_t>(i, 2) == targetsBeforeGap[i]);
    const auto meta = YAML::LoadFile((directory / "measurements.csv.meta.yaml").string());
    CHECK_FALSE(meta["completed"].as<bool>());
    CHECK(meta["failure"].as<std::string>().find("Command velocity limit exceeded: joint_1") !=
          std::string::npos);
    CHECK(std::filesystem::exists(directory / "measurements.csv"));
}

TEST_CASE_FIXTURE(OfflineMaster, "Missing feedback PDO is rejected before enabling") {
    std::strcpy(master.ecatBus->slaves[0].input_vars[2].name, "missing_velocity");
    CHECK(run(false) == 1);
    CHECK_FALSE(enabled);
}

TEST_CASE_FIXTURE(OfflineMaster,
                  "Lost cycle notifications disable drives and preserve failure status") {
    CHECK(run(false, true) == 1);
    CHECK(enabled);
    const auto meta = YAML::LoadFile((directory / "measurements.csv.meta.yaml").string());
    CHECK_FALSE(meta["completed"].as<bool>());
    CHECK_FALSE(meta["failure"].as<std::string>().empty());
}
