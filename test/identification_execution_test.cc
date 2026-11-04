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
    static int nextId() {
        static int id = 600000 + getpid() * 10;
        return id++;
    }
    const int id = nextId();
    rocos::SharedMemoryConfig master{id};
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / ("rocos-identification-" + std::to_string(id));
    bool enabled = false, positionMoved = false, torqueWritten = false;

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
            slave.input_var_num = slave.output_var_num = 4;
            descriptor(slave.input_vars[0], "status", i * 16, 2);
            descriptor(slave.input_vars[1], "position", i * 16 + 2, 4);
            descriptor(slave.input_vars[2], "velocity", i * 16 + 6, 4);
            descriptor(slave.input_vars[3], "torque", i * 16 + 10, 2);
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
    int run(bool impedance, bool loseCycles = false) {
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
        int status = 0, activeCycles = 0;
        pid_t finished = 0;
        while ((finished = waitpid(child, &status, WNOHANG)) == 0) {
            for (int i = 0; i < 2; ++i) {
                const auto control = master.getSlaveOutputVarValue<uint16_t>(i, 0);
                uint16_t driveStatus = 0x40;
                if ((control & 0x8f) == 0x06) driveStatus = 0x21;
                if ((control & 0x8f) == 0x07) driveStatus = 0x23;
                if ((control & 0x8f) == 0x0f) driveStatus = 0x27;
                master.setSlaveInputVarValue<uint16_t>(i, 0, driveStatus);
                if (driveStatus == 0x27) {
                    enabled = true;
                    const auto mode = master.getSlaveOutputVarValue<int8_t>(i, 1);
                    if (mode == 8) {
                        const auto target = master.getSlaveOutputVarValue<int32_t>(i, 2);
                        positionMoved |= target != (i ? -500 : 1000);
                        master.setSlaveInputVarValue<int32_t>(i, 1, target);
                    }
                    if (mode == 10)
                        torqueWritten |= master.getSlaveOutputVarValue<int16_t>(i, 3) != 0;
                }
            }
            if (enabled) ++activeCycles;
            if (!loseCycles || activeCycles < 10) REQUIRE(master.notifyClients());
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

TEST_CASE_FIXTURE(OfflineMaster, "Hardware-only impedance preloads and writes CST torque") {
    REQUIRE(run(true) == 0);
    CHECK(enabled);
    CHECK(torqueWritten);
    std::ifstream csv(directory / "measurements.csv");
    std::string line;
    std::getline(csv, line);
    bool checked = false;
    while (std::getline(csv, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream input(line);
        std::vector<double> row;
        double value;
        while (input >> value) row.push_back(value);
        REQUIRE(row.size() == 20);
        if (row[3] == 1) {
            CHECK(master.getSlaveOutputVarValue<int16_t>(0, 3) ==
                  static_cast<int16_t>(row[11] * 10));
            checked = true;
        }
    }
    CHECK(checked);
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
