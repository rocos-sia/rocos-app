#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "src/hardware.hpp"
#include "src/pdo_entry.hpp"
#include "test/doctest.h"

namespace {
int nextBusId() {
    static int next = 100000 + static_cast<int>(getpid()) * 10;
    return next++;
}

struct BusFixture {
    const int id = nextBusId();
    rocos::SharedMemoryConfig master{id};

    explicit BusFixture(int drive_count = 1) {
        REQUIRE(master.createSharedMemory());
        REQUIRE(master.createPdDataMemoryProvider(8 * drive_count + 8, 8 * drive_count + 8));
        auto& bus = *master.ecatBus;
        bus.slave_num = drive_count;
        bus.dt = 1000;
        // 未知主站状态及只有少量 PDO 都是有效布局，不能再用扫描特征推断结构位置。
        bus.current_state = 0;
        for (int i = 0; i < drive_count; ++i) {
            auto& slave = bus.slaves[i];
            slave.id = i;
            std::strcpy(slave.name, "test drive");
            slave.input_var_num = 2;
            slave.output_var_num = 2;
            descriptor(slave.input_vars[0], "status", 8 * i + 1, 2);
            descriptor(slave.input_vars[1], "position", 8 * i + 3, 4);
            descriptor(slave.output_vars[0], "control", 8 * i + 1, 2);
            descriptor(slave.output_vars[1], "target", 8 * i + 3, 4);
            master.setSlaveInputVarValue<uint16_t>(i, 0, 0x40);
            master.setSlaveInputVarValue<int32_t>(i, 1, 123);
        }
        REQUIRE(master.publishReady());
    }

    static void descriptor(rocos::PdVar& var, const char* name, int offset, int size) {
        std::strcpy(var.name, name);
        var.offset = offset;
        var.size = size;
    }
};
}  // namespace

TEST_CASE_FIXTURE(BusFixture, "App PDO entries handle unaligned bytes and reject incorrect types") {
    rocos::SharedMemoryConfig client(id);
    REQUIRE(client.connect());
    CHECK(client.header->abi_version == 2);
    CHECK(reinterpret_cast<char*>(client.ecatBus) - reinterpret_cast<char*>(client.header) ==
          sizeof(rocos::ShmHeader));
    CHECK(client.getBusCurrentState() == 0);
    auto input = rocos::PdoEntry<int32_t>::input(client, 0, "position");
    auto output = rocos::PdoEntry<int32_t>::output(client, 0, "target");
    REQUIRE(static_cast<bool>(input));
    REQUIRE(static_cast<bool>(output));
    CHECK(input.read() == 123);
    output.write(-456);
    CHECK(master.getSlaveOutputVarValue<int32_t>(0, 1) == -456);
    CHECK_THROWS_AS(input.write(7), std::logic_error);
    CHECK_THROWS_AS(rocos::PdoEntry<int16_t>::input(client, 0, "position"), std::out_of_range);
    CHECK_THROWS_AS(rocos::PdoEntry<int32_t>::input(client, 1, "position"), std::out_of_range);
    const auto missing = rocos::PdoEntry<int32_t>::input(client, 0, "absent");
    CHECK_FALSE(static_cast<bool>(missing));
    CHECK_THROWS_AS(missing.read(), std::logic_error);
    rocos::SharedMemoryConfig monitor(id);
    REQUIRE(monitor.connect(true));
    CHECK(rocos::PdoEntry<int32_t>::input(monitor, 0, "position").read() == 123);
    const auto read_only = rocos::PdoEntry<int32_t>::output(monitor, 0, "target");
    CHECK_THROWS_AS(read_only.write(1), std::out_of_range);
}

TEST_CASE_FIXTURE(BusFixture, "Real Hardware connects to a v2 master in another process") {
    REQUIRE(master.notifyClients());
    const pid_t child = fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        // 子进程单独连接映射；_exit 避免析构 fork 继承的主站资源。
        try {
            rocos::HardwareConfig config;
            rocos::Drive drive;
            drive.id = 0;
            drive.inputs.status_word = "status";
            drive.inputs.position_actual_value = "position";
            drive.outputs.control_word = "control";
            drive.outputs.target_position = "target";
            config.drives.push_back(drive);
            rocos::FTSensor sensor;
            sensor.id = 0;
            sensor.inputs.fx = "status";
            config.ft_sensors.push_back(sensor);
            rocos::IO io;
            io.id = 0;
            io.digital_in_channels = io.digital_out_channels = 8;
            io.analog_in_channels = io.analog_out_channels = 1;
            io.inputs.digital_inputs = io.inputs.digital_outputs = "position";
            io.inputs.analog_inputs = "status";
            io.outputs.digital_outputs = "target";
            io.outputs.analog_outputs = "control";
            config.ios.push_back(io);
            bool ok = false;
            {
                rocos::Hardware hardware(config, id);
                auto* client = rocos::EcatConfig::getInstance(id);
                ok = hardware.GetDt() == 1000 && hardware.GetJointPosition(0) == 123 &&
                     client->getSlaveOutputVarValue<int32_t>(0, 1) == 123 &&
                     hardware.GetWrench().force.x() == 64 && hardware.GetDigitalInput(0, 0) &&
                     hardware.GetAnalogInput(0, 0) == 64;
                hardware.SetDigitalOutput(0, 2, true);
                ok = ok && client->getSlaveOutputVarValue<int32_t>(0, 1) == 127;
                hardware.SetAnalogOutput(0, 0, -2);
                hardware.SetJointPosition(0, -789);
            }
            rocos::EcatConfig::getInstance(id)->disconnect();
            _exit(ok ? 0 : 1);
        } catch (const std::exception& error) {
            std::cerr << error.what() << std::endl;
            _exit(2);
        }
    }
    int status = 0;
    REQUIRE(waitpid(child, &status, 0) == child);
    REQUIRE(WIFEXITED(status));
    CHECK(WEXITSTATUS(status) == 0);
    CHECK(master.getSlaveOutputVarValue<int32_t>(0, 1) == -789);
    CHECK(master.getSlaveOutputVarValue<int16_t>(0, 0) == -2);
}

TEST_CASE_FIXTURE(BusFixture, "App compatibility include rejects an incompatible ABI without exiting") {
    master.header->abi_version = 1;
    rocos::SharedMemoryConfig client(id);
    CHECK_FALSE(client.connect());
    CHECK(client.ecatBus == nullptr);
    CHECK_FALSE(client.lastError().empty());
    CHECK_THROWS_AS(rocos::EcatConfig::getInstance(id), std::runtime_error);
    master.header->abi_version = 2;
    CHECK(client.connect());
}

TEST_CASE("Hardware enables all drive state machines together") {
    bool stalled = false;
    SUBCASE("All drives must receive each transition before any drive advances") {}
    SUBCASE("A stalled first drive does not delay the remaining drives") { stalled = true; }

    BusFixture bus(3);
    auto& master = bus.master;
    REQUIRE(master.notifyClients());
    const pid_t child = fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        try {
            rocos::HardwareConfig config;
            // 配置顺序与从站索引不同；整机使能仍须覆盖所有驱动器。
            for (int id : {0, 2, 1}) {
                rocos::Drive drive;
                drive.id = id;
                drive.inputs.status_word = "status";
                drive.outputs.control_word = "control";
                config.drives.push_back(drive);
            }
            bool ok = false;
            {
                rocos::Hardware hardware(config, bus.id);
                hardware.SetJointBinding({1});
                const auto start = std::chrono::steady_clock::now();
                hardware.SetEnabled();
                const auto elapsed = std::chrono::steady_clock::now() - start;
                ok = hardware.GetJointState(1) == rocos::JntState::ENABLED &&
                     hardware.GetJointState(2) == rocos::JntState::ENABLED &&
                     hardware.GetJointState(0) ==
                         (stalled ? rocos::JntState::DISABLED : rocos::JntState::ENABLED) &&
                     elapsed < std::chrono::milliseconds(250);
                if (!stalled) {
                    // 重复使能也要完成状态确认。
                    hardware.SetEnabled();
                    ok = ok && hardware.GetJointState(0) == rocos::JntState::ENABLED;
                }
            }
            rocos::EcatConfig::getInstance(bus.id)->disconnect();
            _exit(ok ? 0 : 1);
        } catch (const std::exception& error) {
            std::cerr << error.what() << std::endl;
            _exit(2);
        }
    }

    const uint16_t commands[] = {0x06, 0x07, 0x0f};
    const uint16_t states[] = {0x21, 0x23, 0x27};
    int stage = 0;
    int status = 0;
    pid_t finished = 0;
    // 只模拟 PDO 状态反馈，不连接真实 EtherCAT 主站或硬件。
    while ((finished = waitpid(child, &status, WNOHANG)) == 0) {
        if (stalled) {
            for (int id : {1, 2}) {
                const auto control = master.getSlaveOutputVarValue<uint16_t>(id, 0);
                for (int i = 0; i < 3; ++i) {
                    if (control == commands[i]) master.setSlaveInputVarValue(id, 0, states[i]);
                }
            }
        } else if (stage < 3) {
            bool allReady = true;
            for (int id = 0; id < 3; ++id) {
                allReady =
                    allReady && master.getSlaveOutputVarValue<uint16_t>(id, 0) == commands[stage];
            }
            if (allReady) {
                for (int id = 0; id < 3; ++id) master.setSlaveInputVarValue(id, 0, states[stage]);
                ++stage;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    REQUIRE(finished == child);
    REQUIRE(WIFEXITED(status));
    CHECK(WEXITSTATUS(status) == 0);
    if (!stalled) CHECK(stage == 3);
}
