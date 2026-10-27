#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <sys/wait.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

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

    BusFixture() {
        REQUIRE(master.createSharedMemory());
        REQUIRE(master.createPdDataMemoryProvider(16, 16));
        auto& bus = *master.ecatBus;
        bus.slave_num = 1;
        bus.dt = 1000;
        // 未知主站状态及只有少量 PDO 都是有效布局，不能再用扫描特征推断结构位置。
        bus.current_state = 0;
        auto& slave = bus.slaves[0];
        slave.id = 0;
        std::strcpy(slave.name, "test drive");
        slave.input_var_num = 2;
        slave.output_var_num = 2;
        descriptor(slave.input_vars[0], "status", 1, 2);
        descriptor(slave.input_vars[1], "position", 3, 4);
        descriptor(slave.output_vars[0], "control", 1, 2);
        descriptor(slave.output_vars[1], "target", 3, 4);
        master.setSlaveInputVarValue<uint16_t>(0, 0, 0x40);
        master.setSlaveInputVarValue<int32_t>(0, 1, 123);
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
