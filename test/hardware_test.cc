#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <test/doctest.h>

#include "src/hardware.hpp"

namespace {

// 临时 YAML 文件路径（写 /tmp，绝不能覆盖真实配置）
const std::string kTmpConfig = "/tmp/hardware_test_tmp.yaml";

}  // namespace

// ==========================================================================
// 辅助函数
// ==========================================================================

// 离线夹具独立于部署配置目录。
static std::string cfg(const std::string& name) {
    return std::string(ROCOS_HARDWARE_TEST_DATA) + "/" + name;
}

/// @brief 写入临时 YAML 文件，返回路径
static std::string writeTmpYaml(const std::string& content) {
    std::ofstream ofs(kTmpConfig);
    ofs << content;
    ofs.close();
    return kTmpConfig;
}

// ==========================================================================
// 1. 纯驱动器配置测试
// ==========================================================================

TEST_CASE("Hardware YAML - 纯驱动器配置 (6 轴)") {
    auto config = rocos::Hardware::LoadConfigFromYAML(cfg("hardware_driver_config.yaml"));

    SUBCASE("驱动器数量") {
        CHECK(config.drives.size() == 6);
    }
    SUBCASE("无其他类型") {
        CHECK(config.ft_sensors.empty());
        CHECK(config.ios.empty());
    }

    SUBCASE("第 1 关节 (id=1)") {
        REQUIRE(config.drives.size() >= 1);
        const auto& d = config.drives[0];
        CHECK(d.id == 1);
        CHECK(d.joint_name == "joint_1");
        CHECK(d.torque_source == rocos::TorqueSource::LoadTorque);
        CHECK(d.limit.lower == doctest::Approx(-2.967));
        CHECK(d.limit.upper == doctest::Approx(2.967));
        CHECK(d.transform.pos_per_unit == doctest::Approx(200000.0));
        CHECK(d.transform.user_unit_name == "rad");
        CHECK(d.inputs.status_word == "Statusword");
        CHECK(d.inputs.load_torque_value == "Analog Input 1");
        CHECK(d.inputs.digital_inputs == "Digital Inputs");
        CHECK(d.outputs.control_word == "Controlword");
        CHECK(d.outputs.digital_outputs == "Digital Outputs");
    }

    SUBCASE("第 3 关节使用 secondary_position 通道") {
        REQUIRE(config.drives.size() >= 3);
        const auto& d = config.drives[2];
        CHECK(d.id == 3);
        CHECK(d.torque_source == rocos::TorqueSource::SecondaryPosition);
        CHECK(d.transform.offset_pos_cnt == 100);
        CHECK(d.transform.torque_per_unit == doctest::Approx(0.8));
    }

    SUBCASE("第 6 关节极限校验") {
        REQUIRE(config.drives.size() >= 6);
        const auto& d = config.drives[5];
        CHECK(d.id == 6);
        CHECK(d.limit.effort == doctest::Approx(10.0));
        CHECK(d.transform.pos_per_unit == doctest::Approx(50000.0));
    }
}

// ==========================================================================
// 2. 纯六维力传感器配置测试
// ==========================================================================

TEST_CASE("Hardware YAML - 纯六维力传感器配置") {
    auto config = rocos::Hardware::LoadConfigFromYAML(cfg("hardware_ft_config.yaml"));

    SUBCASE("力传感器数量") {
        CHECK(config.ft_sensors.size() == 1);
    }
    SUBCASE("无其他类型") {
        CHECK(config.drives.empty());
        CHECK(config.ios.empty());
    }

    SUBCASE("传感器字段验证") {
        REQUIRE(!config.ft_sensors.empty());
        const auto& ft = config.ft_sensors[0];
        CHECK(ft.id == 10);

        // PDO 变量名
        CHECK(ft.inputs.fx == "Channel 1");
        CHECK(ft.inputs.fy == "Channel 2");
        CHECK(ft.inputs.fz == "Channel 3");
        CHECK(ft.inputs.tx == "Channel 4");
        CHECK(ft.inputs.ty == "Channel 5");
        CHECK(ft.inputs.tz == "Channel 6");

        // 偏置值
        CHECK(ft.offset.force.x()  == doctest::Approx(0.5));
        CHECK(ft.offset.force.y()  == doctest::Approx(-0.3));
        CHECK(ft.offset.force.z()  == doctest::Approx(12.0));
        CHECK(ft.offset.torque.x() == doctest::Approx(0.01));
        CHECK(ft.offset.torque.y() == doctest::Approx(-0.02));
        CHECK(ft.offset.torque.z() == doctest::Approx(0.005));
    }
}

// ==========================================================================
// 3. 纯 IO 模块配置测试
// ==========================================================================

TEST_CASE("Hardware YAML - 纯 IO 模块配置") {
    auto config = rocos::Hardware::LoadConfigFromYAML(cfg("hardware_io_config.yaml"));

    SUBCASE("IO 模块数量") {
        CHECK(config.ios.size() == 1);
    }
    SUBCASE("无其他类型") {
        CHECK(config.drives.empty());
        CHECK(config.ft_sensors.empty());
    }

    SUBCASE("IO 通道数和 PDO 映射") {
        REQUIRE(!config.ios.empty());
        const auto& io = config.ios[0];
        CHECK(io.id == 20);
        CHECK(io.digital_in_channels == 16);
        CHECK(io.digital_out_channels == 16);
        CHECK(io.analog_in_channels == 8);
        CHECK(io.analog_out_channels == 4);

        CHECK(io.inputs.digital_inputs  == "Digital Inputs");
        CHECK(io.inputs.digital_outputs == "Digital Outputs");
        CHECK(io.inputs.analog_inputs   == "Analog Inputs");
        CHECK(io.outputs.digital_outputs == "Digital Outputs");
        CHECK(io.outputs.analog_outputs  == "Analog Outputs");
    }
}

// ==========================================================================
// 4. 六维力 + 驱动器组合配置测试
// ==========================================================================

TEST_CASE("Hardware YAML - 六维力 + 驱动器组合") {
    auto config = rocos::Hardware::LoadConfigFromYAML(
        cfg("hardware_ft_and_driver_config.yaml"));

    SUBCASE("数量校验") {
        CHECK(config.drives.size() == 2);
        CHECK(config.ft_sensors.size() == 1);
        CHECK(config.ios.empty());
    }

    SUBCASE("力传感器在前") {
        REQUIRE(!config.ft_sensors.empty());
        const auto& ft = config.ft_sensors[0];
        CHECK(ft.id == 7);
        // 该配置中无偏置
        CHECK(ft.offset.force.x() == doctest::Approx(0.0));
    }

    SUBCASE("第 1 个驱动器使用 secondary_position") {
        REQUIRE(config.drives.size() >= 1);
        const auto& d = config.drives[0];
        CHECK(d.id == 1);
        CHECK(d.torque_source == rocos::TorqueSource::SecondaryPosition);
        CHECK(d.limit.effort == doctest::Approx(40.0));
    }

    SUBCASE("第 2 个驱动器使用 load_torque") {
        REQUIRE(config.drives.size() >= 2);
        const auto& d = config.drives[1];
        CHECK(d.id == 2);
        CHECK(d.torque_source == rocos::TorqueSource::LoadTorque);
        CHECK(d.transform.offset_pos_cnt == 50);
        CHECK(d.transform.ratio == doctest::Approx(1.2));
    }
}

// ==========================================================================
// 5. 全类型配置测试
// ==========================================================================

TEST_CASE("Hardware YAML - 全类型配置") {
    auto config = rocos::Hardware::LoadConfigFromYAML(cfg("hardware_all_config.yaml"));

    SUBCASE("数量校验") {
        CHECK(config.drives.size() == 2);
        CHECK(config.ft_sensors.size() == 1);
        CHECK(config.ios.size() == 1);
    }

    SUBCASE("查找函数验证") {
        // 这里测试 Hardware 实例的查找方法需要 EcatConfig，
        // 但我们可以手动遍历验证 ID 唯一性
        std::vector<int> drive_ids;
        for (const auto& d : config.drives) drive_ids.push_back(d.id);
        CHECK(drive_ids == std::vector<int>{1, 2});

        REQUIRE(!config.ft_sensors.empty());
        CHECK(config.ft_sensors[0].id == 10);

        REQUIRE(!config.ios.empty());
        CHECK(config.ios[0].id == 20);
    }

    SUBCASE("各类字段完整性抽查") {
        const auto& io = config.ios[0];
        CHECK(io.digital_in_channels == 8);
        CHECK(io.analog_out_channels == 2);

        const auto& ft = config.ft_sensors[0];
        CHECK(ft.inputs.fz == "Channel 3");
    }
}

// ==========================================================================
// 6. 缺失可选字段 → 默认值测试
// ==========================================================================

TEST_CASE("Hardware YAML - 缺失可选字段默认值") {
    std::string yaml = R"(hardware:
  - id: 100
    type: driver
    joint_name: bare_joint
  - id: 200
    type: ft_sensor
  - id: 300
    type: io
)";
    auto config = rocos::Hardware::LoadConfigFromYAML(writeTmpYaml(yaml));

    SUBCASE("Drive 默认值") {
        REQUIRE(config.drives.size() == 1);
        const auto& d = config.drives[0];
        CHECK(d.id == 100);
        CHECK(d.joint_name == "bare_joint");
        // 未指定时取默认值
        CHECK(d.torque_source == rocos::TorqueSource::LoadTorque);
        CHECK(d.limit.lower == doctest::Approx(-2.71));
        CHECK(d.limit.upper == doctest::Approx(2.71));
        CHECK(d.transform.ratio == doctest::Approx(1.0));
        CHECK(d.transform.pos_per_unit == doctest::Approx(1.0));
        CHECK(d.transform.vel_per_unit == doctest::Approx(1.0));
        CHECK(d.transform.torque_per_unit == doctest::Approx(1.0));
        CHECK(d.transform.load_torque_per_unit == doctest::Approx(1.0));
        CHECK(d.transform.user_unit_name == "rad");
        // 未指定 PDO 变量名时为空串
        CHECK(d.inputs.status_word.empty());
        CHECK(d.outputs.control_word.empty());
    }

    SUBCASE("FTSensor 默认值") {
        REQUIRE(config.ft_sensors.size() == 1);
        const auto& ft = config.ft_sensors[0];
        CHECK(ft.id == 200);
        CHECK(ft.inputs.fx.empty());
        CHECK(ft.offset.force.x() == doctest::Approx(0.0));
    }

    SUBCASE("IO 默认值") {
        REQUIRE(config.ios.size() == 1);
        const auto& io = config.ios[0];
        CHECK(io.id == 300);
        CHECK(io.digital_in_channels == 0);
        CHECK(io.digital_out_channels == 0);
        CHECK(io.analog_in_channels == 0);
        CHECK(io.analog_out_channels == 0);
    }
}

// ==========================================================================
// 7. 异常情况测试
// ==========================================================================

TEST_CASE("Hardware YAML - 异常输入") {
    SUBCASE("缺少 hardware 顶层 key 应抛异常") {
        CHECK_THROWS_AS(
            rocos::Hardware::LoadConfigFromYAML(
                writeTmpYaml("other_key:\n  - id: 1\n    type: driver\n")),
            std::runtime_error);
    }

    SUBCASE("空列表正常返回") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml("hardware: []"));
        CHECK(config.drives.empty());
        CHECK(config.ft_sensors.empty());
        CHECK(config.ios.empty());
    }

    SUBCASE("未知 type 跳过") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml(R"(hardware:
  - id: 1
    type: unknown_type
  - id: 2
    type: driver
    joint_name: ok_joint
)"));
        // unknown_type 被跳过，只解析 driver
        CHECK(config.drives.size() == 1);
        CHECK(config.drives[0].id == 2);
    }

    SUBCASE("缺少 type 字段跳过") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml(R"(hardware:
  - id: 1
  - id: 2
    type: io
)"));
        CHECK(config.ios.size() == 1);
        CHECK(config.ios[0].id == 2);
    }
}

// ==========================================================================
// 8. 配置查找辅助函数测试（不需要 EcatConfig）
// ==========================================================================

TEST_CASE("Hardware YAML - 配置查询函数") {
    auto config = rocos::Hardware::LoadConfigFromYAML(
        cfg("hardware_all_config.yaml"));

    SUBCASE("findDriveById - 使用 vector 预分配查找表（PerformanceProfiler 模式）") {
        // 构建 ID → 索引 O(1) 查找表，与 Hardware 内部实现一致
        // 参考 PerformanceProfiler::channel_to_index_：vector 预分配 + -1 哨兵
        int32_t max_id = -1;
        for (const auto& d : config.drives) {
            if (d.id > max_id) max_id = d.id;
        }
        REQUIRE(max_id >= 0);

        std::vector<int32_t> id_to_index(
            static_cast<size_t>(max_id) + 1, -1);
        for (size_t i = 0; i < config.drives.size(); ++i) {
            id_to_index[static_cast<size_t>(config.drives[i].id)] =
                static_cast<int32_t>(i);
        }

        // 查找存在 id=1
        auto idx1 = id_to_index[1];
        REQUIRE(idx1 >= 0);
        const auto& d = config.drives[static_cast<size_t>(idx1)];
        CHECK(d.joint_name == "joint_1");
        CHECK(d.torque_source == rocos::TorqueSource::LoadTorque);

        // 查找不存在的 id：超出 vector 范围 或 哨兵值 -1
        bool not_found = (999 >= static_cast<int32_t>(id_to_index.size()))
                         || id_to_index[999] < 0;
        CHECK(not_found);
    }

    SUBCASE("findDriveByName - 使用 unordered_map 查找（string key 不变）") {
        std::unordered_map<std::string, size_t> name_to_index;
        for (size_t i = 0; i < config.drives.size(); ++i) {
            name_to_index[config.drives[i].joint_name] = i;
        }
        auto it = name_to_index.find("joint_2");
        REQUIRE(it != name_to_index.end());
        CHECK(config.drives[it->second].id == 2);

        // 查找不存在的名称，直接返回
        auto it_missing = name_to_index.find("nonexistent");
        CHECK(it_missing == name_to_index.end());
    }

    SUBCASE("findFTSensorById - 使用 vector 预分配查找表") {
        int32_t max_id = -1;
        for (const auto& ft : config.ft_sensors) {
            if (ft.id > max_id) max_id = ft.id;
        }
        REQUIRE(max_id >= 0);

        std::vector<int32_t> id_to_index(
            static_cast<size_t>(max_id) + 1, -1);
        for (size_t i = 0; i < config.ft_sensors.size(); ++i) {
            id_to_index[static_cast<size_t>(config.ft_sensors[i].id)] =
                static_cast<int32_t>(i);
        }

        // 查找存在 id=10
        REQUIRE(id_to_index[10] >= 0);

        // 查找不存在的 id
        bool not_found = (999 >= static_cast<int32_t>(id_to_index.size()))
                         || id_to_index[999] < 0;
        CHECK(not_found);
    }

    SUBCASE("findIOById - 使用 vector 预分配查找表") {
        int32_t max_id = -1;
        for (const auto& io : config.ios) {
            if (io.id > max_id) max_id = io.id;
        }
        REQUIRE(max_id >= 0);

        std::vector<int32_t> id_to_index(
            static_cast<size_t>(max_id) + 1, -1);
        for (size_t i = 0; i < config.ios.size(); ++i) {
            id_to_index[static_cast<size_t>(config.ios[i].id)] =
                static_cast<int32_t>(i);
        }

        // 查找存在 id=20
        auto idx20 = id_to_index[20];
        REQUIRE(idx20 >= 0);
        CHECK(config.ios[static_cast<size_t>(idx20)].digital_out_channels == 8);

        // 查找不存在的 id
        bool not_found = (999 >= static_cast<int32_t>(id_to_index.size()))
                         || id_to_index[999] < 0;
        CHECK(not_found);
    }
}

// ==========================================================================
// 9. 所有 torque_source 组合测试
// ==========================================================================

TEST_CASE("Hardware YAML - torque_source 解析") {
    SUBCASE("显式 load_torque") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml(R"(hardware:
  - id: 1
    type: driver
    joint_name: j1
    torque_source: load_torque
)"));
        CHECK(config.drives[0].torque_source == rocos::TorqueSource::LoadTorque);
    }

    SUBCASE("显式 secondary_position") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml(R"(hardware:
  - id: 1
    type: driver
    joint_name: j1
    torque_source: secondary_position
)"));
        CHECK(config.drives[0].torque_source == rocos::TorqueSource::SecondaryPosition);
    }

    SUBCASE("不写 torque_source 默认 load_torque") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml(R"(hardware:
  - id: 1
    type: driver
    joint_name: j1
)"));
        CHECK(config.drives[0].torque_source == rocos::TorqueSource::LoadTorque);
    }

    SUBCASE("非法值默认 load_torque") {
        auto config = rocos::Hardware::LoadConfigFromYAML(
            writeTmpYaml(R"(hardware:
  - id: 1
    type: driver
    joint_name: j1
    torque_source: invalid_value
)"));
        CHECK(config.drives[0].torque_source == rocos::TorqueSource::LoadTorque);
    }
}

TEST_CASE("Hardware YAML imports configurable servo braking parameters") {
    const auto file = writeTmpYaml(R"(
hardware:
  - {type: driver, id: 7, joint_name: axis, limit: {acc: 2, jerk: 20, deceleration: 1.5, velocity_offset: 0.03}}
)");
    const auto config = rocos::Hardware::LoadConfigFromYAML(file);
    REQUIRE(config.drives.size() == 1);
    CHECK(config.drives[0].limit.acc == 2);
    CHECK(config.drives[0].limit.jerk == 20);
    CHECK(config.drives[0].limit.deceleration == doctest::Approx(1.5));
    CHECK(config.drives[0].limit.velocity_offset == doctest::Approx(.03));
}

TEST_CASE("Hardware YAML - 独立变换比例和默认值") {
    auto transform = YAML::Load("{pos_per_unit: -100, ratio: 2, offset_pos_cnt: 37}");
    double expected_velocity = -100.0;
    double expected_torque = 1.0;
    double expected_load = 1.0;
    SUBCASE("速度默认继承位置比例，力矩默认独立为 1") {}
    SUBCASE("显式配置独立比例") {
        transform["vel_per_unit"] = 20.0;
        transform["torque_per_unit"] = 4.0;
        transform["load_torque_per_unit"] = -8.0;
        expected_velocity = 20.0;
        expected_torque = 4.0;
        expected_load = -8.0;
    }
    auto yaml = YAML::Load("hardware: [{id: 0, type: driver}]");
    yaml["hardware"][0]["transform"] = transform;
    const auto config = rocos::Hardware::LoadConfigFromYAML(writeTmpYaml(YAML::Dump(yaml)));
    REQUIRE(config.drives.size() == 1);
    const auto& t = config.drives[0].transform;
    CHECK(t.pos_per_unit == -100.0);
    CHECK(t.vel_per_unit == expected_velocity);
    CHECK(t.torque_per_unit == expected_torque);
    CHECK(t.load_torque_per_unit == expected_load);
}

TEST_CASE("Hardware YAML - 拒绝旧字段和无效变换比例") {
    auto yaml = YAML::Load("hardware: [{id: 0, type: driver}]");
    SUBCASE("旧位置字段必须显式迁移") {
        yaml["hardware"][0]["transform"]["cnt_per_unit"] = 100;
        CHECK_THROWS_WITH(rocos::Hardware::LoadConfigFromYAML(writeTmpYaml(YAML::Dump(yaml))),
                          "transform.cnt_per_unit was renamed to pos_per_unit");
    }
    SUBCASE("除数必须有限且非零") {
        for (const auto* key :
             {"ratio", "pos_per_unit", "vel_per_unit", "torque_per_unit", "load_torque_per_unit"}) {
            for (const auto* value : {"0", ".nan", ".inf"}) {
                INFO(key, " = ", value);
                yaml["hardware"][0]["transform"] = YAML::Load(std::string("{") + key + ": " + value + "}");
                CHECK_THROWS_AS(rocos::Hardware::LoadConfigFromYAML(writeTmpYaml(YAML::Dump(yaml))),
                                std::invalid_argument);
            }
        }
    }
}
