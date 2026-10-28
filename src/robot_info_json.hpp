#pragma once

#include <json.hpp>

#include "robot.hpp"
#include "ver.h"

namespace rocos {

// 信息协议 v1：尚未接入的数据保持 null，配置驱动器不代表在线从站扫描结果。
inline nlohmann::json robotInfoJson(const std::vector<Robot::JointInfo>& joints) {
    using Json = nlohmann::json;
    Json joint_infos = Json::array();
    Json slaves = Json::array();
    for (const auto& joint : joints) {
        joint_infos.push_back({{"id", joint.id}, {"name", joint.name},
                               {"cnt_per_unit", joint.cnt_per_unit},
                               {"torque_per_unit", joint.torque_per_unit}, {"ratio", joint.ratio},
                               {"unit_name", joint.unit_name}, {"zero_offset", joint.zero_offset}});
        slaves.push_back({{"id", joint.id}, {"name", nullptr}, {"joint_name", joint.name},
                          {"role", "drive"}, {"source", "configuration"},
                          {"vendor_id", nullptr}, {"product_code", nullptr},
                          {"revision_number", nullptr}, {"serial_number", nullptr},
                          {"firmware_version", nullptr}, {"identity_source", "unknown"},
                          {"input_pdo_count", nullptr}, {"output_pdo_count", nullptr}});
    }
    return {
        {"info_schema_version", 1},
        {"controller", {{"app_version", ROCOS_APP_VERSION}, {"build_commit", nullptr},
                        {"api_version", "3.0.0"}, {"shm_abi_version", SHM_ABI_VERSION},
                        {"capabilities", Json::array({"robot_info_v1"})}}},
        {"robot", {{"source", "placeholder"}, {"name", nullptr}, {"package_version", nullptr},
                   {"device_id", nullptr}, {"serial_number", nullptr}, {"dof", nullptr},
                   {"base_link", nullptr}, {"tip_link", nullptr}, {"model_digest", nullptr},
                   {"link_count", nullptr}, {"mass_kg", nullptr}, {"mass_complete", false}}},
        {"master", {{"source", "placeholder"}, {"backend", "unknown"}, {"version", nullptr},
                    {"sdk_version", nullptr}, {"master_id", nullptr}, {"cycle_time_us", nullptr},
                    {"configured_slave_count", nullptr}}},
        {"slaves", std::move(slaves)},
        {"slaves_complete", false},
        {"joint_infos", std::move(joint_infos)},
    };
}

}  // namespace rocos
