#pragma once
#include "tool_package.hpp"
#include "json.hpp"
namespace rocos {
inline nlohmann::json toolPoseJson(const rocos::Frame& frame) {
    double x,y,z,w; frame.M.GetQuaternion(x,y,z,w);
    return {{"position",{{"x",frame.p.x()},{"y",frame.p.y()},{"z",frame.p.z()}}},
            {"orientation",{{"x",x},{"y",y},{"z",z},{"w",w}}}};
}
inline nlohmann::json toolPackageJson(const ToolPackage& tool, bool detail = true) {
    nlohmann::json value{{"name",tool.name},{"package_version",tool.version},{"digest",tool.digest},
        {"revision",tool.revision},{"urdf",tool.urdf},{"mount",toolPoseJson(tool.mount)},{"tcp",toolPoseJson(tool.tcp)}};
    if (detail) {
        value["default_mount"]=toolPoseJson(tool.default_mount); value["default_tcp"]=toolPoseJson(tool.default_tcp);
        // YAML 保留所有标定元数据；浏览器按原文显示/下载，不把它当作可写状态。
        if (!tool.calibration_yaml.empty()) value["calibration_yaml"]=tool.calibration_yaml;
    }
    return value;
}
} // namespace rocos
