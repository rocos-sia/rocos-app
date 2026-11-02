#pragma once

#include <array>
#include "types.hpp"
#include <string>
#include <vector>

namespace rocos {

struct ToolCalibrationSample {
    rocos::Frame flange;
    std::vector<double> joints;
    double timestamp{0};
};

struct ToolCalibrationSolution {
    rocos::Frame frame;
    double rms_error{0};
    double max_error{0};
    double condition_number{0};
    double axis_angle_degrees{0};
};

// 1–4 点固定 TCP、改变法兰姿态；5/6 点保持第 4 点姿态，沿工具 +X/+Z 移动。
// 所有输入是基坐标系下的法兰位姿，平移与误差单位均为 m。
bool solveSixPointTool(const std::array<rocos::Frame, 6>& samples,
                      double max_allowed_error, ToolCalibrationSolution& result,
                      std::string& error);

} // namespace rocos
