#pragma once

#include <string>
#include <vector>

#include "model_interface.hpp"

namespace rocos {
struct MotionPreviewRequest {
    std::string motion_type;
    JntArray start_joints;  // 必填；由调用者显式选择真实或虚拟起点。
    JntArray joints;
    Frame pose;  // 基座系下的法兰目标，与不指定 tool_name 的运动命令一致。
    double velocity{1.0}, acceleration{2.0}, jerk{10.0};
    unsigned int max_points{300};
};
struct MotionPreviewPoint {
    double time_from_start{0};
    JntArray positions;
    Frame flange;
};
struct MotionPreviewResult {
    std::string motion_type;
    std::vector<std::string> joint_names;
    double duration{0};
    std::vector<MotionPreviewPoint> points;
};

// model 必须是预览任务独占实例。最多 120 秒/120000 步，计算预算 5 秒。
Result GenerateMotionPreview(ModelInterface& model,
                             const MotionPreviewRequest& request, double dt,
                             MotionPreviewResult& result);
}  // namespace rocos
