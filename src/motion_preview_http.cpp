#include <cmath>
#include <set>

#include "robot_http_server.hpp"

namespace rocos {
void RobotHttpServer::handlePreviewMotion(const httplib::Request& req,
                                          httplib::Response& res) {
    // 计算接口：不获取/续期控制权，也不调用任何运动命令。
    const auto body = nlohmann::json::parse(req.body, nullptr, false);
    auto invalid = [&] {
        sendJson(res, false, 1001, "Invalid motion preview parameters");
    };
    if (!body.is_object() || !body.contains("motion_type") ||
        !body["motion_type"].is_string()) {
        invalid();
        return;
    }
    const std::set<std::string> allowed{
        "motion_type", "start_joints", "joints", "pose",
        "velocity",    "acceleration", "jerk",   "max_points"};
    for (const auto& item : body.items())
        if (!allowed.count(item.key())) {
            invalid();
            return;
        }
    MotionPreviewRequest request;
    request.motion_type = body["motion_type"].get<std::string>();
    const bool joint_target =
        request.motion_type == "MoveJ" || request.motion_type == "MoveL_FK";
    if (!joint_target && request.motion_type != "MoveJ_IK" &&
        request.motion_type != "MoveL") {
        invalid();
        return;
    }
    auto joints = [&](const char* key, JntArray& value) {
        if (!body.contains(key) || !body[key].is_array() || body[key].empty() ||
            body[key].size() > 64)
            return false;
        value.resize(body[key].size());
        for (unsigned int i = 0; i < value.rows(); ++i) {
            if (!body[key][i].is_number()) return false;
            value(i) = body[key][i].get<double>();
            if (!std::isfinite(value(i))) return false;
        }
        return true;
    };
    if (!joints("start_joints", request.start_joints)) {
        invalid();
        return;
    }
    if (joint_target) {
        if (body.contains("pose") || !joints("joints", request.joints)) {
            invalid();
            return;
        }
    } else {
        if (body.contains("joints") || !body.contains("pose") ||
            !body["pose"].is_object()) {
            invalid();
            return;
        }
        const auto& pose = body["pose"];
        if (pose.size() != 2 || !pose.contains("position") ||
            !pose.contains("orientation")) {
            invalid();
            return;
        }
        auto components = [](const nlohmann::json& value,
                             const std::vector<std::string>& keys) {
            if (!value.is_object() || value.size() != keys.size()) return false;
            for (const auto& key : keys)
                if (!value.contains(key) || !value[key].is_number() ||
                    !std::isfinite(value[key].get<double>()))
                    return false;
            return true;
        };
        if (!components(pose["position"], {"x", "y", "z"}) ||
            !components(pose["orientation"], {"x", "y", "z", "w"})) {
            invalid();
            return;
        }
        const auto& q = pose["orientation"];
        const double x = q["x"], y = q["y"], z = q["z"], w = q["w"];
        const double norm = std::sqrt(x * x + y * y + z * z + w * w);
        if (!std::isfinite(norm) || norm < 1e-12) {
            invalid();
            return;
        }
        request.pose = Frame(
            KDL::Rotation::Quaternion(x / norm, y / norm, z / norm, w / norm),
            Vector(pose["position"]["x"], pose["position"]["y"],
                   pose["position"]["z"]));
    }
    for (auto entry : {std::make_pair("velocity", &request.velocity),
                       std::make_pair("acceleration", &request.acceleration),
                       std::make_pair("jerk", &request.jerk)}) {
        if (!body.contains(entry.first)) continue;
        if (!body[entry.first].is_number()) {
            invalid();
            return;
        }
        *entry.second = body[entry.first].get<double>();
        if (!std::isfinite(*entry.second) || *entry.second <= 0) {
            invalid();
            return;
        }
    }
    if (body.contains("max_points")) {
        if (!body["max_points"].is_number_integer() || body["max_points"] < 2 ||
            body["max_points"] > 2000) {
            invalid();
            return;
        }
        request.max_points = body["max_points"].get<unsigned int>();
    }
    if (!motion_preview_) {
        sendJson(res, false, -3001, "Motion preview is unavailable");
        return;
    }
    MotionPreviewResult output;
    const auto result = motion_preview_(request, output);
    if (result != Result::NoError) {
        sendJson(res, false, static_cast<int>(result),
                 "Motion preview failed: " + to_string(result));
        return;
    }
    auto points = nlohmann::json::array();
    for (const auto& point : output.points) {
        auto positions = nlohmann::json::array();
        for (unsigned int i = 0; i < point.positions.rows(); ++i)
            positions.push_back(point.positions(i));
        points.push_back({{"time_from_start", point.time_from_start},
                          {"positions", positions},
                          {"flange", frameToJson(point.flange)}});
    }
    sendJson(res, true, 0, "Motion preview generated",
             {
                 {"motion_type", output.motion_type},
                 {"joint_names", output.joint_names},
                 {"duration", output.duration},
                 {"points", points},
                 {"checks", {"joint_position", "joint_velocity", "kinematics"}},
                 {"collision_checked", false},
                 {"execution_checked", false},
             });
}
}  // namespace rocos
