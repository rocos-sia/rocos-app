#include "tool_calibration.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace rocos {
namespace {
Eigen::Matrix3d rotation(const rocos::Frame& frame) {
    Eigen::Matrix3d result;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) result(i, j) = frame.M(i, j);
    return result;
}
Eigen::Vector3d position(const rocos::Frame& frame) {
    return {frame.p.x(), frame.p.y(), frame.p.z()};
}
constexpr double pi = 3.14159265358979323846;
}

bool solveSixPointTool(const std::array<rocos::Frame, 6>& samples,
                      double max_allowed_error, ToolCalibrationSolution& result,
                      std::string& error) {
    result = {};
    error.clear();
    if (!std::isfinite(max_allowed_error) || max_allowed_error <= 0 || max_allowed_error > 0.05) {
        error = "Maximum pivot error must be in (0, 0.05] m.";
        return false;
    }
    std::array<Eigen::Matrix3d, 6> r;
    std::array<Eigen::Vector3d, 6> p;
    for (size_t i = 0; i < samples.size(); ++i) {
        r[i] = rotation(samples[i]); p[i] = position(samples[i]);
        if (!r[i].allFinite() || !p[i].allFinite() ||
            (r[i].transpose() * r[i] - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
            std::abs(r[i].determinant() - 1) > 1e-6) {
            error = "All six flange poses must be finite rigid transforms.";
            return false;
        }
    }

    Eigen::Matrix<double, 9, 3> a;
    Eigen::Matrix<double, 9, 1> b;
    for (int i = 0; i < 3; ++i) {
        a.block<3, 3>(3 * i, 0) = r[i] - r[3];
        b.segment<3>(3 * i) = p[3] - p[i];
    }
    // SVD 不构造正规方程，拒绝不可观测或严重病态的姿态组合。
    Eigen::JacobiSVD<Eigen::Matrix<double, 9, 3>> svd(a, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singular = svd.singularValues();
    if (singular[0] < 1e-6 || singular[2] < std::max(1e-6, singular[0] * 1e-6)) {
        error = "The first four orientations do not constrain the TCP. Use distinct rotations about multiple axes.";
        return false;
    }
    result.condition_number = singular[0] / singular[2];
    const Eigen::Vector3d tcp = svd.solve(b);
    std::array<Eigen::Vector3d, 6> tips;
    for (size_t i = 0; i < samples.size(); ++i) tips[i] = r[i] * tcp + p[i];
    const Eigen::Vector3d pivot = (tips[0] + tips[1] + tips[2] + tips[3]) / 4;
    double sum_squared = 0;
    for (int i = 0; i < 4; ++i) {
        const double deviation = (tips[i] - pivot).norm();
        result.max_error = std::max(result.max_error, deviation);
        sum_squared += deviation * deviation;
    }
    result.rms_error = std::sqrt(sum_squared / 4);
    if (!tcp.allFinite() || result.max_error > max_allowed_error) {
        error = "Pivot error exceeds the selected tolerance. Recapture the first four points at the same fixed TCP.";
        return false;
    }
    for (int i : {4, 5}) {
        const double cosine = std::clamp(((r[3].transpose() * r[i]).trace() - 1) / 2, -1.0, 1.0);
        if (std::acos(cosine) > pi / 180) {
            error = "Points 5 and 6 must keep the flange orientation of point 4 (within 1 degree).";
            return false;
        }
    }
    Eigen::Vector3d x = tips[4] - tips[3], z_hint = tips[5] - tips[3];
    if (x.norm() < 0.001 || z_hint.norm() < 0.001) {
        error = "Move at least 1 mm from point 4 along +X and +Z for points 5 and 6.";
        return false;
    }
    x.normalize(); z_hint.normalize();
    Eigen::Vector3d y = z_hint.cross(x);
    if (y.norm() < std::sin(10 * pi / 180)) {
        error = "The two tool axis directions are nearly collinear. Recapture points 5 and 6.";
        return false;
    }
    result.axis_angle_degrees = std::acos(std::clamp(x.dot(z_hint), -1.0, 1.0)) * 180 / pi;
    y.normalize();
    const Eigen::Vector3d z = x.cross(y).normalized();
    Eigen::Matrix3d base_tool;
    base_tool.col(0) = x; base_tool.col(1) = y; base_tool.col(2) = z;
    const Eigen::Matrix3d flange_tool = r[3].transpose() * base_tool;
    result.frame = rocos::Frame(rocos::Rotation(
        flange_tool(0, 0), flange_tool(0, 1), flange_tool(0, 2),
        flange_tool(1, 0), flange_tool(1, 1), flange_tool(1, 2),
        flange_tool(2, 0), flange_tool(2, 1), flange_tool(2, 2)),
        rocos::Vector(tcp.x(), tcp.y(), tcp.z()));
    return true;
}
} // namespace rocos
