// Offline MoveJog planner linearity diagnostic.
//
// This program does not create Robot, does not start a control thread, and does
// not touch hardware/shared memory. It only runs:
//   MoveJog::FeedJog -> MoveJog::Update -> MoveJog::GenerateRef -> Model::FK
// and checks J(q) * q_dot against the requested BASE X/Y/Z jog direction.
//
// Run from the project root:
//   ./build/bin/move_jog_planner_linearity_test

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "src/model.hpp"
#include "src/move_jog.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kDt = 0.001;
constexpr double kDuration = 2.0;
constexpr double kJogSpeed = 0.03;
constexpr double kJogTimeout = 0.20;
constexpr int kSampleEvery = 20;

struct Point3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct TrajectorySample {
    std::string axis;
    double t{0.0};
    Point3 p;
    rocos::JntArray q;
};

struct VelocitySample {
    std::string axis;
    double t{0.0};
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};
    double wx{0.0};
    double wy{0.0};
    double wz{0.0};
    double linear_speed{0.0};
    double off_axis_speed{0.0};
    double off_axis_ratio{0.0};
    double direction_angle_deg{0.0};
};

struct Summary {
    std::string axis;
    int point_count{0};
    double travel{0.0};
    double max_line_error{0.0};
    double rms_line_error{0.0};
    double angle_to_axis_deg{0.0};
    double max_off_axis_ratio{0.0};
    double rms_off_axis_ratio{0.0};
    double max_angular_speed{0.0};
    double dx{0.0};
    double dy{0.0};
    double dz{0.0};
};

bool isOk(rocos::Result rc) {
    return rc == rocos::Result::NoError;
}

double norm(const Point3& p) {
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

Point3 sub(const Point3& a, const Point3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Point3 scale(const Point3& p, double s) {
    return {p.x * s, p.y * s, p.z * s};
}

double dot(const Point3& a, const Point3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

rocos::JntArray makeHomeJoints() {
    rocos::JntArray q(7);
    const double deg[7] = {0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0};
    for (int i = 0; i < 7; ++i) {
        q(i) = deg[i] * kDegToRad;
    }
    return q;
}

rocos::JntArray subtractJoints(const rocos::JntArray& a, const rocos::JntArray& b) {
    rocos::JntArray out(a.rows());
    for (unsigned int i = 0; i < a.rows(); ++i) {
        out(i) = a(i) - b(i);
    }
    return out;
}

rocos::JntArray divideJoints(const rocos::JntArray& q, double value) {
    rocos::JntArray out(q.rows());
    for (unsigned int i = 0; i < q.rows(); ++i) {
        out(i) = q(i) / value;
    }
    return out;
}

Point3 framePosition(rocos::Model& model, const rocos::JntArray& q) {
    rocos::Frame frame;
    if (!isOk(model.ForwardKinematics(q, frame))) {
        return {};
    }
    return {frame.p.x(), frame.p.y(), frame.p.z()};
}

rocos::Twist directionForAxis(const std::string& axis) {
    rocos::Twist twist;
    if (axis == "X") {
        twist.vel.x(1.0);
    } else if (axis == "Y") {
        twist.vel.y(1.0);
    } else {
        twist.vel.z(1.0);
    }
    return twist;
}

Point3 expectedForAxis(const std::string& axis) {
    if (axis == "X") return {1.0, 0.0, 0.0};
    if (axis == "Y") return {0.0, 1.0, 0.0};
    return {0.0, 0.0, 1.0};
}

std::string findUrdfPath() {
    const std::vector<std::string> candidates = {
        "robot.urdf",
        "config/robot.urdf",
        "config/robots/talon/robot.urdf",
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return "robot.urdf";
}

bool extractJointRef(const rocos::Reference& ref, rocos::JntArray& q) {
    if (!std::holds_alternative<rocos::JntArray>(ref)) {
        return false;
    }
    q = std::get<rocos::JntArray>(ref);
    return true;
}

VelocitySample computeVelocitySample(rocos::Model& model,
                                     const std::string& axis,
                                     double t,
                                     const rocos::JntArray& q,
                                     const rocos::JntArray& q_prev) {
    const rocos::JntArray q_dot = divideJoints(subtractJoints(q, q_prev), kDt);

    rocos::Jacobian jacobian(q.rows());
    model.GetJacobian(q_prev, jacobian);
    const Eigen::VectorXd v = jacobian.data * q_dot.data;

    VelocitySample sample;
    sample.axis = axis;
    sample.t = t;
    sample.vx = v(0);
    sample.vy = v(1);
    sample.vz = v(2);
    sample.wx = v(3);
    sample.wy = v(4);
    sample.wz = v(5);

    const Point3 linear{sample.vx, sample.vy, sample.vz};
    const Point3 expected = expectedForAxis(axis);
    sample.linear_speed = norm(linear);

    const double along_axis = dot(linear, expected);
    const Point3 off_axis = sub(linear, scale(expected, along_axis));
    sample.off_axis_speed = norm(off_axis);
    if (sample.linear_speed > 1e-12) {
        sample.off_axis_ratio = sample.off_axis_speed / sample.linear_speed;
        const double cos_angle = std::clamp(along_axis / sample.linear_speed, -1.0, 1.0);
        sample.direction_angle_deg = std::acos(cos_angle) * 180.0 / kPi;
    }

    return sample;
}

bool runAxis(rocos::Model& model,
             const std::string& axis,
             std::vector<TrajectorySample>& trajectory,
             std::vector<VelocitySample>& velocities) {
    rocos::MoveJog jog(kDt, kJogTimeout, &model);
    jog.setInitialPosition(makeHomeJoints());

    rocos::Result rc = jog.Reset();
    if (!isOk(rc)) {
        std::cerr << "MoveJog Reset failed for " << axis
                  << ", rc=" << static_cast<int>(rc) << std::endl;
        return false;
    }

    rocos::JntArray q_prev = makeHomeJoints();
    const rocos::Twist direction = directionForAxis(axis);
    const int steps = static_cast<int>(kDuration / kDt);

    trajectory.push_back({axis, 0.0, framePosition(model, q_prev), q_prev});

    for (int step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) * kDt;

        rc = jog.FeedJog(direction, kJogSpeed);
        if (!isOk(rc)) {
            std::cerr << "FeedJog failed for " << axis
                      << " at t=" << t << ", rc=" << static_cast<int>(rc) << std::endl;
            return false;
        }


        rocos::Reference ref;
        rc = jog.GenerateRef(ref);
        if (!isOk(rc)) {
            std::cerr << "GenerateRef failed for " << axis
                      << " at t=" << t << ", rc=" << static_cast<int>(rc) << std::endl;
            return false;
        }

        rocos::JntArray q;
        if (!extractJointRef(ref, q)) {
            std::cerr << "GenerateRef did not return JntArray for " << axis << std::endl;
            return false;
        }

        velocities.push_back(computeVelocitySample(model, axis, t, q, q_prev));

        if (step % kSampleEvery == 0 || step == steps) {
            trajectory.push_back({axis, t, framePosition(model, q), q});
        }

        q_prev = q;
    }

    return true;
}

Summary summarizeAxis(const std::string& axis,
                      const std::vector<TrajectorySample>& trajectory,
                      const std::vector<VelocitySample>& velocities) {
    std::vector<Point3> points;
    for (const auto& sample : trajectory) {
        if (sample.axis == axis) {
            points.push_back(sample.p);
        }
    }

    Summary summary;
    summary.axis = axis;
    summary.point_count = static_cast<int>(points.size());
    if (points.size() < 2) {
        summary.angle_to_axis_deg = std::numeric_limits<double>::quiet_NaN();
        return summary;
    }

    const Point3 delta = sub(points.back(), points.front());
    summary.dx = delta.x;
    summary.dy = delta.y;
    summary.dz = delta.z;
    summary.travel = norm(delta);

    if (summary.travel > 1e-12) {
        const Point3 line_dir = scale(delta, 1.0 / summary.travel);
        double sum_sq = 0.0;
        for (const auto& point : points) {
            const Point3 rel = sub(point, points.front());
            const Point3 projection = scale(line_dir, dot(rel, line_dir));
            const double err = norm(sub(rel, projection));
            summary.max_line_error = std::max(summary.max_line_error, err);
            sum_sq += err * err;
        }
        summary.rms_line_error = std::sqrt(sum_sq / static_cast<double>(points.size()));

        const Point3 expected = expectedForAxis(axis);
        const double cos_angle = std::clamp(dot(line_dir, expected), -1.0, 1.0);
        summary.angle_to_axis_deg = std::acos(cos_angle) * 180.0 / kPi;
    } else {
        summary.angle_to_axis_deg = std::numeric_limits<double>::quiet_NaN();
    }

    double ratio_sq = 0.0;
    int ratio_count = 0;
    for (const auto& sample : velocities) {
        if (sample.axis != axis) continue;
        summary.max_off_axis_ratio = std::max(summary.max_off_axis_ratio, sample.off_axis_ratio);
        ratio_sq += sample.off_axis_ratio * sample.off_axis_ratio;
        ++ratio_count;

        const double angular_speed = std::sqrt(
            sample.wx * sample.wx + sample.wy * sample.wy + sample.wz * sample.wz);
        summary.max_angular_speed = std::max(summary.max_angular_speed, angular_speed);
    }
    if (ratio_count > 0) {
        summary.rms_off_axis_ratio = std::sqrt(ratio_sq / static_cast<double>(ratio_count));
    }

    return summary;
}

void writeTrajectoryCsv(const std::filesystem::path& path,
                        const std::vector<TrajectorySample>& trajectory) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(9);
    out << "axis,t,x,y,z";
    for (int i = 0; i < 7; ++i) out << ",q" << i;
    out << '\n';

    for (const auto& sample : trajectory) {
        out << sample.axis << ',' << sample.t << ','
            << sample.p.x << ',' << sample.p.y << ',' << sample.p.z;
        for (int i = 0; i < 7; ++i) {
            out << ',' << sample.q(i);
        }
        out << '\n';
    }
}

void writeVelocityCsv(const std::filesystem::path& path,
                      const std::vector<VelocitySample>& velocities) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(9);
    out << "axis,t,vx,vy,vz,wx,wy,wz,linear_speed,off_axis_speed,"
           "off_axis_ratio,direction_angle_deg\n";
    for (const auto& sample : velocities) {
        out << sample.axis << ',' << sample.t << ','
            << sample.vx << ',' << sample.vy << ',' << sample.vz << ','
            << sample.wx << ',' << sample.wy << ',' << sample.wz << ','
            << sample.linear_speed << ',' << sample.off_axis_speed << ','
            << sample.off_axis_ratio << ',' << sample.direction_angle_deg << '\n';
    }
}

void writeSummaryCsv(const std::filesystem::path& path,
                     const std::vector<Summary>& summaries) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(9);
    out << "axis,point_count,travel,max_line_error,rms_line_error,"
           "angle_to_axis_deg,max_off_axis_ratio,rms_off_axis_ratio,"
           "max_angular_speed,dx,dy,dz\n";
    for (const auto& summary : summaries) {
        out << summary.axis << ',' << summary.point_count << ','
            << summary.travel << ',' << summary.max_line_error << ','
            << summary.rms_line_error << ',' << summary.angle_to_axis_deg << ','
            << summary.max_off_axis_ratio << ',' << summary.rms_off_axis_ratio << ','
            << summary.max_angular_speed << ','
            << summary.dx << ',' << summary.dy << ',' << summary.dz << '\n';
    }
}

}  // namespace

int main() {
    const std::filesystem::path output_dir = "build/move_jog_planner_linearity";
    std::filesystem::create_directories(output_dir);

    const std::string urdf_path = findUrdfPath();
    std::cout << "Using URDF: " << urdf_path << std::endl;

    rocos::Model model(urdf_path, "base_link", "link_7");

    std::vector<TrajectorySample> trajectory;
    std::vector<VelocitySample> velocities;
    const std::vector<std::string> axes = {"X", "Y", "Z"};

    for (const auto& axis : axes) {
        std::cout << "Offline planning BASE_" << axis << std::endl;
        if (!runAxis(model, axis, trajectory, velocities)) {
            return 1;
        }
    }

    std::vector<Summary> summaries;
    for (const auto& axis : axes) {
        summaries.push_back(summarizeAxis(axis, trajectory, velocities));
    }

    const auto trajectory_path = output_dir / "planner_trajectory.csv";
    const auto velocity_path = output_dir / "planner_velocity_check.csv";
    const auto summary_path = output_dir / "planner_summary.csv";

    writeTrajectoryCsv(trajectory_path, trajectory);
    writeVelocityCsv(velocity_path, velocities);
    writeSummaryCsv(summary_path, summaries);

    std::cout << "\nOffline planner summary" << std::endl;
    std::cout << "axis  travel(m)  line_max(m)  line_rms(m)  angle(deg)  "
                 "Jqd_off_axis_max  Jqd_off_axis_rms  max_w(rad/s)  delta(x,y,z)"
              << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (const auto& summary : summaries) {
        std::cout << summary.axis << "     "
                  << summary.travel << "    "
                  << summary.max_line_error << "    "
                  << summary.rms_line_error << "    "
                  << summary.angle_to_axis_deg << "      "
                  << summary.max_off_axis_ratio << "            "
                  << summary.rms_off_axis_ratio << "            "
                  << summary.max_angular_speed << "      ("
                  << summary.dx << ", " << summary.dy << ", " << summary.dz << ")"
                  << std::endl;
    }

    std::cout << "\nWrote: " << trajectory_path << std::endl;
    std::cout << "Wrote: " << velocity_path << std::endl;
    std::cout << "Wrote: " << summary_path << std::endl;

    return 0;
}
