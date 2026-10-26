// MoveJog 笛卡尔点动 — 偏轴漂移分析
//
// 单向 BASE_X 点动，高频采样 FK，分析 Y/Z 偏轴漂移是否随行程单调累加。
// 同时跑规划层和模拟控制链路层对比。
//
// 运行: ./build/bin/move_jog_offaxis_drift_test [duration_s]

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>

#include <Eigen/SVD>

#include "src/model.hpp"
#include "src/move_jog.hpp"

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kDt       = 0.001;
constexpr double kTimeout  = 0.20;
constexpr double kJogSpeed = 0.03;
constexpr double kFeedPeriod = 0.05;

bool isOk(rocos::Result rc) { return rc == rocos::Result::NoError; }

rocos::JntArray makeHomeJoints() {
    rocos::JntArray q(7);
    const double deg[7] = {0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0};
    for (int i = 0; i < 7; ++i) q(i) = deg[i] * kDegToRad;
    return q;
}

bool extractJointRef(const rocos::Reference& ref, rocos::JntArray& q) {
    if (!std::holds_alternative<rocos::JntArray>(ref)) return false;
    q = std::get<rocos::JntArray>(ref);
    return true;
}

std::string findUrdf() {
    for (const auto& c : {"robot.urdf", "config/robot.urdf", "config/robots/talon/robot.urdf"})
        if (std::filesystem::exists(c)) return c;
    return "robot.urdf";
}

}  // namespace

int main(int argc, char* argv[]) {
    const double duration = (argc > 1) ? std::stod(argv[1]) : 10.0;
    const std::string csv_path = (argc > 2) ? argv[2] : "build/offaxis_drift.csv";

    std::cout << "MoveJog 偏轴漂移分析 — BASE_X 单向 " << duration << "s" << std::endl;

    rocos::Model model(findUrdf(), "base_link", "link_7");
    const rocos::JntArray q_home = makeHomeJoints();

    // 初始 FK
    rocos::Frame fk0;
    model.ForwardKinematics(q_home, fk0);
    const double px0 = fk0.p.x(), py0 = fk0.p.y(), pz0 = fk0.p.z();

    rocos::MoveJog jog(kDt, kTimeout, &model);
    jog.setInitialPosition(q_home);
    jog.Reset();

    rocos::Twist dir_x;
    dir_x.vel.x(1.0);
    jog.FeedJog(dir_x, kJogSpeed);

    const int total_steps = static_cast<int>(duration / kDt);
    const int feed_every  = static_cast<int>(kFeedPeriod / kDt);
    const int record_every = static_cast<int>(0.01 / kDt);  // 每 10ms 记录
    rocos::JntArray q_cur = q_home;

    // CSV 输出
    std::ofstream csv(csv_path);
    csv << std::fixed << std::setprecision(9);
    csv << "t,px,py,pz,dx,dy,dz";
    for (int i = 0; i < 7; ++i) csv << ",q" << i;
    // 额外输出雅可比条件数
    csv << ",cond_J";
    csv << '\n';

    for (int step = 1; step <= total_steps; ++step) {
        const double t = step * kDt;

        if (step > 1 && step % feed_every == 0) {
            jog.FeedJog(dir_x, kJogSpeed);
        }

        rocos::Reference ref;
        auto rc = jog.GenerateRef(ref);
        if (rc == rocos::Result::PlanFinished) break;
        extractJointRef(ref, q_cur);

        if (step % record_every == 0) {
            rocos::Frame fk;
            model.ForwardKinematics(q_cur, fk);
            const double dx = (fk.p.x() - px0) * 1000.0;  // mm
            const double dy = (fk.p.y() - py0) * 1000.0;
            const double dz = (fk.p.z() - pz0) * 1000.0;

            // 条件数
            rocos::Jacobian J(7);
            model.GetJacobian(q_cur, J);
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(
                J.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const auto& sv = svd.singularValues();
            double cond = (sv(sv.size()-1) > 1e-12)
                          ? sv(0) / sv(sv.size()-1) : -1.0;

            csv << t << ',' << fk.p.x() << ',' << fk.p.y() << ',' << fk.p.z() << ','
                << dx << ',' << dy << ',' << dz;
            for (int i = 0; i < 7; ++i) csv << ',' << q_cur(i);
            csv << ',' << cond;
            csv << '\n';
        }
    }

    // 终止 FK
    rocos::Frame fkn;
    model.ForwardKinematics(q_cur, fkn);
    const double dx_final = (fkn.p.x() - px0) * 1000.0;
    const double dy_final = (fkn.p.y() - py0) * 1000.0;
    const double dz_final = (fkn.p.z() - pz0) * 1000.0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n单向 BASE_X " << duration << "s 结果:" << std::endl;
    std::cout << "  X 行程: " << dx_final << " mm (理论 "
              << (kJogSpeed * duration * 1000.0) << " mm)" << std::endl;
    std::cout << "  Y 偏轴漂移: " << dy_final << " mm"
              << "  (" << (dy_final / duration * 1e6) << " nm/s)" << std::endl;
    std::cout << "  Z 偏轴漂移: " << dz_final << " mm"
              << "  (" << (dz_final / duration * 1e6) << " nm/s)" << std::endl;

    // 偏轴/主轴比
    const double off_ratio = std::sqrt(dy_final*dy_final + dz_final*dz_final)
                             / std::abs(dx_final) * 100.0;
    std::cout << "  偏轴比例: " << off_ratio << " %" << std::endl;
    std::cout << "\n写入: " << csv_path << std::endl;

    return 0;
}
