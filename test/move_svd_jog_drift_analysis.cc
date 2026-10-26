// SVD 零空间点动 — 末端漂移累积分析
//
// 离线运行 MoveSvdJog 较长时间，每个 dt 采样 FK，
// 输出 CSV 用于可视化末端位姿的漂移趋势。
//
// 运行方式（项目根目录）：
//   ./build/bin/move_svd_jog_drift_analysis [duration_s] [speed] [csv_output]
//   默认: 30s, 0.03 rad/s, build/svd_drift.csv
//
// 核心问题：
//   J(q) 随 q 变化 → 零空间基向量 V₀(q) 也在漂 →
//   q_dot = α · V₀(q_current) 始终在 J(q_current) 的零空间，
//   但 q(t+dt) = q(t) + q_dot*dt 移动后，J(q) 变了，
//   V₀(q_old) ≠ V₀(q_new) → 末端沿零空间的"法向"分量累积漂移。

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <variant>
#include <vector>

#include "src/model.hpp"
#include "src/move_svd_jog.hpp"

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kDt       = 0.001;
constexpr double kTimeout  = 0.20;

struct FkSample {
    double t{0.0};
    double px{0.0}, py{0.0}, pz{0.0};
    double m00{0.0}, m01{0.0}, m02{0.0};
    double m10{0.0}, m11{0.0}, m12{0.0};
    double m20{0.0}, m21{0.0}, m22{0.0};
    double q[7]{0.0};
};

bool isOk(rocos::Result rc) { return rc == rocos::Result::NoError; }

rocos::JntArray makeHomeJoints() {
    rocos::JntArray q(7);
    const double deg[7] = {0.0, 60.0, 0.0, 90.0, 0.0, -60.0, 0.0};
    for (int i = 0; i < 7; ++i) q(i) = deg[i] * kDegToRad;
    return q;
}

void frameToSample(double t, const rocos::Frame& f, const rocos::JntArray& q, FkSample& s) {
    s.t   = t;
    s.px  = f.p.x();  s.py  = f.p.y();  s.pz  = f.p.z();
    s.m00 = f.M(0,0); s.m01 = f.M(0,1); s.m02 = f.M(0,2);
    s.m10 = f.M(1,0); s.m11 = f.M(1,1); s.m12 = f.M(1,2);
    s.m20 = f.M(2,0); s.m21 = f.M(2,1); s.m22 = f.M(2,2);
    for (int i = 0; i < 7; ++i) s.q[i] = q(i);
}

void writeCsv(const std::string& path, const std::vector<FkSample>& samples) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(12);
    out << "t,px,py,pz,m00,m01,m02,m10,m11,m12,m20,m21,m22";
    for (int i = 0; i < 7; ++i) out << ",q" << i;
    out << '\n';
    for (const auto& s : samples) {
        out << s.t << ',' << s.px << ',' << s.py << ',' << s.pz << ','
            << s.m00 << ',' << s.m01 << ',' << s.m02 << ','
            << s.m10 << ',' << s.m11 << ',' << s.m12 << ','
            << s.m20 << ',' << s.m21 << ',' << s.m22;
        for (int i = 0; i < 7; ++i) out << ',' << s.q[i];
        out << '\n';
    }
}

std::string findUrdf() {
    for (const auto& c : {"robot.urdf", "config/robot.urdf", "config/robots/talon/robot.urdf"})
        if (std::filesystem::exists(c)) return c;
    return "robot.urdf";
}

bool extractJointRef(const rocos::Reference& ref, rocos::JntArray& q) {
    if (!std::holds_alternative<rocos::JntArray>(ref)) return false;
    q = std::get<rocos::JntArray>(ref);
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    const double duration = (argc > 1) ? std::stod(argv[1]) : 30.0;
    const double speed    = (argc > 2) ? std::stod(argv[2]) : 0.03;
    const std::string outpath = (argc > 3) ? argv[3] : "build/svd_drift.csv";

    std::cout << "SVD 零空间漂移分析" << std::endl;
    std::cout << "  时长: " << duration << " s, 速度: " << speed << " rad/s" << std::endl;
    std::cout << "  输出: " << outpath << std::endl;

    rocos::Model model(findUrdf(), "base_link", "link_7");
    const rocos::JntArray q_home = makeHomeJoints();

    // 初始 FK
    rocos::Frame f0;
    model.ForwardKinematics(q_home, f0);

    rocos::MoveSvdJog svd(kDt, kTimeout, &model);
    svd.setInitialPosition(q_home);
    if (!isOk(svd.Reset())) { std::cerr << "Reset failed\n"; return 1; }

    const int steps = static_cast<int>(duration / kDt);
    // 每 20ms 记录一次（避免 CSV 过大）
    const int record_every = static_cast<int>(0.02 / kDt);
    std::vector<FkSample> samples;
    rocos::JntArray q_cur = q_home;

    // 记录初始点
    {
        FkSample s0;
        frameToSample(0.0, f0, q_home, s0);
        samples.push_back(s0);
    }

    for (int step = 1; step <= steps; ++step) {
        const double t = step * kDt;
        svd.FeedSvdJog({speed});
        rocos::Reference ref;
        svd.GenerateRef(ref);
        extractJointRef(ref, q_cur);

        if (step % record_every == 0) {
            rocos::Frame f;
            model.ForwardKinematics(q_cur, f);
            FkSample s;
            frameToSample(t, f, q_cur, s);
            samples.push_back(s);
        }
    }

    writeCsv(outpath, samples);

    // 概要
    const auto& last = samples.back();
    const double dx = last.px - samples[0].px;
    const double dy = last.py - samples[0].py;
    const double dz = last.pz - samples[0].pz;
    const double drift = std::sqrt(dx*dx + dy*dy + dz*dz);

    // 旋转变化 Frobenius 范数
    double dfrob = 0.0;
    {
        const auto& a = samples[0]; const auto& b = last;
        auto sq = [](double v){ return v*v; };
        dfrob = std::sqrt(sq(b.m00-a.m00)+sq(b.m01-a.m01)+sq(b.m02-a.m02)
                        + sq(b.m10-a.m10)+sq(b.m11-a.m11)+sq(b.m12-a.m12)
                        + sq(b.m20-a.m20)+sq(b.m21-a.m21)+sq(b.m22-a.m22));
    }

    std::cout << std::fixed << std::setprecision(9);
    std::cout << "\n漂移结果 (" << duration << " s):" << std::endl;
    std::cout << "  位置漂移: " << drift * 1000.0 << " mm"
              << "  (dx=" << dx*1000 << " dy=" << dy*1000 << " dz=" << dz*1000 << " mm)" << std::endl;
    std::cout << "  旋转漂移: " << dfrob << " (Frobenius)" << std::endl;
    std::cout << "  漂移速率: " << (drift / duration) * 1e6 << " nm/s" << std::endl;
    std::cout << "\n写入: " << outpath << std::endl;

    return 0;
}
