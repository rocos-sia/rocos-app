// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn
#pragma once

#include "motion_interface.hpp"
#include "model_interface.hpp"

#include <ruckig/ruckig.hpp>

#include <atomic>
#include <chrono>
#include <mutex>

namespace rocos {

/// @brief 连续点动 (Jog) — 支持关节空间和笛卡尔空间
///
/// 生命周期三态：
///   Active → [超时/Stop] → Decelerating → [speed=0] → Stopped
///   Stopped 后拒绝 FeedJog，须重建实例或 Reset 复用。
///
/// 速度 OTG：ruckig::Ruckig<1> Velocity 模式平滑变速。
/// 关节空间直接积分；笛卡尔通过 Jacobian 伪逆转关节速度再积分。
///
/// 线程：FeedJog=HTTP 线程，GenerateRef=控制线程，mtx_ 保护共享状态。
class MoveJog : public MotionInterface {
public:
    enum class State { Active, Decelerating, Stopped };

    MoveJog(double dt = 0.001,
            double timeout = 0.1,
            ModelInterface* model = nullptr,
            double dir_threshold = 0.99);

    ~MoveJog() override;

    // ─── MotionInterface ───
    Result ValidateParameters() const override;
    Result Reset() override;
    Result GenerateRef(Reference& ref_out) override;

    bool CanPause()  const override { return false; }
    bool CanResume() const override { return false; }
    bool CanStop()   const override { return true; }

    Result Pause() override;
    Result Resume() override;
    Result Stop() override;

    // ─── Jog 特有 ───
    void setInitialPosition(const JntArray& q);

    /// @brief 上位机周期喂入方向 + 速度模长；twist 在 BASE 表达。
    /// @param flange_to_point 速度参考点相对法兰的偏移（法兰系表达）。
    ///        TOOL 点动传入 TCP 偏移；归一化在参考点换算前完成。
    Result FeedJog(const JogVec& direction, double speed,
                   const Vector& flange_to_point = Vector::Zero());

    bool IsStopped() const { return state_.load() == State::Stopped; }

private:
    static double normalizeDirection(const JogVec& src, JogVec& dst);
    static double directionCosine(const JogVec& a, const JogVec& b);

    void reconfigureSpeedOtg(double target_speed);
    Result computeJointVelocity(JntArray& q_dot);

    const double dt_;
    const double timeout_;
    const double dir_threshold_;

    // 速度 OTG（一维，控制线程独占）
    ruckig::Ruckig<1>          speed_otg_;
    ruckig::InputParameter<1>  speed_input_;
    ruckig::OutputParameter<1> speed_output_;
    double speed_otg_current_{0.0};

    double v_max_{3.0};
    double a_max_{5.0};
    double j_max_{10.0};

    // ─── mtx_ 保护（HTTP 写 / 控制读） ───
    mutable std::mutex mtx_;
    JogVec direction_;
    double target_speed_{0.0};
    double elapsed_since_feed_{0.0};
    bool   is_joint_space_{false};
    Vector flange_to_point_;

    // ─── 原子变量（无锁读） ───
    std::atomic<State> state_{State::Active};
    std::atomic<bool>  has_started_{false};

    // ─── 控制线程独享 ───
    JntArray  q_integral_;
    int       joint_count_{0};
    Jacobian  jacobian_;
};

} // namespace rocos
