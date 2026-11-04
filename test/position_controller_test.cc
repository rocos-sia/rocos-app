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

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include <test/doctest.h>

#include "src/position_controller.hpp"

// ==========================================================================
// Fake 实现 — 替代真实 Hardware / Model，使 PositionController 可脱离硬件测试
// ==========================================================================

namespace {

/// @brief FakeHardware: 实现 HardwareInterface 的全部纯虚函数，
///        仅 GetPosition / SetPosition / SetMode 有真实行为，其余为空桩。
class FakeHardware : public rocos::HardwareInterface {
public:
    // ---- 可配置行为 ----
    rocos::JntArray fake_position_;     // GetPosition() 的返回值
    rocos::JntArray last_set_position_; // 最后一次 SetPosition 的入参
    int8_t last_set_mode_{-1};          // 最后一次 SetMode 的入参
    int set_mode_count_{0};             // SetMode 被调用次数
    int set_position_count_{0};         // SetPosition 被调用次数
    int wait_for_signal_count_{0};      // WaitForSignal 被调用次数
    uint32_t dt_us_{1000};              // 控制周期 [us]
    rocos::JntState GetState() override { return rocos::JntState::ENABLED; }

    rocos::JntState GetJointState(int32_t id) override { return rocos::JntState::ENABLED; }

    ~FakeHardware() override {}

    // ========== DriveInterface — 批量操作 ==========
    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return {}; }
    rocos::JntArray GetTorque() override { return {}; }
    rocos::JntArray GetLoadTorque() override { return {}; }
    void SetPosition(const rocos::JntArray& q) override {
        last_set_position_ = q;
        ++set_position_count_;
    }
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray&) override {}
    void SetMode(int8_t mode) override {
        last_set_mode_ = mode;
        ++set_mode_count_;
    }
    void SetEnabled() override {}
    void SetDisabled() override {}

    // ========== DriveInterface — 单关节操作 ==========
    double GetJointPosition(int32_t) override { return 0.0; }
    double GetJointVelocity(int32_t) override { return 0.0; }
    double GetJointTorque(int32_t) override { return 0.0; }
    double GetJointLoadTorque(int32_t) override { return 0.0; }
    void SetJointPosition(int32_t, double) override {}
    void SetJointVelocity(int32_t, double) override {}
    void SetJointTorque(int32_t, double) override {}
    void SetJointMode(int32_t, int8_t) override {}
    void SetJointEnabled(int32_t) override {}
    void SetJointDisabled(int32_t) override {}
    std::string getJointName(int32_t) override { return ""; }

    // ========== FTSensorInterface ==========
    rocos::Wrench GetWrench() override { return rocos::Wrench::Zero(); }

    // ========== IOInteface ==========
    bool GetDigitalInput(int, int) override { return false; }
    void SetDigitalOutput(int, int, bool) override {}
    double GetAnalogInput(int, int) override { return 0.0; }
    void SetAnalogOutput(int, int, double) override {}

    // ========== HardwareInterface ==========
    bool Reset() override { return true; }
    void WaitForSignal() override { ++wait_for_signal_count_; }
    uint32_t GetDt() const override { return dt_us_; }
};

/// @brief FakeModel: 实现 ModelInterface，IK 将种子直接作为输出返回（透传），
///        可通过标志位控制失败或输出 NaN。
class FakeModel : public rocos::ModelInterface {
public:
    rocos::Result GetJacobian(const rocos::JntArray &q, rocos::Jacobian &J_out) override {
        return rocos::Result::NoError;
    }

    bool ik_should_fail_{false};   // InverseKinematics 返回 IkCalcFail
    bool ik_output_nan_{false};    // InverseKinematics 输出第一个元素为 NaN
    rocos::JntArray lower_limit_;
    rocos::JntArray upper_limit_;
    rocos::JntArray velocity_limit_;

    rocos::Result ForwardKinematics(const rocos::JntArray&,
                                    rocos::Frame&) override {
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& q_in,
                                     const rocos::Frame&,
                                     rocos::JntArray& q_out) override {
        if (ik_should_fail_) {
            return rocos::Result::IkCalcFail;
        }
        q_out = q_in;   // 透传种子作为 IK 结果，便于验证
        if (ik_output_nan_ && q_out.rows() > 0) {
            q_out(0) = std::numeric_limits<double>::quiet_NaN();
        }
        return rocos::Result::NoError;
    }

    rocos::Result ForwardDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray&) override {
        return rocos::Result::NoError;
    }

    int GetJointNum() const override {
        return lower_limit_.rows() > 0 ? static_cast<int>(lower_limit_.rows()) : 7;
    }
    std::vector<std::string> GetJointNames() const override { return {}; }

    const rocos::JntArray& GetPosLowerLimit() const override { return lower_limit_; }
    const rocos::JntArray& GetPosUpperLimit() const override { return upper_limit_; }
    const rocos::JntArray& GetVelocityLimit() const override { return velocity_limit_; }

    void SetLimits(unsigned int n, double lower, double upper, double velocity) {
        lower_limit_.resize(n);
        upper_limit_.resize(n);
        velocity_limit_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            lower_limit_(i) = lower;
            upper_limit_(i) = upper;
            velocity_limit_(i) = velocity;
        }
    }
};

/// @brief 创建一个指定大小的 JntArray 并填充递增的值: [1.0, 2.0, ...]
rocos::JntArray makeJntArray(unsigned int n) {
    rocos::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
        q(i) = static_cast<double>(i + 1);
    }
    return q;
}

}  // namespace

// ==========================================================================
// 1. SetHardware / SetModel
// ==========================================================================

TEST_CASE("PositionController - SetHardware") {
    FakeHardware hw;
    rocos::PositionController ctrl;

    SUBCASE("传入 nullptr 返回 ParameterPointerEqualsNullptr") {
        CHECK(ctrl.SetHardware(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("传入有效指针返回 NoError") {
        CHECK(ctrl.SetHardware(&hw) == rocos::Result::NoError);
    }
}

TEST_CASE("PositionController - SetModel") {
    rocos::PositionController ctrl;
    FakeModel model;

    SUBCASE("传入 nullptr 返回 ParameterPointerEqualsNullptr") {
        CHECK(ctrl.SetModel(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("传入有效指针返回 NoError") {
        CHECK(ctrl.SetModel(&model) == rocos::Result::NoError);
    }
}

// ==========================================================================
// 2. Reset
// ==========================================================================

TEST_CASE("PositionController - Reset") {
    rocos::PositionController ctrl;
    CHECK(ctrl.Reset() == true);
}

TEST_CASE(
    "PositionController validates the coupled position and velocity before "
    "writing") {
    class CoupledHardware : public FakeHardware {
    public:
        rocos::Result ApplyPositionCoupling(rocos::JntArray& q) const override {
            if (q.rows() != 2) return rocos::Result::UnmatchedJointsNumber;
            q(1) = 2 * q(0) + 0.1;
            return rocos::Result::NoError;
        }
    } hardware;
    hardware.fake_position_.resize(2);
    FakeModel model;
    model.SetLimits(2, -1, 1, 2);
    rocos::PositionController controller;
    REQUIRE(controller.SetHardware(&hardware) == rocos::Result::NoError);
    REQUIRE(controller.SetModel(&model) == rocos::Result::NoError);
    rocos::JntArray q(2);
    q.data << 0.1, 999;
    hardware.fake_position_.data << 0.1, 0;
    CHECK(controller.UpdateCmd(q) == rocos::Result::SpeedLimit);
    CHECK(hardware.set_position_count_ == 0);
    hardware.fake_position_(1) = 0.3;
    REQUIRE(controller.UpdateCmd(q) == rocos::Result::NoError);
    CHECK(hardware.last_set_position_(1) == doctest::Approx(0.3));
    q(0) = 0.1005;
    REQUIRE(controller.UpdateCmd(q) == rocos::Result::NoError);
    CHECK(hardware.last_set_position_(1) == doctest::Approx(0.301));
    q(0) = 0.102;
    CHECK(controller.UpdateCmd(q) == rocos::Result::SpeedLimit);
    controller.Reset();
    q(0) = 0.8;
    CHECK(controller.UpdateCmd(q) == rocos::Result::PosLimit);
    CHECK(hardware.set_position_count_ == 2);
}

TEST_CASE("PositionController - SetReady routes through UpdateCmd") {
    FakeHardware hw;
    FakeModel model;
    rocos::PositionController ctrl;

    SUBCASE("未设置 model 时返回错误且不直接写硬件") {
        ctrl.SetHardware(&hw);
        hw.fake_position_ = makeJntArray(3);

        auto res = ctrl.SetReady();

        CHECK(res == rocos::Result::ParameterPointerEqualsNullptr);
        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("ready 目标通过 UpdateCmd 下发当前位置") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        hw.fake_position_ = makeJntArray(3);
        model.SetLimits(3, -10.0, 10.0, 1000.0);

        auto res = ctrl.SetReady();

        CHECK(res == rocos::Result::NoError);
        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 1);
        CHECK(hw.set_position_count_ == 1);
        REQUIRE(hw.last_set_position_.rows() == 3u);
        CHECK(hw.last_set_position_(0) == doctest::Approx(hw.fake_position_(0)));
    }
}

TEST_CASE("PositionController - destructor routes through UpdateCmd") {
    SUBCASE("未设置 model 时不直接写硬件") {
        FakeHardware hw;
        hw.fake_position_ = makeJntArray(3);
        {
            rocos::PositionController ctrl;
            ctrl.SetHardware(&hw);
        }

        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("析构收尾通过 UpdateCmd 下发当前位置") {
        FakeHardware hw;
        FakeModel model;
        hw.fake_position_ = makeJntArray(3);
        model.SetLimits(3, -10.0, 10.0, 1000.0);
        {
            rocos::PositionController ctrl;
            ctrl.SetHardware(&hw);
            ctrl.SetModel(&model);
        }

        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 1);
        CHECK(hw.set_position_count_ == 1);
        REQUIRE(hw.last_set_position_.rows() == 3u);
        CHECK(hw.last_set_position_(0) == doctest::Approx(hw.fake_position_(0)));
    }
}

// ==========================================================================
// 3. GenerateCmd — JntArray（关节空间透传）
// ==========================================================================

TEST_CASE("PositionController - GenerateCmd with JntArray") {
    FakeHardware hw;
    rocos::PositionController ctrl;
    hw.fake_position_ = makeJntArray(3);
    ctrl.SetHardware(&hw);

    SUBCASE("正常透传") {
        rocos::JntArray q_in = makeJntArray(3);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::NoError);
        CHECK(q_cmd.rows() == 3u);
        CHECK(q_cmd(0) == doctest::Approx(1.0));
        CHECK(q_cmd(1) == doctest::Approx(2.0));
        CHECK(q_cmd(2) == doctest::Approx(3.0));
    }

    SUBCASE("空数组返回 MoveInput") {
        rocos::JntArray q_in;  // rows() == 0
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::MoveInput);
    }

    SUBCASE("含 NaN 返回 ParameterNanOrInf") {
        rocos::JntArray q_in(3);
        q_in(0) = 1.0;
        q_in(1) = std::numeric_limits<double>::quiet_NaN();
        q_in(2) = 3.0;
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::ParameterNanOrInf);
    }

    SUBCASE("含 Inf 返回 ParameterNanOrInf") {
        rocos::JntArray q_in(3);
        q_in(0) = 1.0;
        q_in(1) = std::numeric_limits<double>::infinity();
        q_in(2) = 3.0;
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::ParameterNanOrInf);
    }

    SUBCASE("维度与硬件不匹配返回 UnmatchedJointsNumber") {
        // 硬件报告 6 关节，参考值只有 3 个
        hw.fake_position_ = makeJntArray(6);
        rocos::JntArray q_in = makeJntArray(3);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::UnmatchedJointsNumber);
    }

    SUBCASE("未设置硬件时透传仍然正常（跳过维度校验）") {
        rocos::PositionController ctrl2;  // 不设 hardware
        rocos::JntArray q_in = makeJntArray(5);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;

        auto res = ctrl2.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::NoError);
        CHECK(q_cmd.rows() == 5u);
    }
}

// ==========================================================================
// 4. GenerateCmd — Frame（笛卡尔空间 → IK）
// ==========================================================================

TEST_CASE("PositionController - GenerateCmd with Frame") {
    FakeHardware hw;
    FakeModel model;
    rocos::PositionController ctrl;

    SUBCASE("未设置 hardware 返回 ParameterPointerEqualsNullptr") {
        ctrl.SetModel(&model);
        // 不设 hardware
        rocos::Frame frame;
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("未设置 model 返回 ParameterPointerEqualsNullptr") {
        ctrl.SetHardware(&hw);
        // 不设 model
        rocos::Frame frame;
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("硬件返回空关节（0 轴）返回 JointStateError") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // fake_position_ 默认为空 → rows() == 0
        hw.fake_position_ = rocos::JntArray{};
        rocos::Frame frame;
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::JointStateError);
    }

    SUBCASE("IK 成功 — 输出应与种子一致（FakeModel 透传）") {
        hw.fake_position_ = makeJntArray(3);  // 种子: [1, 2, 3]
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        rocos::Frame frame;                    // 默认 identity 位姿
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::NoError);
        CHECK(q_cmd.rows() == 3u);
        CHECK(q_cmd(0) == doctest::Approx(1.0));
        CHECK(q_cmd(1) == doctest::Approx(2.0));
        CHECK(q_cmd(2) == doctest::Approx(3.0));
    }

    SUBCASE("IK 失败返回 IkCalcFail") {
        hw.fake_position_ = makeJntArray(3);
        model.ik_should_fail_ = true;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        rocos::Frame frame;
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::IkCalcFail);
    }

    SUBCASE("IK 输出含 NaN 返回 IkCalcFail") {
        hw.fake_position_ = makeJntArray(3);
        model.ik_output_nan_ = true;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        rocos::Frame frame;
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;

        auto res = ctrl.GenerateCmd(ref, q_cmd);
        CHECK(res == rocos::Result::IkCalcFail);
    }
}

// ==========================================================================
// 5. UpdateCmd — 发送指令到硬件
// ==========================================================================

TEST_CASE("PositionController - UpdateCmd") {
    FakeHardware hw;
    FakeModel model;
    rocos::PositionController ctrl;

    SUBCASE("未设置 hardware 返回 ParameterPointerEqualsNullptr") {
        rocos::JntArray q(3);
        auto res = ctrl.UpdateCmd(q);
        CHECK(res == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("未设置 model 返回 ParameterPointerEqualsNullptr 且不下发") {
        ctrl.SetHardware(&hw);
        hw.fake_position_ = makeJntArray(3);
        rocos::JntArray q = makeJntArray(3);

        auto res = ctrl.UpdateCmd(q);

        CHECK(res == rocos::Result::ParameterPointerEqualsNullptr);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("首次调用设置 CSP 模式并写入位置") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        rocos::JntArray q = makeJntArray(3);
        hw.fake_position_ = q;
        model.SetLimits(3, -10.0, 10.0, 1000.0);

        auto res = ctrl.UpdateCmd(q);
        CHECK(res == rocos::Result::NoError);
        // 验证 SetMode 被调用且参数为 8 (CSP)
        CHECK(hw.last_set_mode_ == 8);
        CHECK(hw.set_mode_count_ == 1);
        // 验证 SetPosition 被调用且数值正确
        CHECK(hw.last_set_position_.rows() == 3u);
        CHECK(hw.last_set_position_(0) == doctest::Approx(1.0));
    }

    SUBCASE("目标位置超过上限返回 PosLimit 且不下发") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        hw.fake_position_.resize(1);
        hw.fake_position_(0) = 0.0;
        model.SetLimits(1, -1.0, 1.0, 1000.0);
        rocos::JntArray q(1);
        q(0) = 2.0;

        auto res = ctrl.UpdateCmd(q);

        CHECK(res == rocos::Result::PosLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("目标位置低于下限返回 PosLimit 且不下发") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        hw.fake_position_.resize(1);
        hw.fake_position_(0) = 0.0;
        model.SetLimits(1, -1.0, 1.0, 1000.0);
        rocos::JntArray q(1);
        q(0) = -2.0;

        auto res = ctrl.UpdateCmd(q);

        CHECK(res == rocos::Result::PosLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_position_count_ == 0);
    }

    SUBCASE("指令步进速度超限返回 SpeedLimit 且不下发") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        hw.dt_us_ = 1000;
        hw.fake_position_.resize(1);
        hw.fake_position_(0) = 0.0;
        model.SetLimits(1, -10.0, 10.0, 5.0);

        rocos::JntArray q0(1);
        q0(0) = 0.0;
        CHECK(ctrl.UpdateCmd(q0) == rocos::Result::NoError);  // 建立上一周期指令基线

        rocos::JntArray q1(1);
        q1(0) = 0.01;  // (0.01 - 0.0) / 0.001 = 10 rad/s > 5.0
        auto res = ctrl.UpdateCmd(q1);

        CHECK(res == rocos::Result::SpeedLimit);
        CHECK(hw.set_position_count_ == 1);  // 仅基线那次下发成功
    }

    SUBCASE("反馈位置滞后(跟随误差)不再误报 SpeedLimit") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        hw.dt_us_ = 1000;
        hw.fake_position_.resize(1);
        hw.fake_position_(0) = 0.0;          // 反馈停在 0，模拟伺服跟随误差
        model.SetLimits(1, -10.0, 10.0, 5.0);

        // 指令小步进前进（0.001 / 0.001 = 1 rad/s < 5.0），但反馈始终落后
        rocos::JntArray q(1);
        q(0) = 0.5;
        CHECK(ctrl.UpdateCmd(q) == rocos::Result::NoError);  // 首次建立基线
        q(0) = 0.501;
        CHECK(ctrl.UpdateCmd(q) == rocos::Result::NoError);
        q(0) = 0.502;
        CHECK(ctrl.UpdateCmd(q) == rocos::Result::NoError);

        // 反馈(0)与指令(0.502)已相差 0.502，旧逻辑 (q_cmd-q_actual)/dt 会误报 SpeedLimit
        CHECK(hw.set_position_count_ == 3);
    }

    SUBCASE("第二次调用不再重复设置模式") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);

        rocos::JntArray q1 = makeJntArray(3);
        hw.fake_position_ = q1;
        model.SetLimits(3, -10.0, 10.0, 1000.0);
        ctrl.UpdateCmd(q1);
        CHECK(hw.set_mode_count_ == 1);

        rocos::JntArray q2 = makeJntArray(4);
        hw.fake_position_ = q2;
        model.SetLimits(4, -10.0, 10.0, 1000.0);
        ctrl.UpdateCmd(q2);
        CHECK(hw.set_mode_count_ == 1);             // 模式只设一次
        // 但位置应该更新
        CHECK(hw.last_set_position_.rows() == 4u);
    }

    SUBCASE("Reset 后重新设置模式") {
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        rocos::JntArray q = makeJntArray(3);
        hw.fake_position_ = q;
        model.SetLimits(3, -10.0, 10.0, 1000.0);

        ctrl.UpdateCmd(q);
        CHECK(hw.set_mode_count_ == 1);

        ctrl.Reset();
        ctrl.UpdateCmd(q);
        CHECK(hw.set_mode_count_ == 2);             // Reset 后重新设模式
    }
}

// ==========================================================================
// 6. 集成流程测试
// ==========================================================================

TEST_CASE("PositionController - 完整调用链") {
    FakeHardware hw;
    hw.fake_position_ = makeJntArray(6);
    FakeModel model;
    model.SetLimits(6, -10.0, 10.0, 1000.0);

    rocos::PositionController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    SUBCASE("JntArray 路径: GenerateCmd → UpdateCmd 完整流程") {
        rocos::JntArray q_ref = makeJntArray(6);
        rocos::Reference ref = q_ref;
        rocos::JntArray q_cmd;

        auto gen_res = ctrl.GenerateCmd(ref, q_cmd);
        REQUIRE(gen_res == rocos::Result::NoError);

        auto upd_res = ctrl.UpdateCmd(q_cmd);
        CHECK(upd_res == rocos::Result::NoError);
        CHECK(hw.last_set_mode_ == 8);
        CHECK(hw.last_set_position_.rows() == 6u);
    }

    SUBCASE("Frame 路径: GenerateCmd(帧→IK→关节) → UpdateCmd 完整流程") {
        rocos::Frame target;  // 默认 identity
        rocos::Reference ref = target;
        rocos::JntArray q_cmd;

        auto gen_res = ctrl.GenerateCmd(ref, q_cmd);
        REQUIRE(gen_res == rocos::Result::NoError);

        // FakeModel 透传种子 [1..6]
        CHECK(q_cmd.rows() == 6u);
        CHECK(q_cmd(0) == doctest::Approx(1.0));

        auto upd_res = ctrl.UpdateCmd(q_cmd);
        CHECK(upd_res == rocos::Result::NoError);
    }
}
