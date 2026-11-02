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

#include "src/cartesian_impedance_controller.hpp"

// ==========================================================================
// Fake 实现 — 替代真实 Hardware / Model，使控制器可脱离硬件测试
// ==========================================================================

namespace {

class FakeHardware : public rocos::HardwareInterface {
public:
    rocos::JntArray fake_position_;
    rocos::JntArray fake_velocity_;
    rocos::JntArray last_set_torque_;
    rocos::JntArray last_set_position_;
    int8_t last_set_mode_{-1};
    int set_mode_count_{0};
    int set_torque_count_{0};
    int set_position_count_{0};
    int wait_for_signal_count_{0};

    rocos::JntState GetState() override { return rocos::JntState::ENABLED; }
    rocos::JntState GetJointState(int32_t) override { return rocos::JntState::ENABLED; }
    ~FakeHardware() override {}

    // DriveInterface — batch
    rocos::JntArray GetPosition() override { return fake_position_; }
    rocos::JntArray GetVelocity() override { return fake_velocity_; }
    rocos::JntArray GetTorque() override { return {}; }
    rocos::JntArray GetLoadTorque() override { return {}; }
    void SetPosition(const rocos::JntArray& q) override {
        last_set_position_ = q;
        ++set_position_count_;
    }
    void SetVelocity(const rocos::JntArray&) override {}
    void SetTorque(const rocos::JntArray& tau) override {
        last_set_torque_ = tau;
        ++set_torque_count_;
    }
    void SetMode(int8_t mode) override {
        last_set_mode_ = mode;
        ++set_mode_count_;
    }
    void SetEnabled() override {}
    void SetDisabled() override {}

    // DriveInterface — single joint
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

    // FTSensorInterface
    rocos::Wrench GetWrench() override { return rocos::Wrench::Zero(); }

    // IOInteface
    bool GetDigitalInput(int32_t, int32_t) override { return false; }
    void SetDigitalOutput(int32_t, int32_t, bool) override {}
    double GetAnalogInput(int32_t, int32_t) override { return 0.0; }
    void SetAnalogOutput(int32_t, int32_t, double) override {}

    // HardwareInterface
    bool Reset() override { return true; }
    void WaitForSignal() override { ++wait_for_signal_count_; }
};

/// @brief FakeModel: 提供可预测的 FK、Jacobian、InverseDynamics 返回值
///
/// FK:    p_out = Vector(q[0], q[1], q[2]), R = Identity
/// Jacobian: 6×n，前 3 列为 XYZ 平动，后 3 列为 RPY 转动，其余列为混合
/// ID:    每个关节返回常数重力力矩
class FakeModel : public rocos::ModelInterface {
public:
    FakeModel() { SetLimits(7, -10.0, 10.0, 10000.0, 10000.0); }

    bool fk_should_fail_{false};
    bool jac_should_fail_{false};
    bool id_should_fail_{false};
    double grav_torque_{5.0};   // 每个关节的模拟重力力矩 [Nm]
    rocos::JntArray lower_limit_;
    rocos::JntArray upper_limit_;
    rocos::JntArray velocity_limit_;
    rocos::JntArray effort_limit_;

    rocos::Result ForwardKinematics(const rocos::JntArray& q_in,
                                    rocos::Frame& p_out) override {
        if (fk_should_fail_) return rocos::Result::FkCalcFail;
        // 位置 = (q[0], q[1], q[2])，旋转 = Identity
        double x = q_in.rows() > 0 ? q_in(0) : 0.0;
        double y = q_in.rows() > 1 ? q_in(1) : 0.0;
        double z = q_in.rows() > 2 ? q_in(2) : 0.0;
        p_out = rocos::Frame(rocos::Rotation::Identity(), rocos::Vector(x, y, z));
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& q_in,
                                     const rocos::Frame&,
                                     rocos::JntArray& q_out) override {
        // 透传种子作为 IK 解
        q_out = q_in;
        return rocos::Result::NoError;
    }

    /// @brief 构造简单 6×n Jacobian:
    ///        前 3 列 = XYZ 平动轴 (dof 0-2)
    ///        列 3-5 = RPY 转动轴 (dof 3-5)
    ///        列 6   = 混合列 (用于构造 1 维零空间)
    rocos::Result GetJacobian(const rocos::JntArray& q,
                              rocos::Jacobian& J_out) override {
        (void)q;
        if (jac_should_fail_) return rocos::Result::JacobianCalcFail;

        const unsigned int n = q.rows();
        J_out.resize(n);

        // 构造 Jacobian: 每列是一个 Twist
        // 列 0: prisma X  → Twist(vel=[1,0,0], rot=[0,0,0])
        // 列 1: prisma Y  → Twist(vel=[0,1,0], rot=[0,0,0])
        // 列 2: prisma Z  → Twist(vel=[0,0,1], rot=[0,0,0])
        // 列 3: revo X    → Twist(vel=[0,0,0], rot=[1,0,0])
        // 列 4: revo Y    → Twist(vel=[0,0,0], rot=[0,1,0])
        // 列 5: revo Z    → Twist(vel=[0,0,0], rot=[0,0,1])
        // 列 6+: 混合列   → Twist(vel=[0.3,0.3,0.3], rot=[0.3,0.3,0.3])

        for (unsigned int j = 0; j < n; ++j) {
            rocos::Vector vel(0.0, 0.0, 0.0), rot(0.0, 0.0, 0.0);
            switch (j) {
                case 0: vel = rocos::Vector(1.0, 0.0, 0.0); break;
                case 1: vel = rocos::Vector(0.0, 1.0, 0.0); break;
                case 2: vel = rocos::Vector(0.0, 0.0, 1.0); break;
                case 3: rot = rocos::Vector(1.0, 0.0, 0.0); break;
                case 4: rot = rocos::Vector(0.0, 1.0, 0.0); break;
                case 5: rot = rocos::Vector(0.0, 0.0, 1.0); break;
                default:
                    // 其余列为混合，构造 1 维零空间 (n=7 时有 rank 6)
                    vel = rocos::Vector(0.3, 0.3, 0.3);
                    rot = rocos::Vector(0.3, 0.3, 0.3);
                    break;
            }
            J_out.setColumn(j, rocos::Twist(vel, rot));
        }
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray& torques) override {
        if (id_should_fail_) return rocos::Result::IdCalcFail;
        for (unsigned int i = 0; i < torques.rows(); ++i) {
            torques(i) = grav_torque_;
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

    int GetJointNum() const override { return static_cast<int>(lower_limit_.rows()); }
    std::vector<std::string> GetJointNames() const override { return {}; }

    const rocos::JntArray& GetPosLowerLimit() const override { return lower_limit_; }
    const rocos::JntArray& GetPosUpperLimit() const override { return upper_limit_; }
    const rocos::JntArray& GetVelocityLimit() const override { return velocity_limit_; }
    const rocos::JntArray& GetEffortLimit() const override { return effort_limit_; }

    void SetLimits(unsigned int n, double lower, double upper,
                   double velocity, double effort) {
        lower_limit_.resize(n);
        upper_limit_.resize(n);
        velocity_limit_.resize(n);
        effort_limit_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            lower_limit_(i) = lower;
            upper_limit_(i) = upper;
            velocity_limit_(i) = velocity;
            effort_limit_(i) = effort;
        }
    }
};

// ---- 工具函数 ----

rocos::JntArray makeJntArray(unsigned int n, double start = 1.0) {
    rocos::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
        q(i) = start + static_cast<double>(i);
    }
    return q;
}

rocos::JntArray makeConstJntArray(unsigned int n, double val = 0.0) {
    rocos::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
        q(i) = val;
    }
    return q;
}

/// @brief 构造位姿：仅平移 (x,y,z)，无旋转
rocos::Frame makeFrame(double x, double y, double z) {
    return rocos::Frame(rocos::Rotation::Identity(), rocos::Vector(x, y, z));
}

}  // namespace

// 被各个测试复用的 GenerateCmd 输出参数
static rocos::JntArray dummy_q_cmd;

// ==========================================================================
// §1. 基础接口
// ==========================================================================

TEST_CASE("CartesianImpedance - SetHardware / SetModel") {
    // 控制器析构会访问依赖，硬件和模型必须先构造、后销毁。
    FakeHardware hw;
    FakeModel model;
    rocos::CartesianImpedanceController ctrl;

    SUBCASE("SetHardware nullptr → ParameterPointerEqualsNullptr") {
        CHECK(ctrl.SetHardware(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    }
    SUBCASE("SetHardware 有效 → NoError") {
        CHECK(ctrl.SetHardware(&hw) == rocos::Result::NoError);
    }
    SUBCASE("SetModel nullptr → ParameterPointerEqualsNullptr") {
        CHECK(ctrl.SetModel(nullptr) == rocos::Result::ParameterPointerEqualsNullptr);
    }
    SUBCASE("SetModel 有效 → NoError") {
        CHECK(ctrl.SetModel(&model) == rocos::Result::NoError);
    }
}

TEST_CASE("CartesianImpedance - Reset") {
    rocos::CartesianImpedanceController ctrl;
    CHECK(ctrl.Reset() == true);
}

// ==========================================================================
// §2. 参数设置
// ==========================================================================

TEST_CASE("CartesianImpedance - 笛卡尔阻抗参数") {
    rocos::CartesianImpedanceController ctrl;

    SUBCASE("SetTranslationalStiffness 正常") {
        CHECK(ctrl.SetTranslationalStiffness(1000.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTranslationalStiffness 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetTranslationalStiffness(-100.0) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetTranslationalStiffness NaN → ParameterNanOrInf") {
        CHECK(ctrl.SetTranslationalStiffness(
            std::numeric_limits<double>::quiet_NaN()) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetRotationalStiffness 正常") {
        CHECK(ctrl.SetRotationalStiffness(80.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTranslationalDamping 正常") {
        CHECK(ctrl.SetTranslationalDamping(150.0) == rocos::Result::NoError);
    }
    SUBCASE("SetRotationalDamping 正常") {
        CHECK(ctrl.SetRotationalDamping(15.0) == rocos::Result::NoError);
    }
}

TEST_CASE("CartesianImpedance - 力矩安全参数") {
    rocos::CartesianImpedanceController ctrl;

    SUBCASE("SetTorqueRateLimit 正常") {
        CHECK(ctrl.SetTorqueRateLimit(500.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueRateLimit 零值 → 有效（关闭限制）") {
        CHECK(ctrl.SetTorqueRateLimit(0.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueRateLimit 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetTorqueRateLimit(-100.0) == rocos::Result::ParameterNanOrInf);
    }
    SUBCASE("SetTorqueSaturation 正常") {
        CHECK(ctrl.SetTorqueSaturation(50.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueSaturation 零值 → 有效（关闭限制）") {
        CHECK(ctrl.SetTorqueSaturation(0.0) == rocos::Result::NoError);
    }
    SUBCASE("SetTorqueSaturation 负值 → ParameterNanOrInf") {
        CHECK(ctrl.SetTorqueSaturation(-10.0) == rocos::Result::ParameterNanOrInf);
    }
}

// ==========================================================================
// §3. GenerateCmd
// ==========================================================================

TEST_CASE("CartesianImpedance - GenerateCmd") {
    FakeHardware hw;
    FakeModel model;
    rocos::CartesianImpedanceController ctrl;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    SUBCASE("JntArray → FK 生成笛卡尔位姿") {
        auto q_in = makeConstJntArray(7, 0.1);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
        CHECK(q_cmd.rows() == 7u);
    }

    SUBCASE("Frame 直接存储") {
        rocos::Frame frame = makeFrame(0.5, 0.0, 0.0);
        rocos::Reference ref = frame;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::NoError);
        // q_cmd 由 IK 填充
        CHECK(q_cmd.rows() == 7u);
    }

    SUBCASE("空 JntArray → MoveInput") {
        rocos::Reference ref = dummy_q_cmd;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::MoveInput);
    }

    SUBCASE("JntArray 含 NaN → ParameterNanOrInf") {
        auto q_bad = makeConstJntArray(7, 0.0);
        q_bad(2) = std::numeric_limits<double>::quiet_NaN();
        rocos::Reference ref = q_bad;
        rocos::JntArray q_cmd;
        CHECK(ctrl.GenerateCmd(ref, q_cmd) == rocos::Result::ParameterNanOrInf);
    }

    SUBCASE("JntArray 无 model → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        // 不设 model
        auto q_in = makeConstJntArray(7, 0.0);
        rocos::Reference ref = q_in;
        rocos::JntArray q_cmd;
        CHECK(ctrl2.GenerateCmd(ref, q_cmd) == rocos::Result::ParameterPointerEqualsNullptr);
    }
}

// ==========================================================================
// §4. UpdateCmd — 笛卡尔阻抗力矩
// ==========================================================================

TEST_CASE("CartesianImpedance - UpdateCmd 笛卡尔阻抗力矩") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;  // 先关重力，隔离观察阻抗力矩

    SUBCASE("无 hardware → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetModel(&model);
        // 设 x_des_ 以通过 x_des_valid_ 检查
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("无 model → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("未设 x_des_ → MoveInput") {
        rocos::CartesianImpedanceController ctrl;
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // 未调用 GenerateCmd → x_des_valid_ = false
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::MoveInput);
    }

    SUBCASE("位置误差产生 X 方向力矩") {
        // FK: p_cur = (q[0], q[1], q[2]) = (0,0,0)
        // 设 x_des_.p = (1,0,0) → Δx = diff(I, Frame((1,0,0))) = Twist([1,0,0],[0,0,0])
        // F_cur = K_p_lin(1500)*[1,0,0] - K_d*0 = [1500,0,0]
        // J^T * F: col_0=[1,0,0,0,0,0] → τ[0] = 1*1500 + 0 = 1500
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        CHECK(hw.last_set_torque_.rows() == 7u);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));
    }

    SUBCASE("Y 方向误差产生对应力矩") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 1.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // col_1=[0,1,0,0,0,0] → τ[1] = 1*1500 = 1500
        CHECK(hw.last_set_torque_(1) == doctest::Approx(1500.0).epsilon(0.01));
    }

    SUBCASE("自定义刚度改变力矩比例") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetTranslationalStiffness(100.0);  // 改成 100 N/m
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // τ[0] = 100*1 = 100
        CHECK(hw.last_set_torque_(0) == doctest::Approx(100.0).epsilon(0.01));
    }

    SUBCASE("FK 与当前关节一致 → 零误差 → 零阻抗力矩") {
        // FK: p_cur = (q[0], q[1], q[2]) = (1,0,0)
        // x_des_ = Frame with p=(1,0,0) → diff ≈ 0
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_position_(0) = 1.0;  // q[0]=1 → p_cur.x = 1
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // 位置匹配 → 阻抗力矩 ≈ 0（含重力 5 Nm）
        CHECK(hw.last_set_torque_(0) == doctest::Approx(0.0).epsilon(0.01));
    }
}

// ==========================================================================
// §5. 重力补偿
// ==========================================================================

TEST_CASE("CartesianImpedance - 重力补偿") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 8.0;  // 每关节 8 Nm 重力

    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // x_des == x_cur → 零阻抗力矩
    ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                     dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    // τ = τ_imp(≈0) + τ_grav(8) = 8
    CHECK(hw.last_set_torque_(0) == doctest::Approx(8.0).epsilon(0.01));
}

// ==========================================================================
// §6. 阻尼项
// ==========================================================================

TEST_CASE("CartesianImpedance - 阻尼项") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    SUBCASE("正速度 → 阻尼产生反向力矩") {
        // v_base = J * q̇, J col_0=[1,0,0,...], q̇[0]=2 → v_base.vel.x = 2
        // v_cur = R^T * v_base, R=I → v_cur.vel.x = 2
        // F_cur.force.x = K_p*0 - K_d_lin(120)*2 = -240
        // τ[0] = 1*(-240) = -240
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_(0) = 2.0;  // q̇[0] = 2

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // x_des == x_cur → 零位置误差
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // τ[0] = K_p_lin*0 - K_d_lin*2 = -240
        CHECK(hw.last_set_torque_(0) == doctest::Approx(-240.0).epsilon(0.01));
    }

    SUBCASE("无速度反馈 → 阻尼项为零") {
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = dummy_q_cmd;  // 0 维

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        // Δx ≠ 0 → 仅刚度产生力矩
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                         dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // τ[0] = 1500*1 - 0 = 1500（无阻尼）
        CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));
    }
}

// ==========================================================================
// §7. 力矩变化率限制
// ==========================================================================

TEST_CASE("CartesianImpedance - 力矩变化率限制") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // 设一个小的变化率限制: 100 Nm/s, dt≈0.001 → max_delta = 0.1 Nm/step
    ctrl.SetTorqueRateLimit(100.0);

    // x_des 偏移 1m → 稳态力矩 = 1500 Nm
    ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                     dummy_q_cmd);

    // 第一次调用：无 tau_prev_（但 GenerateCmd 不设 tau_prev_）
    // 所以第一拍 tau_prev_valid_ 为 false，应直接输出完整力矩
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    double first_torque = hw.last_set_torque_(0);
    // 第一拍无限制，直接输出 1500
    CHECK(first_torque == doctest::Approx(1500.0).epsilon(0.01));

    // 第二次调用：tau_prev_ = first_torque
    // 此时位置仍为 (0,0,0)，x_des 仍为 (1,0,0)，稳态力矩仍为 1500
    // 但 rate limiter 已被 tau_prev_valid_ 激活
    // 由于稳态不变，Δτ = 0，力矩保持 1500
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));

    SUBCASE("变化率限制设为 0 → 不限制") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        ctrl2.SetModel(&model);
        ctrl2.SetTorqueRateLimit(0.0);  // 关闭
        ctrl2.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                          dummy_q_cmd);
        CHECK(ctrl2.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        // 直接输出完整力矩
        CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));
    }
}

// ==========================================================================
// §8. 力矩饱和
// ==========================================================================

TEST_CASE("CartesianImpedance - 力矩饱和") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;

    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.SetTorqueSaturation(3.0);  // 限制 ±3 Nm

    // x_des 偏移 1m → 原始力矩 500 Nm，应被截断到 3 Nm
    ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                     dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    // 每个关节都被限制在 ±3 Nm
    for (unsigned int i = 0; i < 7; ++i) {
        CHECK(std::abs(hw.last_set_torque_(i)) <= 3.0);
    }

    SUBCASE("饱和设为 0 → 不限制") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);
        ctrl2.SetModel(&model);
        ctrl2.SetTorqueSaturation(0.0);  // 关闭
        ctrl2.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)},
                          dummy_q_cmd);
        CHECK(ctrl2.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));
    }

    SUBCASE("负方向同样被限制") {
        // x_des 反方向偏移 → 原始力矩 -500 Nm，应被截断到 -3 Nm
        hw.fake_position_ = makeConstJntArray(7, 0.0);

        rocos::CartesianImpedanceController ctrl3;
        ctrl3.SetHardware(&hw);
        ctrl3.SetModel(&model);
        ctrl3.SetTorqueSaturation(3.0);
        ctrl3.GenerateCmd(rocos::Reference{makeFrame(-1.0, 0.0, 0.0)},
                          dummy_q_cmd);

        CHECK(ctrl3.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
        CHECK(hw.last_set_torque_(0) <= -3.0 + 0.01);
    }
}

// ==========================================================================
// §9. CST 模式管理
// ==========================================================================

TEST_CASE("CartesianImpedance - CST 模式管理") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                     dummy_q_cmd);

    SUBCASE("首次 UpdateCmd 设置 CST 模式") {
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.last_set_mode_ == 10);
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("第二次不重复设模式") {
        ctrl.UpdateCmd(makeConstJntArray(7));
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("Reset 后重新设模式") {
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.set_mode_count_ == 1);
        ctrl.Reset();
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        ctrl.UpdateCmd(makeConstJntArray(7));
        CHECK(hw.set_mode_count_ == 2);
    }
}

// ==========================================================================
// §10. 异常路径
// ==========================================================================

TEST_CASE("CartesianImpedance - 异常路径") {
    FakeHardware hw;
    FakeModel model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    SUBCASE("FK 失败 → 传播 FkCalcFail") {
        model.fk_should_fail_ = true;
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::FkCalcFail);
    }

    SUBCASE("Jacobian 失败 → 传播 JacobianCalcFail") {
        model.jac_should_fail_ = true;
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::JacobianCalcFail);
    }

    SUBCASE("InverseDynamics 失败 → 传播 IdCalcFail") {
        model.id_should_fail_ = true;
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::IdCalcFail);
    }

    SUBCASE("GetPosition 返回空 → JointStateError") {
        dummy_q_cmd = rocos::JntArray{};  // 重置为空的 JntArray
        hw.fake_position_ = dummy_q_cmd;  // 0 维
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::JointStateError);
    }
}

// ==========================================================================
// §11. 每周期 Servo validation
// ==========================================================================

TEST_CASE("CartesianImpedance - Servo validation details") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    SUBCASE("缺少 model 时拒绝且不写力矩") {
        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)}, dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ParameterPointerEqualsNullptr);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("当前位置超过 URDF 上限时返回 PosLimit") {
        hw.fake_position_ = makeConstJntArray(7, 2.0);
        model.SetLimits(7, -1.0, 1.0, 10000.0, 10000.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(2.0, 2.0, 2.0)}, dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::PosLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("当前速度超过 URDF velocity 时返回 SpeedLimit") {
        hw.fake_velocity_ = makeConstJntArray(7, 2.0);
        model.SetLimits(7, -10.0, 10.0, 1.0, 10000.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)}, dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::SpeedLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("最终力矩超过 URDF effort 时返回 ForceLimit") {
        model.SetLimits(7, -10.0, 10.0, 10000.0, 10.0);

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.SetTorqueRateLimit(0.0);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)}, dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ForceLimit);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("最终力矩非有限值时返回 ParameterNanOrInf") {
        model.grav_torque_ = std::numeric_limits<double>::quiet_NaN();

        rocos::CartesianImpedanceController ctrl;
        ctrl.SetHardware(&hw);
        ctrl.SetModel(&model);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)}, dummy_q_cmd);

        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::ParameterNanOrInf);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }
}

// ==========================================================================
// §12. SetReady 初始化
// ==========================================================================

TEST_CASE("CartesianImpedance - SetReady") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 7.0;
    hw.fake_position_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    SUBCASE("SetReady 无 hardware → ParameterPointerEqualsNullptr") {
        rocos::CartesianImpedanceController ctrl2;
        CHECK(ctrl2.SetReady() == rocos::Result::ParameterPointerEqualsNullptr);
    }

    SUBCASE("SetReady 缺少 model 时不直接写力矩") {
        rocos::CartesianImpedanceController ctrl2;
        ctrl2.SetHardware(&hw);

        CHECK(ctrl2.SetReady() == rocos::Result::ParameterPointerEqualsNullptr);
        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("SetReady 通过 UpdateCmd 执行 effort 保护") {
        model.SetLimits(7, -10.0, 10.0, 10000.0, 2.0);

        CHECK(ctrl.SetReady() == rocos::Result::ForceLimit);
        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("SetReady 成功后 CST 模式和初始力矩") {
        CHECK(ctrl.SetReady() == rocos::Result::NoError);
        // 检查力矩为重力补偿值
        CHECK(hw.last_set_torque_(0) == doctest::Approx(7.0).epsilon(0.01));
        // 检查 CST 模式已设置
        CHECK(hw.last_set_mode_ == 10);
        CHECK(hw.set_mode_count_ == 1);
    }

    SUBCASE("SetReady 后力矩安全初始化正确") {
        CHECK(ctrl.SetReady() == rocos::Result::NoError);

        // τ_prev_ 初始化为重力补偿值，变化率限制从安全起点开始
        ctrl.GenerateCmd(rocos::Reference{makeFrame(0.0, 0.0, 0.0)},
                         dummy_q_cmd);
        CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

        // 无位置误差 → τ_imp = 0 → τ = τ_grav = 7
        CHECK(hw.last_set_torque_(0) == doctest::Approx(7.0).epsilon(0.01));
    }

    SUBCASE("SetReady 覆盖之前的 Frame 目标并锁定当前位置") {
        ctrl.SetTorqueRateLimit(0.0);
        ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)}, dummy_q_cmd);

        CHECK(ctrl.SetReady() == rocos::Result::NoError);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(7.0).epsilon(0.01));
        CHECK(hw.set_torque_count_ == 1);
    }
}

TEST_CASE("CartesianImpedance - destructor routes through UpdateCmd") {
    SUBCASE("effort 保护失败时析构不直接写位置或模式") {
        FakeHardware hw;
        FakeModel model;
        model.grav_torque_ = 7.0;
        model.SetLimits(7, -10.0, 10.0, 10000.0, 2.0);
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        {
            rocos::CartesianImpedanceController ctrl;
            ctrl.SetHardware(&hw);
            ctrl.SetModel(&model);
        }

        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_position_count_ == 0);
        CHECK(hw.set_mode_count_ == 0);
        CHECK(hw.set_torque_count_ == 0);
    }

    SUBCASE("析构成功路径通过 UpdateCmd 下发力矩而不是位置") {
        FakeHardware hw;
        FakeModel model;
        model.grav_torque_ = 7.0;
        hw.fake_position_ = makeConstJntArray(7, 0.0);
        hw.fake_velocity_ = makeConstJntArray(7, 0.0);

        {
            rocos::CartesianImpedanceController ctrl;
            ctrl.SetHardware(&hw);
            ctrl.SetModel(&model);
        }

        CHECK(hw.wait_for_signal_count_ == 1);
        CHECK(hw.set_position_count_ == 0);
        CHECK(hw.set_mode_count_ == 1);
        REQUIRE(hw.set_torque_count_ == 1);
        REQUIRE(hw.last_set_torque_.rows() == 7u);
        CHECK(hw.last_set_torque_(0) == doctest::Approx(7.0).epsilon(0.01));
    }
}

// ==========================================================================
// §13. 综合场景
// ==========================================================================

TEST_CASE("CartesianImpedance - 综合：刚度 + 阻尼 + 重力") {
    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 3.0;

    // q_act = [0.1, 0, 0, 0, 0, 0, 0] → p_cur = (0.1, 0, 0)
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_position_(0) = 0.1;
    // q̇ = [0.5, 0, 0, 0, 0, 0, 0]
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_(0) = 0.5;

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.SetTranslationalStiffness(200.0);  // 200 N/m
    ctrl.SetTranslationalDamping(50.0);     // 50 N·s/m

    // x_des = (0.3, 0.2, 0.0) → p_des - p_cur = (0.2, 0.2, 0)
    ctrl.GenerateCmd(rocos::Reference{makeFrame(0.3, 0.2, 0.0)},
                     dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

    // 手动验算 τ[0]:
    //   Δx.vel = R^T*(0.2, 0.2, 0) = (0.2, 0.2, 0) [R=I]
    //   v_cur.vel = R^T * J*q̇, J*[0.5,0,0,...]^T = [0.5,0,0,0,0,0]^T → v_cur.vel = (0.5, 0, 0)
    //   F_cur.force = 200*(0.2,0.2,0) - 50*(0.5,0,0) = (40-25, 40-0, 0) = (15, 40, 0)
    //   τ_imp[0] = J_col_0^T * F = [1,0,0]^T * [15,40,0]^T = 15
    //   τ_grav[0] = 3
    //   τ_raw[0] = 15 + 3 = 18
    CHECK(hw.last_set_torque_(0) == doctest::Approx(18.0).epsilon(0.01));
    CHECK(hw.last_set_torque_.rows() == 7u);
}

// ==========================================================================
// §14. 非零末端姿态 — 验证坐标系变换正确性
// ==========================================================================

/// @brief FakeModel 变体: FK 返回绕 Z 轴旋转 90° 的姿态
class FakeModelRotated : public rocos::ModelInterface {
public:
    FakeModelRotated() { SetLimits(7, -10.0, 10.0, 10000.0, 10000.0); }

    bool fk_should_fail_{false};
    double grav_torque_{0.0};
    rocos::JntArray lower_limit_;
    rocos::JntArray upper_limit_;
    rocos::JntArray velocity_limit_;
    rocos::JntArray effort_limit_;

    rocos::Result ForwardKinematics(const rocos::JntArray& q_in,
                                    rocos::Frame& p_out) override {
        if (fk_should_fail_) return rocos::Result::FkCalcFail;
        double x = q_in.rows() > 0 ? q_in(0) : 0.0;
        double y = q_in.rows() > 1 ? q_in(1) : 0.0;
        double z = q_in.rows() > 2 ? q_in(2) : 0.0;
        // 绕 Z 轴旋转 90°: R_cur = Rz(π/2)
        p_out = rocos::Frame(rocos::Rotation::RPY(0.0, 0.0, M_PI_2),
                             rocos::Vector(x, y, z));
        return rocos::Result::NoError;
    }

    rocos::Result InverseKinematics(const rocos::JntArray& q_in,
                                     const rocos::Frame&,
                                     rocos::JntArray& q_out) override {
        q_out = q_in;
        return rocos::Result::NoError;
    }

    rocos::Result GetJacobian(const rocos::JntArray& q,
                              rocos::Jacobian& J_out) override {
        (void)q;
        const unsigned int n = q.rows();
        J_out.resize(n);
        for (unsigned int j = 0; j < n; ++j) {
            rocos::Vector vel(0.0, 0.0, 0.0), rot(0.0, 0.0, 0.0);
            switch (j) {
                case 0: vel = rocos::Vector(1.0, 0.0, 0.0); break;
                case 1: vel = rocos::Vector(0.0, 1.0, 0.0); break;
                case 2: vel = rocos::Vector(0.0, 0.0, 1.0); break;
                case 3: rot = rocos::Vector(1.0, 0.0, 0.0); break;
                case 4: rot = rocos::Vector(0.0, 1.0, 0.0); break;
                case 5: rot = rocos::Vector(0.0, 0.0, 1.0); break;
                default:
                    vel = rocos::Vector(0.3, 0.3, 0.3);
                    rot = rocos::Vector(0.3, 0.3, 0.3);
                    break;
            }
            J_out.setColumn(j, rocos::Twist(vel, rot));
        }
        return rocos::Result::NoError;
    }

    rocos::Result InverseDynamics(const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::JntArray&,
                                  const rocos::Wrenches&,
                                  rocos::JntArray& torques) override {
        for (unsigned int i = 0; i < torques.rows(); ++i) {
            torques(i) = grav_torque_;
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

    int GetJointNum() const override { return static_cast<int>(lower_limit_.rows()); }
    std::vector<std::string> GetJointNames() const override { return {}; }

    const rocos::JntArray& GetPosLowerLimit() const override { return lower_limit_; }
    const rocos::JntArray& GetPosUpperLimit() const override { return upper_limit_; }
    const rocos::JntArray& GetVelocityLimit() const override { return velocity_limit_; }
    const rocos::JntArray& GetEffortLimit() const override { return effort_limit_; }

    void SetLimits(unsigned int n, double lower, double upper,
                   double velocity, double effort) {
        lower_limit_.resize(n);
        upper_limit_.resize(n);
        velocity_limit_.resize(n);
        effort_limit_.resize(n);
        for (unsigned int i = 0; i < n; ++i) {
            lower_limit_(i) = lower;
            upper_limit_(i) = upper;
            velocity_limit_(i) = velocity;
            effort_limit_(i) = effort;
        }
    }
};

TEST_CASE("CartesianImpedance - 非零姿态: 位置误差方向正确") {
    // R_cur = Rz(90°), p_cur = (0,0,0)
    // p_des = (0, 1.0, 0) → 基坐标系下误差在 +Y 方向
    //
    // 修复后:
    //   Δx_base.vel = (0, 1.0, 0)           ← rocos::diff 在基坐标系
    //   Δx_tool.vel = Rz(-90°) * (0,1,0) = (1.0, 0, 0)  ← 变换到工具系
    //   F_cur.force  = Kp(1500) * (1,0,0) = (1500, 0, 0)    ← 工具系 +X
    //   F_base.force = Rz(90°) * (1500,0,0) = (0, 1500, 0) ← 基坐标系 +Y ✓
    //   τ[0] = J_col0^T * F_base = [1,0,0]·[0,1500,0] = 0
    //   τ[1] = J_col1^T * F_base = [0,1,0]·[0,1500,0] = 1500
    //
    // 修复前（错误）:
    //   F_cur.force  = Kp * (0,1,0) = (0, 500, 0)       ← 误当工具系
    //   F_base.force = Rz(90°) * (0,500,0) = (-500, 0, 0) ← 基坐标系 -X ✗

    FakeHardware hw;
    FakeModelRotated model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // x_des: p = (0, 1.0, 0), R = Rz(90°)（与当前姿态相同，误差仅为平移）
    rocos::Frame x_des(rocos::Rotation::RPY(0.0, 0.0, M_PI_2),
                       rocos::Vector(0.0, 1.0, 0.0));
    ctrl.GenerateCmd(rocos::Reference{x_des}, dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

    // τ[0] ≈ 0（力在 Y 方向，J_col0 在 X 方向 → 点积=0）
    CHECK(hw.last_set_torque_(0) == doctest::Approx(0.0).epsilon(0.01));
    // τ[1] ≈ 1500（力在 Y 方向，J_col1 在 Y 方向 → 点积=1500）
    CHECK(hw.last_set_torque_(1) == doctest::Approx(1500.0).epsilon(0.01));
}

TEST_CASE("CartesianImpedance - 非零姿态: 零误差 → 零阻抗力") {
    // 当前位姿 == 期望位姿 → 无论姿态如何，阻抗力应为 0
    FakeHardware hw;
    FakeModelRotated model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // x_des == x_cur: p=(0,0,0), R=Rz(90°)
    rocos::Frame x_des(rocos::Rotation::RPY(0.0, 0.0, M_PI_2),
                       rocos::Vector(0.0, 0.0, 0.0));
    ctrl.GenerateCmd(rocos::Reference{x_des}, dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    // 零位姿误差 → 所有关节的阻抗力矩部分为 0
    for (unsigned int i = 0; i < 7; ++i) {
        CHECK(hw.last_set_torque_(i) == doctest::Approx(0.0).epsilon(0.01));
    }
}

TEST_CASE("CartesianImpedance - 非零姿态: X 方向误差在旋转后仍正确") {
    // R_cur = Rz(90°), p_cur = (0,0,0)
    // p_des = (1.0, 0, 0) → 基坐标系下误差在 +X 方向
    //
    //   Δx_base.vel = (1.0, 0, 0)
    //   Δx_tool.vel = Rz(-90°) * (1,0,0) = (0, -1.0, 0)
    //   F_cur.force  = Kp(1500) * (0,-1,0) = (0, -1500, 0)
    //   F_base.force = Rz(90°) * (0,-1500,0) = (1500, 0, 0)  ← 基坐标系 +X ✓
    //   τ[0] = [1,0,0]·[1500,0,0] = 1500
    //   τ[1] = [0,1,0]·[1500,0,0] = 0

    FakeHardware hw;
    FakeModelRotated model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    rocos::Frame x_des(rocos::Rotation::RPY(0.0, 0.0, M_PI_2),
                       rocos::Vector(1.0, 0.0, 0.0));
    ctrl.GenerateCmd(rocos::Reference{x_des}, dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));
    CHECK(hw.last_set_torque_(1) == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("CartesianImpedance - 非零姿态: 合力大小不随姿态改变") {
    // 同样的位置误差幅值(1m)，不管 R_cur 是什么，阻抗力大小应相同
    // 验证: R * (Kp * R^T * e) = Kp * e（标量刚度时旋转不变）
    // |F_base| = Kp * |e| = 1500 * 1.0 = 1500

    FakeHardware hw;
    FakeModelRotated model;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    // 期望位置在基坐标系 (1,0,0)，距当前 1m
    rocos::Frame x_des(rocos::Rotation::RPY(0.0, 0.0, M_PI_2),
                       rocos::Vector(1.0, 0.0, 0.0));
    ctrl.GenerateCmd(rocos::Reference{x_des}, dummy_q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);

    // 合力: τ = J^T * F_base, J_col0=[1,0,0], J_col1=[0,1,0]
    // F_base = (1500, 0, 0) → |F| = 1500
    // τ[0]^2 + τ[1]^2 = 1500^2 + 0^2 = 2250000
    double f_sq = hw.last_set_torque_(0) * hw.last_set_torque_(0)
                + hw.last_set_torque_(1) * hw.last_set_torque_(1);
    CHECK(f_sq == doctest::Approx(2250000.0).epsilon(0.01));
}

// ==========================================================================
// §15. x_des_ 保持行为 — 运动结束后不因 hold_position_ 而跳变
// ==========================================================================

TEST_CASE("CartesianImpedance - x_des_ 保持: JntArray 不覆盖已设置的 Frame") {
    // 场景: 先通过 Frame 设置期望位姿（模拟 MoveL 的最后一帧），
    //       再用 JntArray 调用 GenerateCmd（模拟运动结束后 hold_position_），
    //       验证 x_des_ 不会被 FK(hold_position_) 覆盖。
    //
    // 这确保运动结束后机器人继续追踪运动最后的期望位姿，而非"放弃"剩余误差。

    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    // Step 1: 模拟 MoveL 最后帧 — 设置期望位姿 p=(0.5, 0, 0)
    rocos::Frame target_frame = makeFrame(0.5, 0.0, 0.0);
    ctrl.GenerateCmd(rocos::Reference{target_frame}, dummy_q_cmd);

    // Step 2: 模拟运动结束后 hold_position_ — p_cur=(0,0,0), FK→p=(0,0,0)
    // 如果 x_des_ 被覆盖，期望位姿会跳回 (0,0,0)，误差消失→力矩归零
    // 正确行为: x_des_ 保持 (0.5,0,0)，误差为 0.5m，力矩=Kp*0.5=750
    auto hold_pos = makeConstJntArray(7, 0.0);
    rocos::JntArray q_cmd;
    ctrl.GenerateCmd(rocos::Reference{hold_pos}, q_cmd);

    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    // 期望位姿应保持 p=(0.5,0,0)，力矩 = 1500 * 0.5 = 750
    CHECK(hw.last_set_torque_(0) == doctest::Approx(750.0).epsilon(0.01));
}

TEST_CASE("CartesianImpedance - x_des_ 保持: Reset 后首次 JntArray 正常设置") {
    // Reset 后 x_des_valid_=false，首次 JntArray 应正常设置 x_des_

    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);

    // Step 1: 先设一个 Frame
    ctrl.GenerateCmd(rocos::Reference{makeFrame(1.0, 0.0, 0.0)}, dummy_q_cmd);
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));

    // Step 2: Reset → x_des_valid_ = false
    ctrl.Reset();

    // Step 3: JntArray 首次调用 — 应正常用 FK 设置 x_des_ = FK((0.1,0,0,...))→p=(0.1,0,0)
    auto hold_pos = makeConstJntArray(7, 0.0);
    hold_pos(0) = 0.1;  // q[0]=0.1 → FK.p.x=0.1
    rocos::JntArray q_cmd;
    ctrl.GenerateCmd(rocos::Reference{hold_pos}, q_cmd);

    // 当前 p_cur=(0.1,0,0) = x_des_ → 误差 ≈ 0
    hw.fake_position_(0) = 0.1;  // 更新当前位置到新期望点
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("CartesianImpedance - x_des_ 保持: JntArray→JntArray 仍正常更新 (MoveJ)") {
    // 当 x_des_ 来源是 JntArray（非 Frame）时，后续 JntArray 应继续更新。
    // 这确保 MoveJ（关节空间运动）在 cart_imp 模式下仍能正常工作。

    FakeHardware hw;
    FakeModel model;
    model.grav_torque_ = 0.0;
    hw.fake_position_ = makeConstJntArray(7, 0.0);
    hw.fake_velocity_ = makeConstJntArray(7, 0.0);

    rocos::CartesianImpedanceController ctrl;
    ctrl.SetHardware(&hw);
    ctrl.SetModel(&model);
    ctrl.SetTorqueRateLimit(0.0);  // 关闭速率限制，仅测试 x_des_ 更新逻辑

    // Step 1: 初始 hold_position_ (JntArray) — q=(0,0,0), FK→p=(0,0,0)
    auto hold_init = makeConstJntArray(7, 0.0);
    rocos::JntArray q_cmd;
    ctrl.GenerateCmd(rocos::Reference{hold_init}, q_cmd);
    // x_des_ = FK(0,0,0) = (0,0,0), x_des_from_frame_ = false

    // 当前位置 = 期望位置 → 无误差
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(0.0).epsilon(0.01));

    // Step 2: 模拟 MoveJ 第一步 — q=(1,0,0,...), FK→p=(1,0,0)
    auto movej_step1 = makeConstJntArray(7, 0.0);
    movej_step1(0) = 1.0;  // q[0]=1 → FK.p.x=1
    ctrl.GenerateCmd(rocos::Reference{movej_step1}, q_cmd);
    // x_des_from_frame_ = false → x_des_ 应更新为 FK(1,0,0) = (1,0,0)

    // 当前 p_cur=(0,0,0), x_des=(1,0,0) → 误差 1m → 力矩 = 1500
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(1500.0).epsilon(0.01));

    // Step 3: 模拟 MoveJ 第二步 — q=(2,0,0,...), FK→p=(2,0,0)
    auto movej_step2 = makeConstJntArray(7, 0.0);
    movej_step2(0) = 2.0;
    ctrl.GenerateCmd(rocos::Reference{movej_step2}, q_cmd);
    // 应继续更新 x_des_ = (2,0,0)

    // 误差 2m → 力矩 = 3000
    CHECK(ctrl.UpdateCmd(makeConstJntArray(7)) == rocos::Result::NoError);
    CHECK(hw.last_set_torque_(0) == doctest::Approx(3000.0).epsilon(0.01));
}
