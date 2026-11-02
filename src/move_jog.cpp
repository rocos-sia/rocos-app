//
// Created by think on 2026/7/7.
//

#include "move_jog.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace rocos {

namespace {
constexpr double kSpeedEpsilon = 1e-6;

double vecNorm(const JogVec& v) noexcept {
    if (std::holds_alternative<JntArray>(v)) {
        const auto& q = std::get<JntArray>(v);
        double s = 0.0;
        for (unsigned int i = 0; i < q.rows(); ++i) s += q(i) * q(i);
        return std::sqrt(s);
    }
    const auto& tw = std::get<Twist>(v);
    double s = 0.0;
    for (int i = 0; i < 3; ++i) s += tw.vel(i) * tw.vel(i) + tw.rot(i) * tw.rot(i);
    return std::sqrt(s);
}

bool vecIsFinite(const JogVec& v) noexcept {
    if (std::holds_alternative<JntArray>(v)) {
        for (unsigned int i = 0; i < std::get<JntArray>(v).rows(); ++i)
            if (!std::isfinite(std::get<JntArray>(v)(i))) return false;
        return true;
    }
    const auto& tw = std::get<Twist>(v);
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(tw.vel(i)) || !std::isfinite(tw.rot(i))) return false;
    return true;
}
} // namespace

// ============================================================================
// 归一化方向
// ============================================================================

double MoveJog::normalizeDirection(const JogVec& src, JogVec& dst) {
    const double norm = vecNorm(src);
    if (norm < kSpeedEpsilon) return 0.0;

    if (std::holds_alternative<JntArray>(src)) {
        const auto& q = std::get<JntArray>(src);
        JntArray dir(q.rows());
        for (unsigned int i = 0; i < q.rows(); ++i) dir(i) = q(i) / norm;
        dst = std::move(dir);
    } else {
        const auto& tw = std::get<Twist>(src);
        Twist dir;
        for (int i = 0; i < 3; ++i) {
            dir.vel(i) = tw.vel(i) / norm;
            dir.rot(i) = tw.rot(i) / norm;
        }
        dst = dir;
    }
    return norm;
}

double MoveJog::directionCosine(const JogVec& a, const JogVec& b) {
    // 调用方保证 a、b 都是归一化向量（模长=1），cos = dot(a,b)
    if (std::holds_alternative<JntArray>(a)) {
        const auto& qa = std::get<JntArray>(a);
        const auto& qb = std::get<JntArray>(b);
        double s = 0.0;
        for (unsigned int i = 0; i < qa.rows(); ++i) s += qa(i) * qb(i);
        return s;
    }
    const auto& ta = std::get<Twist>(a);
    const auto& tb = std::get<Twist>(b);
    double s = 0.0;
    for (int i = 0; i < 3; ++i) s += ta.vel(i) * tb.vel(i) + ta.rot(i) * tb.rot(i);
    return s;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

MoveJog::MoveJog(double dt, double timeout, ModelInterface* model, double dir_threshold)
    : MotionInterface(model)
    , dt_(dt)
    , timeout_(timeout)
    , dir_threshold_(dir_threshold)
    , speed_otg_(dt) {}

MoveJog::~MoveJog() = default;

// ============================================================================
// 参数校验
// ============================================================================

Result MoveJog::ValidateParameters() const {
    if (!std::isfinite(dt_) || dt_ <= 0.0) return Result::ParameterNanOrInf;
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) return Result::ParameterNanOrInf;
    return Result::NoError;
}

// ============================================================================
// 初始化 — 等待第一次 FeedJog 后再启动
// ============================================================================

Result MoveJog::Reset() {
    std::lock_guard<std::mutex> lock(mtx_);

    // 外部必须先调用 setInitialPosition 设置当前关节角
    if (q_integral_.rows() == 0) {
        return Result::IllegalParameter;
    }

    // 无条件初始化 OTG 到安全状态（即使尚未 FeedJog）
    // 避免 Update() 收到 has_started_=true 后裸奔未初始化的 speed_input_
    speed_otg_current_ = 0.0;
    speed_input_.control_interface = ruckig::ControlInterface::Velocity;
    speed_input_.current_velocity     = {0.0};
    speed_input_.current_acceleration = {0.0};
    speed_input_.target_velocity      = {target_speed_};
    speed_input_.max_velocity         = {v_max_};
    speed_input_.max_acceleration     = {a_max_};
    speed_input_.max_jerk             = {j_max_};

    // 清空残留状态，允许实例复用
    direction_        = JogVec{};
    target_speed_     = 0.0;
    is_joint_space_   = false;
    elapsed_since_feed_ = 0.0;
    state_.store(State::Active);
    has_started_.store(false);

    return Result::NoError;
}

// ============================================================================
// 设置初始关节位置（Reset 之前由 Robot 调用）
// ============================================================================

void MoveJog::setInitialPosition(const JntArray& q) {
    joint_count_ = static_cast<int>(q.rows());
    q_integral_ = q;
    jacobian_.resize(static_cast<unsigned int>(joint_count_));
}

// ============================================================================
// FeedJog — HTTP 线程旁路写入
// ============================================================================

Result MoveJog::FeedJog(const JogVec& direction, double speed) {
    if (state_.load() != State::Active) return Result::PlanError;
    if (!std::isfinite(speed)) return Result::ParameterNanOrInf;
    if (!vecIsFinite(direction)) return Result::ParameterNanOrInf;

    JogVec unit_dir;
    if (normalizeDirection(direction, unit_dir) < kSpeedEpsilon)
        return Result::IllegalParameter;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (state_.load() != State::Active) return Result::PlanError;

        const bool is_first = (vecNorm(direction_) < kSpeedEpsilon);

        // 首次 FeedJog → 锁定坐标系类型
        if (is_first) {
            is_joint_space_ = std::holds_alternative<JntArray>(direction);
        }
        // 运动中检测坐标系切换 → 拒绝
        else if (std::holds_alternative<JntArray>(direction) != is_joint_space_) {
            return Result::IllegalParameter;
        }
        // 运动中检测方向突变：余弦相似度 < 阈值 → 拒绝（不重置时间戳，看门狗继续跑）
        else if (directionCosine(direction_, unit_dir) < dir_threshold_) {
            return Result::PlanError;  // 不更新 elapsed_since_feed_，超时后自动刹车
        }

        direction_ = std::move(unit_dir);
        target_speed_ = std::abs(speed);
        elapsed_since_feed_ = 0.0;
        has_started_.store(true);
    }
    return Result::NoError;
}

// ============================================================================
// 速度 OTG 配置（控制线程调用）
// ============================================================================

void MoveJog::reconfigureSpeedOtg(double target_speed) {
    // 只更新目标速度和限制，不动 current_velocity / current_acceleration。
    // 当前状态由 speed_output_.pass_to_input(speed_input_) 在 Update 中负责传递，
    // 这里重置会导致 Ruckig 认为每周期加速度都是 0，Jerk 限制失效。
    speed_input_.control_interface = ruckig::ControlInterface::Velocity;
    speed_input_.target_velocity = {target_speed};
    speed_input_.max_velocity = {v_max_};
    speed_input_.max_acceleration = {a_max_};
    speed_input_.max_jerk = {j_max_};
}

// ============================================================================
// GenerateRef — 控制线程每周期调用（内含看门狗 + OTG 单步推进）
// ============================================================================

Result MoveJog::GenerateRef(Reference& ref_out) {
    if (state_.load() == State::Stopped) {
        ref_out = q_integral_;
        return Result::PlanFinished;
    }
    if (!has_started_.load()) {
        ref_out = q_integral_;
        return Result::NoError;
    }

    // ── 看门狗：超时触发减速 ──
    double target;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        elapsed_since_feed_ += dt_;
        if (elapsed_since_feed_ > timeout_ && state_.load() == State::Active) {
            state_.store(State::Decelerating);
            target_speed_ = 0.0;
        }
        target = target_speed_;
    }

    // ── 推进速度 OTG ──
    reconfigureSpeedOtg(target);
    const auto res = speed_otg_.update(speed_input_, speed_output_);

    if (res == ruckig::Result::Error) {
        state_.store(State::Stopped);
        return Result::PlanError;
    }

    speed_otg_current_ = speed_output_.new_velocity[0];
    speed_output_.pass_to_input(speed_input_);

    // ── 减速完成 → 停车 ──
    const bool plan_finished = (state_.load() == State::Decelerating &&
                                std::abs(speed_otg_current_) < kSpeedEpsilon);
    if (plan_finished) {
        state_.store(State::Stopped);
    }

    JntArray q_dot = computeJointVelocity();
    for (unsigned int i = 0; i < q_integral_.rows(); ++i)
        q_integral_(i) += q_dot(i) * dt_;

    ref_out = q_integral_;
    return plan_finished ? Result::PlanFinished : Result::NoError;
}

JntArray MoveJog::computeJointVelocity() {
    std::lock_guard<std::mutex> lock(mtx_);
    const double s = speed_otg_current_;
    JntArray q_dot(static_cast<unsigned int>(joint_count_));

    if (is_joint_space_) {
        const auto& dir = std::get<JntArray>(direction_);
        for (unsigned int i = 0; i < dir.rows(); ++i)
            q_dot(i) = dir(i) * s;
    } else {
        // 笛卡尔空间：Jacobian 伪逆
        const auto& tw = std::get<Twist>(direction_);
        rocos::Twist cart_vel(tw.vel * s, tw.rot * s);

        if (model_) {
            model_->GetJacobian(q_integral_, jacobian_);

            // SVD pinv: q_dot = J⁺ v
            const auto& J = jacobian_.data;
            Eigen::VectorXd v(6);
            for (int i = 0; i < 3; ++i) { v(i) = cart_vel.vel(i); v(i+3) = cart_vel.rot(i); }

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(
                J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const double tol = 1e-6;
            const Eigen::VectorXd singular = svd.singularValues();
            Eigen::VectorXd inv_singular(singular.size());
            for (int i = 0; i < singular.size(); ++i) {
                inv_singular(i) = singular(i) > tol ? 1.0 / singular(i) : 0.0;
            }

            Eigen::VectorXd qd =
                svd.matrixV() * inv_singular.asDiagonal() * svd.matrixU().transpose() * v;
            for (unsigned int i = 0; i < q_dot.rows(); ++i) q_dot(i) = qd(static_cast<int>(i));
        }
    }
    return q_dot;
}


// ============================================================================
// Pause / Resume / Stop
// ============================================================================

Result MoveJog::Pause()  { return Result::PlanError; }
Result MoveJog::Resume() { return Result::PlanError; }

Result MoveJog::Stop() {
    state_.store(State::Decelerating);
    std::lock_guard<std::mutex> lock(mtx_);
    target_speed_ = 0.0;
    return Result::NoError;
}

} // namespace rocos
