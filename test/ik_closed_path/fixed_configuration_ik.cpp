#include "fixed_configuration_ik.hpp"

#include <algorithm>
#include <cmath>
#include <pin_ik/pinocchio_types.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <stdexcept>

FixedConfigurationIk::FixedConfigurationIk(const pinocchio::Model& model, pinocchio::FrameIndex tip,
                                           const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
                                           const Eigen::VectorXd& anchor, double timeout, double epsilon)
    : model_(model),
      data_(model),
      tip_(tip),
      current_(model.nv),
      delta_(model.nv),
      trial_(model.nv),
      angles_(model.nv),
      configuration_(model.nq),
      jacobian_(6, model.nv),
      augmented_(model.nv, model.nv),
      decomposition_(model.nv, model.nv, Eigen::ComputeFullU | Eigen::ComputeFullV),
      residual_(model.nv),
      timeout_(timeout),
      epsilon_(epsilon) {
    if (model.nv < 6 || anchor.size() != model.nv || lower.size() != model.nv || upper.size() != model.nv ||
        tip >= model.frames.size() || !anchor.allFinite() || !lower.allFinite() || !upper.allFinite() ||
        !std::isfinite(timeout) || timeout <= 0 || !std::isfinite(epsilon) || epsilon <= 0 ||
        (lower.array() >= upper.array()).any())
        throw std::invalid_argument("Invalid fixed-configuration IK model or settings");
    lower_ = lower;
    upper_ = upper;
    anchor_ = anchor;
    if (!WithinLimits(anchor_)) throw std::invalid_argument("Anchor outside joint limits");
    PIN_IK::toPinocchioConfiguration(model_, anchor, configuration_);
    jacobian_.setZero();
    pinocchio::computeFrameJacobian(model_, data_, configuration_, tip_, pinocchio::LOCAL, jacobian_);
    Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(jacobian_, Eigen::ComputeFullV);
    if (decomposition.singularValues().minCoeff() < 1e-4) throw std::invalid_argument("Singular anchor");
    directions_ = decomposition.matrixV().rightCols(model.nv - 6);
    augmented_.bottomRows(model.nv - 6) = directions_.transpose();
}

bool FixedConfigurationIk::WithinLimits(const JointVector& q) const {
    return q.allFinite() && (q.array() >= lower_.array()).all() && (q.array() <= upper_.array()).all();
}

void FixedConfigurationIk::Evaluate(const JointVector& q, const pinocchio::SE3& target) {
    angles_ = q;
    PIN_IK::toPinocchioConfiguration(model_, angles_, configuration_);
    pinocchio::forwardKinematics(model_, data_, configuration_);
    const auto& current = pinocchio::updateFramePlacement(model_, data_, tip_);
    relative_ = current.actInv(target);
    residual_.head<6>() = pinocchio::log6(relative_).toVector();
    residual_.tail(model_.nv - 6).noalias() = -directions_.transpose() * (q - anchor_);
    position_error_ = relative_.translation().norm();
    rotation_error_ = residual_.segment<3>(3).norm();
}

int FixedConfigurationIk::Solve(const Eigen::VectorXd& seed, const pinocchio::SE3& target,
                                Eigen::VectorXd& output) {
    const auto deadline =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_));
    diagnostics_ = {};
    const auto expired = [&] { return Clock::now() >= deadline; };
    if (seed.size() != model_.nv || !seed.allFinite() || !target.translation().allFinite() ||
        !target.rotation().allFinite() ||
        !(target.rotation().transpose() * target.rotation()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
        std::abs(target.rotation().determinant() - 1.) >= 1e-8)
        return InvalidInput;
    current_ = seed;
    if (!WithinLimits(current_)) return JointLimit;

    // 1 m 特征长度下的增广矩阵。数值阈值只用于本独立实验。
    constexpr double minimum_sigma = 1e-4;
    constexpr double maximum_iteration_step = .05;
    for (int iteration = 0; iteration < 32; ++iteration) {
        if (expired()) return Timeout;
        Evaluate(current_, target);
        if (expired()) return Timeout;
        diagnostics_.configuration_error = residual_.tail(model_.nv - 6).norm();
        jacobian_.setZero();
        pinocchio::computeFrameJacobian(model_, data_, configuration_, tip_, pinocchio::LOCAL, jacobian_);
        pinocchio::Jlog6(relative_.inverse(), log_jacobian_);
        augmented_.topRows<6>().noalias() = log_jacobian_ * jacobian_;
        if (expired()) return Timeout;
        if (!augmented_.allFinite() || !residual_.allFinite()) return InvalidInput;
        decomposition_.compute(augmented_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if (expired()) return Timeout;
        const double sigma = decomposition_.singularValues().minCoeff();
        diagnostics_.min_augmented_sigma = std::min(diagnostics_.min_augmented_sigma, sigma);
        if (sigma < minimum_sigma) return Singular;
        if (position_error_ <= epsilon_ && rotation_error_ <= epsilon_ &&
            diagnostics_.configuration_error <= configuration_tolerance) {
            // 截止检查通过后一次提交，所有失败路径保留调用方原输出。
            if (expired()) return Timeout;
            output = current_;
            return Success;
        }
        delta_ = decomposition_.solve(residual_);
        if (expired()) return Timeout;
        if (!delta_.allFinite()) return InvalidInput;
        const double norm = delta_.cwiseAbs().maxCoeff();
        if (norm < 1e-14) return NoProgress;
        double alpha = std::min(1., maximum_iteration_step / norm);
        const double merit = residual_.squaredNorm();
        bool accepted = false;
        for (int backtrack = 0; backtrack < 16; ++backtrack) {
            if (expired()) return Timeout;
            trial_ = current_ + alpha * delta_;
            if (WithinLimits(trial_)) {
                Evaluate(trial_, target);
                if (expired()) return Timeout;
                if (residual_.squaredNorm() <= (1. - 1e-4 * alpha) * merit) {
                    current_ = trial_;
                    accepted = true;
                    break;
                }
            }
            alpha *= .5;
            ++diagnostics_.backtracks;
        }
        if (!accepted) return NoProgress;
        ++diagnostics_.iterations;
    }
    return expired() ? Timeout : IterationLimit;
}

const char* FixedConfigurationIk::ResultName(int result) {
    switch (result) {
        case Success:
            return "success";
        case Timeout:
            return "deadline_exceeded";
        case JointLimit:
            return "seed_joint_limit";
        case Singular:
            return "augmented_jacobian_singular";
        case NoProgress:
            return "line_search_stalled";
        case InvalidInput:
            return "invalid_input";
        case IterationLimit:
            return "iteration_limit";
        default:
            return "unknown_result";
    }
}
