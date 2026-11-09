#pragma once

#include <Eigen/SVD>
#include <chrono>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

// 独立实验求解器。模型由调用方持有，构型参考在构造后固定。
class FixedConfigurationIk {
   public:
    using JointVector = Eigen::VectorXd;
    enum Result {
        Success = 1,
        Timeout = -3,
        JointLimit = -4,
        Singular = -5,
        NoProgress = -6,
        InvalidInput = -7,
        IterationLimit = -8
    };
    struct Diagnostics {
        int iterations = 0;
        int backtracks = 0;
        double min_augmented_sigma = 1.;
        double configuration_error = 0.;
    };

    FixedConfigurationIk(const pinocchio::Model& model, pinocchio::FrameIndex tip,
                         const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
                         const Eigen::VectorXd& anchor, double timeout, double epsilon);
    int Solve(const Eigen::VectorXd& seed, const pinocchio::SE3& target, Eigen::VectorXd& output);
    const JointVector& Anchor() const { return anchor_; }
    const Eigen::MatrixXd& Directions() const { return directions_; }
    const Diagnostics& LastDiagnostics() const { return diagnostics_; }
    static const char* ResultName(int result);
    static constexpr double configuration_tolerance = 1e-10;

   private:
    using Clock = std::chrono::steady_clock;
    void Evaluate(const JointVector& q, const pinocchio::SE3& target);
    bool WithinLimits(const JointVector& q) const;

    const pinocchio::Model& model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex tip_;
    JointVector lower_, upper_, anchor_;
    Eigen::MatrixXd directions_;
    JointVector current_, delta_, trial_;
    Eigen::VectorXd angles_, configuration_;
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_;
    Eigen::Matrix<double, 6, 6> log_jacobian_;
    Eigen::MatrixXd augmented_;
    Eigen::JacobiSVD<Eigen::MatrixXd> decomposition_;
    pinocchio::SE3 relative_;
    JointVector residual_;
    double position_error_ = 0.;
    double rotation_error_ = 0.;
    double timeout_, epsilon_;
    Diagnostics diagnostics_;
};
