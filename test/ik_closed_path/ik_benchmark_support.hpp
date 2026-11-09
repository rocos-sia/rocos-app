#pragma once
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <json.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#ifdef IK_TEST_pin
#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "fixed_configuration_ik.hpp"
#else
#include <trac_ik/trac_ik.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using Pose = Eigen::Isometry3d;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

std::string setting(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    return value ? value : fallback;
}

std::vector<double> values(const Eigen::VectorXd& q) { return {q.data(), q.data() + q.size()}; }

Pose fromKdl(const KDL::Frame& frame) {
    Pose pose = Pose::Identity();
    for (int r = 0; r < 3; ++r) {
        pose.translation()[r] = frame.p[r];
        for (int c = 0; c < 3; ++c) pose.linear()(r, c) = frame.M(r, c);
    }
    return pose;
}

KDL::Frame toKdl(const Pose& pose) {
    KDL::Frame frame;
    for (int r = 0; r < 3; ++r) {
        frame.p[r] = pose.translation()[r];
        for (int c = 0; c < 3; ++c) frame.M(r, c) = pose.linear()(r, c);
    }
    return frame;
}

struct Model {
    KDL::Chain chain;
    KDL::JntArray lower, upper;
    Eigen::VectorXd initial;
    Eigen::VectorXd velocity;
    std::string base_link, tip_link, urdf_path;
    std::string xml;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian;

    Model(const std::string& path = IK_TEST_URDF, const std::string& base = "base_link",
          const std::string& tip = "link_7")
        : base_link(base), tip_link(tip), urdf_path(path) {
        std::ifstream input(path);
        if (!input) throw std::runtime_error("Cannot open robot URDF");
        xml.assign(std::istreambuf_iterator<char>(input), {});
        auto urdf = urdf::parseURDF(xml);
        KDL::Tree tree;
        if (!urdf || !kdl_parser::treeFromUrdfModel(*urdf, tree) ||
            !tree.getChain(base_link, tip_link, chain) || chain.getNrOfJoints() < 6)
            throw std::runtime_error("Invalid robot chain");
        const auto count = chain.getNrOfJoints();
        lower.resize(count);
        upper.resize(count);
        initial.resize(count);
        velocity.resize(count);
        unsigned int axis = 0;
        for (const auto& segment : chain.segments) {
            if (segment.getJoint().getType() == KDL::Joint::None) continue;
            const auto joint = urdf->getJoint(segment.getJoint().getName());
            if (!joint || !joint->limits) throw std::runtime_error("Missing joint limits");
            lower(axis) = joint->limits->lower;
            velocity(axis) = joint->limits->velocity;
            upper(axis++) = joint->limits->upper;
        }
        if (count == 7) {
            initial << 0.015154451601302701, 0.7853981633974483, 0.0024351944580752126, -1.5707963267948966,
                -0.018880746533213064, 1.5707963267948966, -0.0017947575133036402;
        } else {
            initial = (lower.data + upper.data) * .5;
            initial[1] = -.7;
            initial[2] = 1.1;
            initial[4] = -.5;
        }
        fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
        jacobian = std::make_unique<KDL::ChainJntToJacSolver>(chain);
    }

    Pose forward(const Eigen::VectorXd& q) const {
        KDL::JntArray input(chain.getNrOfJoints());
        input.data = q;
        KDL::Frame frame;
        if (fk->JntToCart(input, frame) < 0) throw std::runtime_error("FK failed");
        return fromKdl(frame);
    }

    double sigmaMin(const Eigen::VectorXd& q) const {
        KDL::JntArray input(chain.getNrOfJoints());
        input.data = q;
        KDL::Jacobian matrix(chain.getNrOfJoints());
        if (jacobian->JntToJac(input, matrix) < 0) throw std::runtime_error("Jacobian failed");
        return Eigen::JacobiSVD<Eigen::MatrixXd>(matrix.data).singularValues().minCoeff();
    }
};

struct Solver {
#ifdef IK_TEST_pin
    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    pinocchio::FrameIndex tip{};
    std::unique_ptr<PIN_IK::PIN_IK> combined;
    std::unique_ptr<PIN_IK::ChainIkSolverPos_TL> newton;
    std::unique_ptr<FixedConfigurationIk> fixed;
#else
    std::unique_ptr<TRAC_IK::TRAC_IK> combined;
    std::unique_ptr<KDL::ChainIkSolverPos_TL> newton;
#endif
    std::unique_ptr<NLOPT_IK::NLOPT_IK> sqp;

    Solver(const Model& common, const std::string& backend, double timeout, double epsilon) {
#ifdef IK_TEST_pin
        Eigen::VectorXd lower, upper;
        PIN_IK::loadURDFModel(common.xml, common.base_link, common.tip_link, model, lower, upper, tip);
        if (!lower.isApprox(common.lower.data, 1e-12) || !upper.isApprox(common.upper.data, 1e-12))
            throw std::runtime_error("Backend joint limits differ");
        data = std::make_unique<pinocchio::Data>(model);
        if (backend == "anchored")
            fixed = std::make_unique<FixedConfigurationIk>(model, tip, lower, upper, common.initial, timeout,
                                                           epsilon);
        else if (backend == "newton")
            newton = std::make_unique<PIN_IK::ChainIkSolverPos_TL>(model, lower, upper, tip, timeout, epsilon,
                                                                   true, true);
        else if (backend == "sqp")
            sqp = std::make_unique<NLOPT_IK::NLOPT_IK>(model, lower, upper, tip, timeout, epsilon,
                                                       NLOPT_IK::SumSq);
        else
            combined =
                std::make_unique<PIN_IK::PIN_IK>(model, lower, upper, tip, timeout, epsilon,
                                                 backend == "distance" ? PIN_IK::Distance : PIN_IK::Speed);
        if (combined) combined->setRandomSeed(42);
#else
        if (backend == "newton")
            newton = std::make_unique<KDL::ChainIkSolverPos_TL>(common.chain, common.lower, common.upper,
                                                                timeout, epsilon, true, true);
        else if (backend == "sqp")
            sqp = std::make_unique<NLOPT_IK::NLOPT_IK>(common.chain, common.lower, common.upper, timeout,
                                                       epsilon, NLOPT_IK::SumSq);
        else
            combined = std::make_unique<TRAC_IK::TRAC_IK>(
                common.chain, common.lower, common.upper, timeout, epsilon,
                backend == "distance" ? TRAC_IK::Distance : TRAC_IK::Speed);
#endif
    }

    int solve(const Eigen::VectorXd& seed, const Pose& target, Eigen::VectorXd& out) {
#ifdef IK_TEST_pin
        const pinocchio::SE3 pose(target.linear(), target.translation());
        if (fixed) return fixed->Solve(seed, pose, out);
        if (newton) return newton->CartToJnt(seed, pose, out);
        if (sqp) return sqp->CartToJnt(seed, pose, out);
        return combined->CartToJnt(seed, pose, out);
#else
        KDL::JntArray input(seed.size()), output(seed.size());
        input.data = seed;
        const auto pose = toKdl(target);
        int result;
        if (newton)
            result = newton->CartToJnt(input, pose, output);
        else if (sqp)
            result = sqp->CartToJnt(input, pose, output);
        else
            result = combined->CartToJnt(input, pose, output);
        out = output.data;
        return result;
#endif
    }
    const char* resultName(int result) const {
#ifdef IK_TEST_pin
        if (fixed) return FixedConfigurationIk::ResultName(result);
#endif
        return result >= 0 ? "success" : "library_solve_failed";
    }

    Pose nativeForward(const Model& common, const Eigen::VectorXd& q) {
#ifdef IK_TEST_pin
        pinocchio::forwardKinematics(model, *data, PIN_IK::toPinocchioConfiguration(model, q));
        const auto& frame = pinocchio::updateFramePlacement(model, *data, tip);
        Pose pose = Pose::Identity();
        pose.linear() = frame.rotation();
        pose.translation() = frame.translation();
        return pose;
#else
        return common.forward(q);
#endif
    }
};

}  // namespace
