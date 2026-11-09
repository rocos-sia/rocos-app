// 独立第三方 IK 对照。输入使用虚拟时间，无 App、UDP 或物理仿真调用。
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "ik_closed_path/ik_benchmark_support.hpp"

namespace {
Pose targetAt(const Pose& origin, double time, const std::string& shape) {
    Pose target = origin;
    const double pi = std::acos(-1.);
    const double ramp = std::pow(std::sin(std::min(time / 5., 1.) * pi / 2), 2);
    double phase = 2 * pi * time;
    if (shape == "reverse") phase = -phase;
    if (shape == "hold") return target;
    if (shape == "line") {
        target.translation() += ramp * std::sin(phase) * Eigen::Vector3d(.01, .005, .0025);
    } else {
        target.translation() += ramp * Eigen::Vector3d(.01 * std::sin(phase), .005 * std::cos(phase),
                                                       .0025 * std::sin(phase + pi / 4));
    }
    if (shape == "pose") {
        target.linear() = origin.linear() * Eigen::AngleAxisd(.03 * ramp * std::sin(phase),
                                                              Eigen::Vector3d(1., 2., 3.).normalized())
                                                .toRotationMatrix();
    }
    return target;
}
}  // namespace

TEST_CASE("Standalone Talon closed path IK comparison") {
    const auto backend = setting("IK_DRIFT_BACKEND", "speed");
    const auto shape = setting("IK_DRIFT_SHAPE", "ellipse");
    const auto seed_mode = setting("IK_DRIFT_SEED", "previous");
    const int cycles = std::stoi(setting("IK_DRIFT_CYCLES", "20"));
    const int samples = std::stoi(setting("IK_DRIFT_SAMPLES", "1000"));
    const double timeout_ms = std::stod(setting("IK_DRIFT_TIMEOUT_MS", "1"));
    const double epsilon = std::stod(setting("IK_DRIFT_EPS", "0.00001"));
    const auto prefix = setting("IK_DRIFT_OUTPUT", "ik_closed_path_result");
    REQUIRE((backend == "speed" || backend == "distance" || backend == "newton" || backend == "sqp" ||
             backend == "anchored"));
#ifndef IK_TEST_pin
    REQUIRE(backend != "anchored");
#endif
    REQUIRE(
        (shape == "ellipse" || shape == "reverse" || shape == "line" || shape == "hold" || shape == "pose"));
    REQUIRE((seed_mode == "previous" || seed_mode == "fixed"));
    REQUIRE(cycles > 0);
    REQUIRE(samples >= 100);
    REQUIRE(std::isfinite(timeout_ms));
    REQUIRE(timeout_ms > 0);
    REQUIRE(std::isfinite(epsilon));
    REQUIRE(epsilon > 0);
    std::srand(42);
    Model model;
    Solver solver(model, backend, timeout_ms / 1000, epsilon);
    const auto origin = model.forward(model.initial);
    KDL::JntArray initial_joints(7);
    initial_joints.data = model.initial;
    KDL::Jacobian initial_jacobian(7);
    REQUIRE(model.jacobian->JntToJac(initial_joints, initial_jacobian) >= 0);
    const Eigen::VectorXd direction =
        Eigen::JacobiSVD<Eigen::MatrixXd>(initial_jacobian.data, Eigen::ComputeFullV).matrixV().col(6);
    // 共同 FK 与后端自身 FK 在起点及附近确定性样本上交叉校验。
    double fk_disagreement = 0;
    for (int i = 0; i < 100; ++i) {
        auto q = model.initial;
        for (int j = 0; j < 7; ++j) q[j] += .05 * std::sin(i * .31 + j);
        fk_disagreement = std::max(
            fk_disagreement,
            (model.forward(q).matrix() - solver.nativeForward(model, q).matrix()).cwiseAbs().maxCoeff());
    }
    REQUIRE(fk_disagreement < 1e-10);

    const std::filesystem::path output_path(prefix);
    if (!output_path.parent_path().empty()) std::filesystem::create_directories(output_path.parent_path());
    std::ofstream trace(prefix + ".csv");
    REQUIRE(trace.good());
    trace << "step,time_s,return_code,solve_us,position_error_m,rotation_error_rad,sigma_min";
    for (int j = 1; j <= 7; ++j) trace << ",q" << j;
    trace << ",configuration_error_rad,joint_step_rad,velocity_ratio,iterations,augmented_sigma";
    trace << '\n' << std::setprecision(17);

    Eigen::VectorXd accepted = model.initial;
    Eigen::VectorXd cycle_start = model.initial;
    Json checkpoints = Json::array();
    std::vector<double> times;
    const int total_steps = (5 + cycles) * samples;
    double max_position_error = 0, max_rotation_error = 0, min_sigma = model.sigmaMin(accepted);
    double max_step = 0;
    double max_configuration_error = 0, max_velocity_ratio = 0, max_cycle_drift = 0;
    double min_augmented_sigma = 1.;
    int max_iterations = 0;
    int failed_step = -1, failed_result = 0, over_budget = 0;
    for (int step = 0; step <= total_steps; ++step) {
        const double time = static_cast<double>(step) / samples;
        const Pose target = targetAt(origin, time, shape);
        const Eigen::VectorXd seed = seed_mode == "previous" ? accepted : model.initial;
        Eigen::VectorXd candidate(7);
        const auto begin = Clock::now();
        const int result = solver.solve(seed, target, candidate);
        const double us = std::chrono::duration<double, std::micro>(Clock::now() - begin).count();
        times.push_back(us);
        if (us > timeout_ms * 1000) ++over_budget;
        const bool valid = result >= 0 && candidate.size() == 7 && candidate.allFinite();
        double position_error = 0, rotation_error = 0, sigma = 0;
        double configuration_error = 0, joint_step = 0, velocity_ratio = 0, augmented_sigma = 0;
        int iterations = -1;
#ifdef IK_TEST_pin
        if (solver.fixed) {
            const auto& diagnostic = solver.fixed->LastDiagnostics();
            iterations = diagnostic.iterations;
            augmented_sigma = diagnostic.min_augmented_sigma;
            min_augmented_sigma = std::min(min_augmented_sigma, augmented_sigma);
            max_iterations = std::max(max_iterations, iterations);
        }
#endif
        bool feasible = false;
        if (valid) {
            const auto actual = model.forward(candidate);
            position_error = (actual.translation() - target.translation()).norm();
            rotation_error = Eigen::AngleAxisd(target.linear() * actual.linear().transpose()).angle();
            sigma = model.sigmaMin(candidate);
            configuration_error = direction.dot(candidate - model.initial);
            joint_step = (candidate - accepted).cwiseAbs().maxCoeff();
            velocity_ratio =
                ((candidate - accepted).cwiseAbs().array() * samples / model.velocity.array()).maxCoeff();
            feasible = (candidate.array() >= model.lower.data.array() - 1e-10).all() &&
                       (candidate.array() <= model.upper.data.array() + 1e-10).all() &&
                       position_error <= 2 * epsilon && rotation_error <= 2 * epsilon &&
                       velocity_ratio <= 1. + 1e-9;
            if (backend == "anchored") feasible = feasible && std::abs(configuration_error) <= 1e-10;
            max_position_error = std::max(max_position_error, position_error);
            max_rotation_error = std::max(max_rotation_error, rotation_error);
            min_sigma = std::min(min_sigma, sigma);
            max_step = std::max(max_step, (candidate - accepted).cwiseAbs().maxCoeff());
            max_configuration_error = std::max(max_configuration_error, std::abs(configuration_error));
            max_velocity_ratio = std::max(max_velocity_ratio, velocity_ratio);
        }
        trace << step << ',' << time << ',' << result << ',' << us << ',' << position_error << ','
              << rotation_error << ',' << sigma;
        for (int j = 0; j < 7; ++j) trace << ',' << (valid ? candidate[j] : accepted[j]);
        trace << ',' << configuration_error << ',' << joint_step << ',' << velocity_ratio << ',' << iterations
              << ',' << augmented_sigma;
        trace << '\n';
        if (!valid || !feasible) {
            failed_step = step;
            failed_result = result;
            break;
        }
        accepted = candidate;
        if (step == 5 * samples) cycle_start = accepted;
        if (step >= 5 * samples && step % samples == 0) {
            max_cycle_drift = std::max(max_cycle_drift, (accepted - cycle_start).cwiseAbs().maxCoeff());
            checkpoints.push_back({{"cycle", step / samples - 5},
                                   {"q", values(accepted)},
                                   {"drift_rad", values(accepted - cycle_start)},
                                   {"position_error_m", position_error},
                                   {"rotation_error_rad", rotation_error},
                                   {"sigma_min", sigma}});
        }
    }
    trace.close();
    REQUIRE(trace.good());
    std::sort(times.begin(), times.end());
    Json timing;
    for (const auto& quantile :
         std::vector<std::pair<std::string, double>>{{"p50", .5}, {"p99", .99}, {"p999", .999}, {"max", 1.}})
        timing[quantile.first] = times[static_cast<size_t>(quantile.second * (times.size() - 1))];
#ifdef IK_TEST_pin
    const char* library = "pin_ik";
#else
    const char* library = "trac_ik";
#endif
    Json report{{"library", library},
                {"backend", backend},
                {"shape", shape},
                {"seed", seed_mode},
                {"urdf", IK_TEST_URDF},
                {"q_initial", values(model.initial)},
                {"epsilon", epsilon},
                {"timeout_ms", timeout_ms},
                {"samples_per_cycle", samples},
                {"requested_cycles", cycles},
                {"ramp_seconds", 5},
                {"calls", times.size()},
                {"failed_step", failed_step},
                {"failed_result", failed_result},
                {"failed_reason", failed_step < 0 ? "none"
                                                  : (failed_result >= 0 ? "candidate_validation_failed"
                                                                        : solver.resultName(failed_result))},
                {"completed", failed_step < 0},
                {"solve_us", timing},
                {"over_budget", over_budget},
                {"fk_disagreement_max", fk_disagreement},
                {"max_position_error_m", max_position_error},
                {"max_rotation_error_rad", max_rotation_error},
                {"min_sigma_min", min_sigma},
                {"max_joint_step_rad", max_step},
                {"max_configuration_error_rad", max_configuration_error},
                {"configuration_direction", values(direction)},
                {"max_velocity_ratio", max_velocity_ratio},
                {"max_cycle_drift_rad", max_cycle_drift},
                {"min_augmented_sigma", backend == "anchored" ? Json(min_augmented_sigma) : Json(nullptr)},
                {"max_iterations", backend == "anchored" ? Json(max_iterations) : Json(nullptr)},
                {"checkpoints", checkpoints},
                {"application_jerk_baseline", 10000},
                {"pipeline", "isolated IK, virtual time"}};
    std::ofstream summary(prefix + ".json");
    summary << report.dump(2) << '\n';
    summary.close();
    REQUIRE(summary.good());
    std::cout << library << ' ' << backend << ' ' << shape << " calls=" << times.size()
              << " completed=" << (failed_step < 0) << " sigma_min=" << min_sigma
              << " drift=" << (accepted - cycle_start).transpose() << '\n';
    INFO("Result file " << prefix << ".json, failed step " << failed_step << ", code " << failed_result);
    CHECK(failed_step < 0);
    // 仅用于本 Talon 非奇异测试邻域的检查，防止轨迹退化后继续累计漂移。
    CHECK(min_sigma > .2);
    if (backend == "anchored") {
        CHECK(max_configuration_error <= 1e-10);
        CHECK(max_cycle_drift <= 4 * epsilon / std::max(min_augmented_sigma, 1e-4));
    }
}

#ifdef IK_TEST_pin
TEST_CASE("Six-axis AJM uses zero extra constraints") {
    Model model(IK_TEST_ARM6, "base", "tip");
    Solver solver(model, "anchored", .001, 1e-5);
    REQUIRE(model.initial.size() == 6);
    CHECK(solver.fixed->Directions().cols() == 0);
    Eigen::VectorXd witness = model.initial;
    witness[0] += .0002;
    witness[3] -= .0001;
    const Pose target = model.forward(witness);
    Eigen::VectorXd output(6);
    REQUIRE(solver.solve(model.initial, target, output) == FixedConfigurationIk::Success);
    const Pose actual = model.forward(output);
    CHECK((actual.translation() - target.translation()).norm() <= 1e-5);
    CHECK(Eigen::AngleAxisd(target.linear() * actual.linear().transpose()).angle() <= 1e-5);
    CHECK(solver.fixed->LastDiagnostics().configuration_error == 0.);
}

TEST_CASE("Fixed configuration IK rejects faults without committing output") {
    Model model;
    Solver solver(model, "anchored", .001, 1e-5);
    const auto origin = model.forward(model.initial);
    const pinocchio::SE3 target(origin.linear(), origin.translation());
    Eigen::VectorXd output = Eigen::VectorXd::Constant(7, 123.);
    Eigen::VectorXd seed = model.initial;
    seed[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK(solver.fixed->Solve(seed, target, output) == FixedConfigurationIk::InvalidInput);
    CHECK((output.array() == 123.).all());
    seed = model.initial;
    seed[0] = model.upper(0) + 1.;
    CHECK(solver.fixed->Solve(seed, target, output) == FixedConfigurationIk::JointLimit);
    CHECK((output.array() == 123.).all());
    auto invalid_target = target;
    invalid_target.rotation()(0, 0) = 10.;
    CHECK(solver.fixed->Solve(model.initial, invalid_target, output) == FixedConfigurationIk::InvalidInput);
    CHECK((output.array() == 123.).all());
    auto unreachable = target;
    unreachable.translation().x() += 100.;
    CHECK(solver.fixed->Solve(model.initial, unreachable, output) < 0);
    CHECK((output.array() == 123.).all());
    CHECK(solver.fixed->Anchor().isApprox(model.initial, 0.));
    CHECK(solver.fixed->Solve(model.initial, target, output) == FixedConfigurationIk::Success);
    CHECK(output.isApprox(model.initial, 1e-12));
    Model singular_model;
    singular_model.initial.setZero();
    REQUIRE(singular_model.sigmaMin(singular_model.initial) < 1e-4);
    CHECK_THROWS_AS(Solver(singular_model, "anchored", .001, 1e-5), std::invalid_argument);
}
#endif
