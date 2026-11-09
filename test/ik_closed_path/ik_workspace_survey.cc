#include <random>

#include "ik_benchmark_support.hpp"

namespace {
Eigen::MatrixXd nullBasis(const Model& model, const Eigen::VectorXd& q) {
    KDL::JntArray input(q.size());
    input.data = q;
    KDL::Jacobian jacobian(q.size());
    if (model.jacobian->JntToJac(input, jacobian) < 0) throw std::runtime_error("Jacobian failed");
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian.data, Eigen::ComputeFullV);
    return svd.matrixV().rightCols(q.size() - 6);
}

double fitScale(const Model& model, const Eigen::VectorXd& q, const Eigen::VectorXd& delta) {
    double scale = 1.;
    for (int j = 0; j < q.size(); ++j) {
        if (delta[j] > 0) scale = std::min(scale, (model.upper(j) - q[j]) / delta[j]);
        if (delta[j] < 0) scale = std::min(scale, (model.lower(j) - q[j]) / delta[j]);
    }
    return .95 * scale;
}

Json quantiles(std::vector<double> values) {
    if (values.empty()) return nullptr;
    std::sort(values.begin(), values.end());
    Json result;
    for (const auto& p :
         std::vector<std::pair<std::string, double>>{{"p50", .5}, {"p99", .99}, {"p999", .999}, {"max", 1.}})
        result[p.first] = values[static_cast<size_t>(p.second * (values.size() - 1))];
    return result;
}
}  // namespace

int main() try {
    const auto model_name = setting("IK_SURVEY_MODEL", "talon");
    const auto backend = setting("IK_DRIFT_BACKEND", "speed");
    const auto kind = setting("IK_SURVEY_KIND", "global");
    const int count = std::stoi(setting("IK_SURVEY_COUNT", kind == "loops" ? "64" : "2000"));
    const int cycles = 3, samples = 500;
    constexpr double timeout = .001, epsilon = 1e-5, dt = .001;
    const auto prefix = setting("IK_DRIFT_OUTPUT", "workspace_survey");
    if ((model_name != "talon" && model_name != "arm6") || count < 1 ||
        (backend != "speed" && backend != "distance" && backend != "anchored") ||
        (kind != "global" && kind != "constrained" && kind != "warm" && kind != "loops"))
        throw std::invalid_argument("Invalid survey settings");
#ifndef IK_TEST_pin
    if (backend == "anchored") throw std::invalid_argument("AJM uses Pinocchio");
#endif
    Model model(model_name == "talon" ? IK_TEST_URDF : IK_TEST_ARM6,
                model_name == "talon" ? "base_link" : "base", model_name == "talon" ? "link_7" : "tip");
    std::srand(42);
    const auto setup_start = Clock::now();
    Solver solver(model, backend, timeout, epsilon);
    const double setup_us = std::chrono::duration<double, std::micro>(Clock::now() - setup_start).count();
    const auto n = model.initial.size();
    std::mt19937_64 random(20261109);
    std::uniform_real_distribution<double> uniform(0., 1.);
    const auto randomJoint = [&] {
        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j)
            q[j] = model.lower(j) + uniform(random) * (model.upper(j) - model.lower(j));
        return q;
    };
    const auto randomDirection = [&] {
        Eigen::VectorXd q(n);
        for (int j = 0; j < n; ++j) q[j] = 2 * uniform(random) - 1;
        return q;
    };
    const auto parent = std::filesystem::path(prefix).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream trace(prefix + ".csv");
    if (!trace) throw std::runtime_error("Cannot open trace");
    trace << "case,step,result,solve_us,pose_ok,constraint_ok,on_time,position_error_m,rotation_error_rad,h_"
             "rad,joint_step_rad,velocity_ratio";
    for (int j = 0; j < n; ++j) trace << ",q" << j + 1;
    trace << '\n' << std::setprecision(17);
    Json trials = Json::array(), inputs = Json::array();
    std::vector<double> solve_times, successful_times, anchor_times;
    int calls = 0, pose_successes = 0, strict_successes = 0, on_time_successes = 0;
    int constraint_successes = 0, completed = 0, initialization_failures = 0, overruns = 0;
    double max_fk_disagreement = 0, max_position = 0, max_rotation = 0, max_witness_h = 0;
    for (int index = 0; index < count; ++index) {
        Eigen::VectorXd anchor = randomJoint();
        if (index % 10 == 0) {
            const int axis = (index / 10) % n;
            anchor[axis] = model.lower(axis) +
                           (index % 20 == 0 ? 1e-5 : 1. - 1e-5) * (model.upper(axis) - model.lower(axis));
        }
        if (index % 10 == 1)
            for (int j = 0; j < n; ++j)
                anchor[j] = std::clamp((uniform(random) - .5) * 2e-4, model.lower(j), model.upper(j));
        const Eigen::MatrixXd basis = nullBasis(model, anchor);
        Eigen::VectorXd direction = randomDirection();
        direction -= basis * (basis.transpose() * direction);
        Eigen::VectorXd second = randomDirection();
        second -= basis * (basis.transpose() * second);
        direction /= std::max(direction.norm(), 1e-15);
        second /= std::max(second.norm(), 1e-15);
        Eigen::VectorXd witness = randomJoint();
        if (kind == "constrained" || kind == "warm") {
            Eigen::VectorXd delta = direction * (kind == "warm" ? .0005 : 3.);
            witness = anchor + fitScale(model, anchor, delta) * delta;
        }
        const double sigma = model.sigmaMin(anchor);
        double margin = 1.;
        for (int j = 0; j < n; ++j)
            margin = std::min(margin, std::min(anchor[j] - model.lower(j), model.upper(j) - anchor[j]) /
                                          (model.upper(j) - model.lower(j)));
        Eigen::VectorXd amplitude = direction.cwiseAbs() * .01 + second.cwiseAbs() * .02;
        double orbit_scale = 1.;
        for (int j = 0; j < n; ++j)
            orbit_scale =
                std::min(orbit_scale, .8 * std::min(anchor[j] - model.lower(j), model.upper(j) - anchor[j]) /
                                          std::max(amplitude[j], 1e-15));
        inputs.push_back({{"index", index},
                          {"anchor", values(anchor)},
                          {"witness", values(witness)},
                          {"direction", values(direction)},
                          {"second", values(second)},
                          {"orbit_scale", orbit_scale}});
        Json trial{{"index", index},
                   {"seed_sigma", sigma},
                   {"normalized_joint_margin", margin},
                   {"orbit_scale", orbit_scale},
                   {"completed", false},
                   {"failure", "none"},
                   {"calls", 0},
                   {"pose_successes", 0},
                   {"constraint_successes", 0},
                   {"on_time_successes", 0}};
        trial["witness_joint_distance_rad"] = (witness - anchor).norm();
        double anchor_us = 0.;
#ifdef IK_TEST_pin
        if (backend == "anchored") {
            const auto start = Clock::now();
            try {
                solver.fixed = std::make_unique<FixedConfigurationIk>(
                    solver.model, solver.tip, model.lower.data, model.upper.data, anchor, timeout, epsilon);
            } catch (const std::invalid_argument& error) {
                trial["failure"] = error.what();
                ++initialization_failures;
            }
            anchor_us = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
            anchor_times.push_back(anchor_us);
        }
#endif
        trial["anchor_setup_us"] = anchor_us;
        if (trial["failure"] != "none") {
            trials.push_back(trial);
            continue;
        }
        Eigen::VectorXd accepted = anchor, candidate(n);
        double closure = 0., max_step = 0., max_h = 0., max_velocity_ratio = 0.;
        Json checkpoints = Json::array();
        bool finished = true;
        const int steps = kind == "loops" ? cycles * samples + 1 : 1;
        for (int step = 0; step < steps; ++step) {
            Eigen::VectorXd target_joints = witness;
            if (kind == "loops") {
                const double phase = 2 * std::acos(-1.) * step / samples;
                target_joints = anchor + orbit_scale * (.01 * std::sin(phase) * direction +
                                                        .01 * (1 - std::cos(phase)) * second);
            }
            const Pose target = model.forward(target_joints);
            if ((target_joints.array() < model.lower.data.array() - 1e-12).any() ||
                (target_joints.array() > model.upper.data.array() + 1e-12).any())
                throw std::runtime_error("Witness outside joint limits");
            if (kind != "global") {
                max_witness_h =
                    std::max(max_witness_h, (basis.transpose() * (target_joints - anchor)).norm());
                if (max_witness_h > 1e-10) throw std::runtime_error("Witness violates fixed configuration");
            }
            max_fk_disagreement = std::max(
                max_fk_disagreement, (target.matrix() - solver.nativeForward(model, target_joints).matrix())
                                         .cwiseAbs()
                                         .maxCoeff());
            const auto start = Clock::now();
            const int result = solver.solve(accepted, target, candidate);
            const double us = std::chrono::duration<double, std::micro>(Clock::now() - start).count();
            solve_times.push_back(us);
            ++calls;
            trial["calls"] = trial["calls"].get<int>() + 1;
            const bool on_time = us <= timeout * 1e6;
            if (!on_time) ++overruns;
            double position = 0., rotation = 0., h = 0., joint_step = 0., velocity_ratio = 0.;
            bool pose_ok = false, constraint_ok = false;
            if (result >= 0 && candidate.size() == n && candidate.allFinite()) {
                const auto actual = model.forward(candidate);
                position = (actual.translation() - target.translation()).norm();
                rotation = Eigen::AngleAxisd(target.linear() * actual.linear().transpose()).angle();
                h = (basis.transpose() * (candidate - anchor)).norm();
                pose_ok = position <= 2 * epsilon && rotation <= 2 * epsilon &&
                          (candidate.array() >= model.lower.data.array() - 1e-10).all() &&
                          (candidate.array() <= model.upper.data.array() + 1e-10).all();
                constraint_ok = pose_ok && h <= 1e-10;
                joint_step = (candidate - accepted).cwiseAbs().maxCoeff();
                velocity_ratio =
                    ((candidate - accepted).cwiseAbs().array() / dt / model.velocity.array()).maxCoeff();
            }
            if (pose_ok) {
                ++pose_successes;
                trial["pose_successes"] = trial["pose_successes"].get<int>() + 1;
                successful_times.push_back(us);
                max_position = std::max(max_position, position);
                max_rotation = std::max(max_rotation, rotation);
                if (position <= epsilon && rotation <= epsilon) ++strict_successes;
                if (constraint_ok) {
                    ++constraint_successes;
                    trial["constraint_successes"] = trial["constraint_successes"].get<int>() + 1;
                }
                if (on_time) {
                    ++on_time_successes;
                    trial["on_time_successes"] = trial["on_time_successes"].get<int>() + 1;
                }
                max_step = std::max(max_step, joint_step);
                max_h = std::max(max_h, h);
                max_velocity_ratio = std::max(max_velocity_ratio, velocity_ratio);
            }
            trace << index << ',' << step << ',' << result << ',' << us << ',' << pose_ok << ','
                  << constraint_ok << ',' << on_time << ',' << position << ',' << rotation << ',' << h << ','
                  << joint_step << ',' << velocity_ratio;
            const bool candidate_available = result >= 0 && candidate.size() == n && candidate.allFinite();
            for (int j = 0; j < n; ++j) trace << ',' << (candidate_available ? candidate[j] : accepted[j]);
            trace << '\n';
            if (!pose_ok || (backend == "anchored" && !constraint_ok) ||
                (kind == "loops" && velocity_ratio > 1. + 1e-9)) {
                trial["failure"] = !pose_ok
                                       ? (result < 0 ? solver.resultName(result) : "pose_validation")
                                       : (!constraint_ok && backend == "anchored" ? "configuration_residual"
                                                                                  : "discontinuity");
                trial["failed_step"] = step;
                finished = false;
                break;
            }
            accepted = candidate;
            if (kind == "loops" && step % samples == 0) {
                const double deviation = (accepted - anchor).cwiseAbs().maxCoeff();
                closure = std::max(closure, deviation);
                checkpoints.push_back({{"cycle", step / samples},
                                       {"joint_closure_rad", deviation},
                                       {"position_error_m", position},
                                       {"rotation_error_rad", rotation}});
            }
        }
        if (finished) ++completed;
        trial["completed"] = finished;
        trial["max_joint_step_rad"] = max_step;
        trial["max_h_rad"] = max_h;
        trial["max_velocity_ratio"] = max_velocity_ratio;
        trial["max_joint_closure_rad"] = closure;
        trial["checkpoints"] = checkpoints;
        trials.push_back(trial);
    }
    trace.close();
    if (!trace) throw std::runtime_error("Trace write failed");
#ifdef IK_TEST_pin
    const char* library = "pin_ik";
#else
    const char* library = "trac_ik";
#endif
    Json report{{"library", library},
                {"backend", backend},
                {"model", model_name},
                {"dof", n},
                {"kind", kind},
                {"samples", count},
                {"budget_ms", timeout * 1000},
                {"epsilon", epsilon},
                {"common_acceptance_tolerance", 2 * epsilon},
                {"loop_cycles", cycles},
                {"loop_samples_per_cycle", samples},
                {"dt", dt},
                {"calls", calls},
                {"completed", completed},
                {"initialization_failures", initialization_failures},
                {"overruns", overruns},
                {"pose_successes", pose_successes},
                {"strict_pose_successes", strict_successes},
                {"on_time_pose_successes", on_time_successes},
                {"constraint_successes", constraint_successes},
                {"solve_us", quantiles(solve_times)},
                {"successful_solve_us", quantiles(successful_times)},
                {"initial_setup_us", setup_us},
                {"anchor_setup_us", quantiles(anchor_times)},
                {"max_position_error_m", max_position},
                {"max_rotation_error_rad", max_rotation},
                {"fk_disagreement", max_fk_disagreement},
                {"max_witness_constraint_residual", max_witness_h},
                {"trials", trials}};
    std::ofstream summary(prefix + ".json"), dataset(prefix + "_inputs.json");
    summary << report.dump(2) << '\n';
    dataset << inputs.dump(2) << '\n';
    summary.close();
    dataset.close();
    if (!summary || !dataset || max_fk_disagreement > 1e-10)
        throw std::runtime_error("Output or model validation failed");
    std::cout << library << ' ' << backend << ' ' << model_name << ' ' << kind << " completed=" << completed
              << '/' << count << " pose_success=" << pose_successes << '/' << calls
              << " overruns=" << overruns << '\n';
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
