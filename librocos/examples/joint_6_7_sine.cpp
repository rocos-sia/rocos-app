#include "common.hpp"
#include "joint_sine_trajectory.hpp"
#include "realtime_thread.hpp"

#include <httplib.h>
#include <arpa/inet.h>
#include <json.hpp>
#include <pthread.h>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <optional>
#include <thread>
#include <vector>

namespace {
using Json = nlohmann::json;
namespace fs = std::filesystem;
constexpr std::size_t kDof = 7;
constexpr double kZeroTolerance = 1e-4;  // rad，约 0.00573°。
constexpr double kStoppedVelocity = 1e-3;

struct Configuration {
    std::array<joint_sine::Limits, kDof> limits;
    std::array<Json, kDof> drives;
    librocos::Options options;
    std::string urdf;
    std::string servo_settings_yaml;
};

Configuration loadConfiguration(const fs::path& path) {
    const auto root = YAML::LoadFile(path.string());
    const auto robot = root["robot"];
    if (!robot || !robot["hardware"] || !robot["master"])
        throw std::invalid_argument("Use the App --export_config effective.yaml");
    if (robot["name"].as<std::string>() != "talon" ||
        robot["base_link"].as<std::string>() != "base_link" ||
        robot["tip_link"].as<std::string>() != "link_7")
        throw std::invalid_argument("This example expects the seven-axis Talon model");
    if (!robot["servo"]["communication_guard_enabled"].as<bool>() ||
        !robot["servo"]["rate_limit_enabled"].as<bool>())
        throw std::invalid_argument("Enable the communication guard and rate limiter");
    Configuration result;
    result.servo_settings_yaml = YAML::Dump(robot["servo"]);
    const int cycle = robot["master"]["cycle"].as<int>();
    const int port = root["software"]["http_port"].as<int>();
    if (cycle != 1000 || port < 1 || port > 65535)
        throw std::invalid_argument("This test expects a 1000 us cycle and valid HTTP port");
    result.options.http_port = static_cast<std::uint16_t>(port);
    fs::path urdf = robot["urdf"].as<std::string>();
    if (urdf.is_relative()) urdf = path.parent_path() / urdf;
    std::ifstream file(urdf);
    if (!file) throw std::runtime_error("Cannot read configured URDF");
    result.urdf.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    tinyxml2::XMLDocument document;
    if (document.Parse(result.urdf.c_str()) != tinyxml2::XML_SUCCESS ||
        !document.FirstChildElement("robot"))
        throw std::invalid_argument("Invalid URDF");
    if (robot["hardware"].size() != kDof)
        throw std::invalid_argument("Expected exactly seven hardware drives");
    for (std::size_t i = 0; i < kDof; ++i) {
        const auto name = "joint_" + std::to_string(i + 1);
        YAML::Node drive;
        for (const auto& item : robot["hardware"])
            if (item["joint_name"].as<std::string>() == name) {
                if (drive.IsDefined() && !drive.IsNull())
                    throw std::invalid_argument("Duplicate hardware joint");
                drive.reset(item);
            }
        if (!drive || drive.IsNull() || drive["type"].as<std::string>() != "driver")
            throw std::invalid_argument("Missing Talon driver " + name);
        const auto transform = drive["transform"];
        if (transform["user_unit_name"].as<std::string>() != "rad")
            throw std::invalid_argument("Expected radian hardware units");
        const double counts = transform["pos_per_unit"].as<double>();
        const double ratio = transform["ratio"].as<double>();
        if (!std::isfinite(counts) || counts <= 0 || !std::isfinite(ratio) || ratio == 0)
            throw std::invalid_argument("Invalid encoder scale");
        result.drives[i] = {{"id", drive["id"].as<int>()}, {"name", name},
                           {"pos_per_unit", transform["pos_per_unit"].as<double>()},
                           {"ratio", transform["ratio"].as<double>()},
                           {"zero_offset", transform["offset_pos_cnt"].as<int>()}};
        const auto hw = drive["limit"];
        auto& limit = result.limits[i];
        limit = {hw["lower"].as<double>(), hw["upper"].as<double>(),
                 hw["vel"].as<double>(), hw["acc"].as<double>(),
                 hw["jerk"].as<double>(), hw["following_error"].as<double>(0)};
        bool found = false;
        for (auto* joint = document.FirstChildElement("robot")->FirstChildElement("joint");
             joint; joint = joint->NextSiblingElement("joint")) {
            if (!joint->Attribute("name") || name != joint->Attribute("name")) continue;
            if (found || !joint->Attribute("type") ||
                std::string(joint->Attribute("type")) != "revolute" ||
                joint->FirstChildElement("mimic"))
                throw std::invalid_argument("Unsupported or duplicate model joint");
            auto* parent = joint->FirstChildElement("parent");
            auto* child = joint->FirstChildElement("child");
            const auto parent_name = i ? "link_" + std::to_string(i) : "base_link";
            const auto child_name = "link_" + std::to_string(i + 1);
            if (!parent || !child || !parent->Attribute("link") || !child->Attribute("link") ||
                parent_name != parent->Attribute("link") || child_name != child->Attribute("link"))
                throw std::invalid_argument("Unexpected Talon joint order");
            auto* bounds = joint->FirstChildElement("limit");
            double lower{}, upper{}, velocity{};
            if (!bounds || bounds->QueryDoubleAttribute("lower", &lower) ||
                bounds->QueryDoubleAttribute("upper", &upper) ||
                bounds->QueryDoubleAttribute("velocity", &velocity))
                throw std::invalid_argument("Incomplete URDF limits");
            if (!std::isfinite(lower) || !std::isfinite(upper) || !std::isfinite(velocity))
                throw std::invalid_argument("Nonfinite URDF limits");
            limit.lower = std::max(limit.lower, lower);
            limit.upper = std::min(limit.upper, upper);
            limit.velocity = std::min(limit.velocity, velocity);
            found = true;
        }
        if (!found) throw std::invalid_argument("Missing model joint " + name);
        for (double value : {limit.lower, limit.upper, limit.velocity, limit.acceleration,
                             limit.jerk, limit.following})
            if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite hardware limit");
        if (limit.lower >= limit.upper || limit.velocity <= 0 || limit.acceleration <= 0 ||
            limit.jerk <= 0 || limit.following < 0)
            throw std::invalid_argument("Invalid hardware limit");
        if (limit.following == 0) limit.following = limit.velocity * .001 * 20;
    }
    for (std::size_t i : {5u, 6u})
        if (result.limits[i].jerk != 10000)
            throw std::invalid_argument("This experiment requires J6/J7 J=10000 rad/s^3");
    return result;
}

Json checkConnectedModel(const Configuration& config, const std::string& host) {
    httplib::Client http(host, config.options.http_port);
    http.set_connection_timeout(1);
    http.set_read_timeout(1);
    const auto model = http.Get("/api/robot/urdf");
    if (!model || model->status != 200 || model->body != config.urdf)
        throw std::runtime_error("Connected URDF differs from exported configuration");
    const auto response = http.Get("/api/robot/info");
    if (!response || response->status != 200) throw std::runtime_error("robot/info unavailable");
    const auto body = Json::parse(response->body);
    if (!body.at("success").get<bool>()) throw std::runtime_error("robot/info rejected");
    const auto data = body.at("data");
    const auto& joints = data.at("joint_infos");
    if (joints.size() != kDof) throw std::runtime_error("Connected drive count differs");
    for (const auto& expected : config.drives) {
        int matches = 0;
        for (const auto& joint : joints) {
            if (joint.at("name") != expected.at("name")) continue;
            ++matches;
            for (const char* field : {"id", "pos_per_unit", "ratio", "zero_offset"})
                if (joint.at(field) != expected.at(field))
                    throw std::runtime_error(std::string("Connected calibration differs: ") + field);
            if (joint.at("unit_name") != "rad") throw std::runtime_error("Connected units differ");
        }
        if (matches != 1) throw std::runtime_error("Connected joint mapping differs");
    }
    return data;
}

void checkStart(const librocos::RobotState& state, const Configuration& config,
                const std::array<double, kDof>& amplitudes) {
    if (state.dof != kDof || !state.enabled ||
        (state.robot_state != "STOPPED" && state.robot_state != "RCI") ||
        state.work_mode != "position")
        throw std::runtime_error("Prepare enabled seven-axis position mode in STOPPED or RCI");
    for (std::size_t i = 0; i < kDof; ++i) {
        const auto& limit = config.limits[i];
        if (!std::isfinite(state.q[i]) || state.q[i] < limit.lower || state.q[i] > limit.upper ||
            !std::isfinite(state.dq[i]) || std::abs(state.dq[i]) > kStoppedVelocity)
            throw std::runtime_error("Startup feedback is invalid or still moving");
    }
    for (std::size_t i = 0; i < kDof; ++i)
        if (amplitudes[i] > 0 && std::abs(state.q[i]) > kZeroTolerance)
            throw std::runtime_error("J" + std::to_string(i + 1) +
                " actual=" + std::to_string(state.q[i]) +
                " rad is outside zero tolerance 0.0001 rad; execute a separate MoveJ before servo");
}

struct Sample {
    librocos::ServoSnapshot state;
    librocos::JointArray requested{}, tau{};
    double time{}, dt{};
};

class Recorder {
   public:
    explicit Recorder(const fs::path& file) : buffer_(8192), output_(file) {
        if (!output_) throw std::runtime_error("Cannot create samples.csv");
        output_ << "time_s,dt_s,cycle,sample_ns,source_us,received_id,applied_id,quality,output_valid";
        for (const char* group : {"requested", "q", "qd", "qcmd", "dq", "dqd", "ddqd", "tau"})
            for (int i = 1; i <= 7; ++i) output_ << ',' << group << i;
        output_ << '\n' << std::setprecision(17);
        thread_ = std::thread([this] { write(); });
    }
    ~Recorder() { finish(); }
    void push(const Sample& sample) {
        const auto next = written_.load(std::memory_order_relaxed);
        if (failed_.load() || next - read_.load(std::memory_order_acquire) >= buffer_.size())
            throw std::runtime_error("Data writer failed or queue overflowed; stopping servo");
        buffer_[next % buffer_.size()] = sample;
        written_.store(next + 1, std::memory_order_release);
    }
    void finish() {
        done_.store(true);
        if (thread_.joinable()) thread_.join();
    }
    bool failed() const { return failed_.load(); }
    std::uint64_t count() const { return read_.load(); }

   private:
    void write() noexcept {
        // 控制线程可能采用 FIFO，写盘线程使用普通调度。
        sched_param parameters{};
        if (pthread_setschedparam(pthread_self(), SCHED_OTHER, &parameters) != 0) {
            failed_.store(true);
            return;
        }
        try {
            while (!done_.load() || read_.load() < written_.load()) {
                const auto current = read_.load(std::memory_order_relaxed);
                if (current == written_.load(std::memory_order_acquire)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                const auto& row = buffer_[current % buffer_.size()];
                const auto& s = row.state;
                output_ << row.time << ',' << row.dt << ',' << s.cycle << ',' << s.sample_time_ns
                        << ',' << s.source_time_us << ',' << s.received_message_id << ','
                        << s.applied_message_id << ',' << s.control_command_success_rate << ','
                        << s.output_valid;
                for (const auto* array : {&row.requested, &s.q, &s.q_d, &s.q_cmd,
                                         &s.dq, &s.dq_d, &s.ddq_d, &row.tau})
                    for (std::size_t i = 0; i < kDof; ++i) output_ << ',' << (*array)[i];
                output_ << '\n';
                if (!output_) throw std::runtime_error("CSV write failed");
                read_.store(current + 1, std::memory_order_release);
            }
            output_.flush();
            output_.close();
            if (!output_) failed_.store(true);
        } catch (...) { failed_.store(true); }
    }
    std::vector<Sample> buffer_;
    std::ofstream output_;
    std::atomic<std::uint64_t> written_{0}, read_{0};
    std::atomic<bool> done_{false}, failed_{false};
    std::thread thread_;
};

void writeJson(const fs::path& file, const Json& data) {
    std::ofstream output(file);
    output << data.dump(2) << '\n';
    output.flush();
    if (!output) throw std::runtime_error("Cannot save " + file.string());
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 3) throw std::invalid_argument(
            "Usage: librocos_joint_6_7_sine effective.yaml output_dir "
            "[--ip 127.0.0.1] [--check] [--realtime] [--all-joints] [--amplitude-deg 60] [--frequency 0.2] [--duration 180] [--udp-port 8081]");
        auto config = loadConfiguration(fs::absolute(argv[1]));
        joint_sine::Parameters parameters;
        bool check_only = false;
        bool realtime = false;
        bool all_joints = false;
        bool custom_amplitude = false;
        std::string host = "127.0.0.1";
        for (int i = 3; i < argc; ++i) {
            const std::string flag = argv[i];
            if (flag == "--check") { check_only = true; continue; }
            if (flag == "--realtime") { realtime = true; continue; }
            if (flag == "--all-joints") { all_joints = true; continue; }
            if (++i == argc) throw std::invalid_argument("Missing option value");
            if (flag == "--ip") {
                host = argv[i];
                in_addr address{};
                if (inet_pton(AF_INET, host.c_str(), &address) != 1 || address.s_addr == INADDR_ANY)
                    throw std::invalid_argument("--ip requires a nonzero IPv4 address");
                continue;
            }
            std::size_t used = 0;
            const double value = std::stod(argv[i], &used);
            if (used != std::string(argv[i]).size() || !std::isfinite(value))
                throw std::invalid_argument("Invalid numeric option");
            if (flag == "--amplitude-deg") {
                parameters.amplitude = value * joint_sine::kPi / 180;
                custom_amplitude = true;
            }
            else if (flag == "--frequency") parameters.frequency = value;
            else if (flag == "--duration") parameters.duration = value;
            else if (flag == "--udp-port" && value >= 1 && value <= 65535 && std::floor(value) == value)
                config.options.udp_port = static_cast<std::uint16_t>(value);
            else throw std::invalid_argument("Unknown or invalid option " + flag);
        }
        if (all_joints && custom_amplitude)
            throw std::invalid_argument("--all-joints uses fixed amplitudes 10/50 deg; omit --amplitude-deg");
        if (all_joints) parameters.amplitude = 50 * joint_sine::kPi / 180;
        parameters.validate();
        std::array<double, kDof> amplitudes{};
        if (all_joints)
            amplitudes = {10, 10, 10, 50, 10, 50, 50};
        for (std::size_t i = 0; i < kDof; ++i)
            amplitudes[i] = all_joints ? amplitudes[i] * joint_sine::kPi / 180 :
                            (i >= 5 ? parameters.amplitude : 0);
        const auto peak = joint_sine::bounds(parameters);
        std::cout << (all_joints ? "ALL SEVEN JOINTS move, J1/J2/J3/J5 +/-10 deg, J4/J6/J7 +/-50 deg.\n" : "J1-J5 hold, J6/J7 move.\n")
                  << "Moving joints zero-centered, in phase, maximum amplitude +/- " << parameters.amplitude * 180 / joint_sine::kPi
                  << " deg, " << parameters.frequency << " Hz, " << parameters.duration
                  << " s including two 5 s ramps, then 1 s hold.\n"
                  << "Conservative peak v/a/j: " << peak.v << " / " << peak.a << " / " << peak.j
                  << " (rad-based units). Startup zero tolerance: " << kZeroTolerance << " rad.\n";
        std::cout << "Trajectory time = (cycle - start_cycle) * 0.001 s.\n";
        for (std::size_t i = 0; i < kDof; ++i) {
            if (amplitudes[i] == 0) continue;
            const auto& l = config.limits[i];
            auto joint_parameters = parameters;
            joint_parameters.amplitude = amplitudes[i];
            if (l.jerk != 10000)
                throw std::invalid_argument("Moving joints require J=10000 rad/s^3");
            std::cout << "J" << i + 1 << " amplitude +/- " << amplitudes[i] * 180 / joint_sine::kPi
                      << " deg, configured q range [" << l.lower << ", " << l.upper
                      << "], v/a/j " << l.velocity << '/' << l.acceleration << '/' << l.jerk
                      << ", following tolerance " << l.following << " rad.\n";
            joint_sine::checkTrajectory(joint_parameters, l);
        }
        if (check_only) { std::cout << "Offline trajectory check passed.\n"; return 0; }
        librocos::Robot robot(host, config.options);
        const auto info = checkConnectedModel(config, host);
        checkStart(robot.readOnce(), config, amplitudes);
        const fs::path directory = fs::absolute(argv[2]);
        if (!fs::create_directory(directory)) throw std::runtime_error("Output directory already exists");
        Json metadata = {{"frequency_hz", parameters.frequency}, {"amplitude_rad", parameters.amplitude},
                         {"data_source", "librocos UDP snapshots"},
                         {"duration_s", parameters.duration}, {"ramp_s", parameters.ramp},
                         {"hold_s", parameters.hold}, {"center_rad", 0}, {"phase_difference_rad", 0},
                         {"effective_config", fs::absolute(argv[1]).string()},
                         {"exported_servo_settings_yaml", config.servo_settings_yaml},
                         {"controller_info", info}, {"cycle_s", .001}, {"zero_tolerance_rad", kZeroTolerance}};
        metadata["joint_amplitudes_rad"] = amplitudes;
        metadata["controller_ip"] = host;
        metadata["trajectory_time_basis"] = "control_cycle";
        for (const auto& l : config.limits)
            metadata["limits"].push_back({{"lower", l.lower}, {"upper", l.upper},
                {"velocity", l.velocity}, {"acceleration", l.acceleration},
                {"jerk", l.jerk}, {"following_error", l.following}});
        writeJson(directory / "metadata.json", metadata);
        std::signal(SIGINT, example::interrupt);
        std::signal(SIGTERM, example::interrupt);
        std::cout << "Confirm the printed moving joints and amplitudes. Press Enter to start.\n";
        if (std::cin.get() == std::char_traits<char>::eof() || example::interrupted)
            throw std::runtime_error("Motion confirmation interrupted or missing");
        checkConnectedModel(config, host);
        checkStart(robot.readOnce(), config, amplitudes);
        Recorder recorder(directory / "samples.csv");
        Json outcome = {{"completed", false}, {"interrupted", false}, {"error", ""}};
        std::optional<std::uint64_t> start_cycle;
        librocos::JointArray initial{};
        try {
            if (realtime) {
                pthread_setname_np(pthread_self(), "librocos_servo");
                const auto scheduling = rocos::ConfigureRealtimeThread(30);
                outcome["client_scheduling"] = scheduling.description();
                std::cout << "Client scheduling: " << scheduling.description() << std::endl;
                if (!scheduling.ready())
                    throw std::runtime_error("Realtime scheduling unavailable; run the example via sudo with the --realtime option");
            }
            robot.enableRci();
            robot.control([&](const librocos::RobotState& state, librocos::Duration dt) {
                if (!start_cycle) {
                    for (std::size_t i = 0; i < kDof; ++i)
                        if (amplitudes[i] > 0 && (std::abs(state.q[i]) > kZeroTolerance ||
                            std::abs(state.q_d[i]) > kZeroTolerance))
                            throw std::runtime_error("Zero position changed before first UDP command");
                    start_cycle = state.cycle;
                    initial = state.q_d;
                }
                if (state.cycle < *start_cycle)
                    throw std::runtime_error("Control cycle counter moved backwards");
                const double elapsed = (state.cycle - *start_cycle) * .001;
                auto target = initial;
                const double shape = joint_sine::evaluate(parameters, elapsed).q / parameters.amplitude;
                for (std::size_t i = 0; i < kDof; ++i)
                    if (amplitudes[i] > 0) target[i] = amplitudes[i] * shape;
                Sample row;
                row.state = static_cast<const librocos::ServoSnapshot&>(state);
                row.requested = target; row.tau = state.tau; row.time = elapsed; row.dt = dt.toSec();
                recorder.push(row);
                if (!state.output_valid || state.output_mode != 8)
                    throw std::runtime_error("Expected valid CSP output snapshot");
                for (std::size_t i = 0; i < kDof; ++i) {
                    const auto& l = config.limits[i];
                    if (!std::isfinite(state.q[i]) || !std::isfinite(state.dq[i]) ||
                        state.q[i] < l.lower || state.q[i] > l.upper ||
                        state.q_cmd[i] < l.lower || state.q_cmd[i] > l.upper ||
                        std::abs(state.dq[i]) > l.velocity ||
                        std::abs(state.dq_d[i]) > l.velocity + 1e-9 ||
                        std::abs(state.ddq_d[i]) > l.acceleration + 1e-9 ||
                        std::abs(state.q[i] - state.q_cmd[i]) > l.following)
                        throw std::runtime_error("Feedback/reference protection on J" + std::to_string(i + 1) +
                            ", q=" + std::to_string(state.q[i]) + ", dq=" + std::to_string(state.dq[i]) +
                            ", following_error=" + std::to_string(state.q[i] - state.q_cmd[i]) +
                            ", dq_d=" + std::to_string(state.dq_d[i]) +
                            ", ddq_d=" + std::to_string(state.ddq_d[i]));
                }
                librocos::JointPositions command(target, kDof);
                if (example::interrupted) return librocos::MotionFinished(command);
                if (elapsed >= parameters.duration + parameters.hold) {
                    for (std::size_t i = 0; i < kDof; ++i)
                        if (amplitudes[i] > 0 && (std::abs(state.q[i]) > kZeroTolerance ||
                            std::abs(state.dq[i]) > kStoppedVelocity))
                            throw std::runtime_error("Final zero position has not settled");
                    return librocos::MotionFinished(command);
                }
                return command;
            });
            outcome["completed"] = !example::interrupted;
        } catch (const std::exception& error) {
            outcome["error"] = error.what();
            if (const auto* control = dynamic_cast<const librocos::ControlException*>(&error)) {
                outcome["exception_code"] = control->code();
                if (control->cause) {
                    try { std::rethrow_exception(control->cause); }
                    catch (const std::exception& cause) { outcome["cause"] = cause.what(); }
                }
            }
        }
        recorder.finish();
        outcome["interrupted"] = bool(example::interrupted);
        outcome["samples"] = recorder.count();
        outcome["writer_failed"] = recorder.failed();
        outcome["initial_reference"] = initial;
        try {
            const auto status = robot.rciStatus();
            outcome["robot_state"] = status.robot_state;
            outcome["control_active"] = status.control_active;
            outcome["rci_enabled"] = status.enabled;
            if (status.last_control) {
                const auto& s = *status.last_control;
                outcome["last_control"] = {{"fault_code", s.fault_code}, {"errors", s.errors},
                    {"sample_time_ns", s.sample_time_ns}, {"applied_id", s.applied_message_id},
                    {"q", s.q}, {"dq", s.dq}, {"q_d", s.q_d}, {"q_cmd", s.q_cmd},
                    {"output_valid", s.output_valid}};
            }
            if (status.control_active || status.robot_state != "RCI" || !status.last_control ||
                status.last_control->fault_code) outcome["completed"] = false;
        } catch (const std::exception& error) {
            outcome["status_error"] = error.what(); outcome["completed"] = false;
        }
        if (recorder.failed()) outcome["completed"] = false;
        writeJson(directory / "outcome.json", outcome);
        std::cout << outcome.dump(2) << "\nData saved in " << directory << '\n';
        return outcome["completed"].get<bool>() ? 0 : 1;
    } catch (const std::exception& error) {
        return example::report(error);
    }
}
