// 辨识模型驱动的关节阻抗：Y(q,dq,0)·Pβ + friction_scale·摩擦补偿 + Kp·(qd−q) − Kd·dq。
// 初始化时加载文件、模型和缓存；控制周期内只校验状态并计算力矩。
#include "identified_joint_impedance.hpp"

#include <openssl/sha.h>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <set>
#include <sstream>
#include <stdexcept>

#include "experiment_common.hpp"

namespace {
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
void require(bool ok, const std::string &message) {
    if (!ok) throw std::runtime_error(message);
}

// 配置中的相对路径以 YAML 所在目录为基准，不随程序工作目录变化。
std::string relativeTo(const std::string &file, const std::string &path) {
    if (!path.empty() && path.front() == '/') return path;
    const auto slash = file.find_last_of('/');
    return (slash == std::string::npos ? "" : file.substr(0, slash + 1)) + path;
}

// 核对辨识时的模型文件，避免结构相同但零点、坐标或参数版本不同的文件混用。
std::string sha256(const std::string &path) {
    std::ifstream file(path, std::ios::binary);
    require(bool(file), "Cannot read URDF: " + path);
    const std::string bytes{std::istreambuf_iterator<char>(file), {}};
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size(), hash);
    std::ostringstream text;
    for (auto byte : hash) text << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    return text.str();
}

Eigen::VectorXd readVector(const YAML::Node &node, int count, const std::string &name) {
    require(node.IsSequence() && node.size() == size_t(count), "Wrong length: " + name);
    Eigen::VectorXd result(count);
    for (int i = 0; i < count; ++i) result[i] = node[i].as<double>();
    require(result.allFinite(), "Nonfinite values: " + name);
    return result;
}

const tinyxml2::XMLElement *findJoint(const tinyxml2::XMLElement *robot, const std::string &name) {
    for (auto *joint = robot->FirstChildElement("joint"); joint;
         joint = joint->NextSiblingElement("joint")) {
        if (joint->Attribute("name", name.c_str())) return joint;
    }
    throw std::runtime_error("Joint missing from URDF: " + name);
}

struct FrictionParameters {
    double coulomb;               // 库仑项 Fc，Nm。
    double loadCoefficient;       // 负载相关系数，乘以刚体力矩绝对值。
    double stribeckAmplitude;     // Stribeck 附加幅值 As，Nm。
    double viscousAtReference;    // 参考温度下的黏滞系数，Nm·s/rad。
    double temperatureSlope;      // 黏滞温度斜率，允许为负。
    double referenceTemperature;  // 参考温度，°C。
    double stribeckVelocity;      // Stribeck 衰减速度 vs，rad/s。
    double smoothVelocity;        // 过零平滑速度 c，rad/s，与 vs 含义不同。

    explicit FrictionParameters(const YAML::Node &node) {
        auto read = [&](const char *key) {
            const double value = node[key].as<double>();
            require(std::isfinite(value), "Nonfinite friction coefficient");
            return value;
        };
        coulomb = read("coulomb");
        loadCoefficient = read("load_coefficient");
        stribeckAmplitude = read("stribeck_amplitude");
        viscousAtReference = read("viscous_at_reference");
        temperatureSlope = read("viscous_temperature_slope");
        referenceTemperature = read("reference_temperature_c");
        stribeckVelocity = read("stribeck_velocity");
        smoothVelocity = read("smooth_velocity");

        require(coulomb >= 0 && loadCoefficient >= 0, "Negative Coulomb/load friction");
        require(stribeckAmplitude >= 0 && viscousAtReference >= 0,
                "Negative Stribeck/viscous friction");
        require(stribeckVelocity > 0 && smoothVelocity > 0, "Friction velocities must be positive");
    }

    double compute(double velocity, double rigidTorque, double temperature) const {
        // load 使用未加摩擦和 PD 的刚体力矩，避免补偿力矩自身进入负载项。
        const double loadFriction = loadCoefficient * std::abs(rigidTorque);
        const double stribeck =
            stribeckAmplitude * std::exp(-std::pow(velocity / stribeckVelocity, 2));
        const double direction = std::tanh(velocity / smoothVelocity);
        const double viscous =
            viscousAtReference + temperatureSlope * (temperature - referenceTemperature);
        require(std::isfinite(viscous) && viscous >= 0,
                "Temperature produces invalid viscous friction");
        // 返回与驱动力矩同侧相加的有符号摩擦需求，不是实际外力矩测量。
        return (coulomb + loadFriction + stribeck) * direction + viscous * velocity;
    }
};
}  // namespace

namespace rocos {
struct IdentifiedJointImpedanceController::Impl {
    IdentifiedJointImpedanceConfig config;
    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    Eigen::VectorXd parameters;  // 离线导出的 Pβ，保持 Pinocchio 的惯性参数顺序。
    Eigen::VectorXd zeroAcceleration, rigidTorque, frictionTorque, command;
    std::vector<FrictionParameters> friction;

    explicit Impl(const std::string &path) {
        const auto yaml = YAML::LoadFile(path);
        auto modelPath = [&](const char *key) {
            return relativeTo(path, yaml[key].as<std::string>());
        };
        config.urdf = modelPath("urdf");
        config.hardware = modelPath("hardware");
        loadRigidModel(modelPath("rigid_model"));
        loadGains(yaml);
        loadHardwareLimits();
        loadFrictionModel(modelPath("friction"));

        // 一次性分配输出和中间缓存，避免周期内反复加载模型或调整向量长度。
        const int n = config.jointNames.size();
        zeroAcceleration = Eigen::VectorXd::Zero(n);
        rigidTorque.resize(n);
        frictionTorque.resize(n);
        command.resize(n);
    }

    // 刚体模型负责 Y 的结构，辨识结果负责参与力矩计算的参数。
    void loadRigidModel(const std::string &path) {
        const auto result = YAML::LoadFile(path);
        config.jointNames = result["joint_names"].as<std::vector<std::string>>();
        const int n = config.jointNames.size();
        require(n > 0, "Empty model joint list");
        const std::set<std::string> uniqueNames(config.jointNames.begin(), config.jointNames.end());
        require(uniqueNames.size() == config.jointNames.size(), "Duplicate model joint names");
        require(result["schema"].as<std::string>() == "rocos_identified_rigid_v1",
                "Unknown rigid model schema");
        require(result["urdf_hash"].as<std::string>() == sha256(config.urdf),
                "Rigid model/URDF hash mismatch");
        // URDF 提供关节轴和连杆变换，只在启动时解析一次。
        // 后续计算 Y·Pβ，使用辨识参数，而非直接调用名义惯量的逆动力学。
        pinocchio::urdf::buildModel(config.urdf, model);
        require(model.nq == n && model.nv == n && model.njoints == n + 1,
                "Expected fixed-base single-DOF model");
        for (int i = 0; i < n; ++i) {
            require(model.names[i + 1] == config.jointNames[i], "Model joint order mismatch");
            require(model.joints[i + 1].nq() == 1 && model.joints[i + 1].nv() == 1,
                    "Unsupported joint type");
        }
        data.reset(new pinocchio::Data(model));
        parameters = readVector(result["rigid_parameters"], 10 * n, "P * beta");
    }

    // 所有数组按刚体模型的 joint_names 排列，Kp/Kd 的单位为 Nm/rad、Nm·s/rad。
    void loadGains(const YAML::Node &yaml) {
        const int n = config.jointNames.size();
        config.kp = readVector(yaml["kp"], n, "kp");
        config.kd = readVector(yaml["kd"], n, "kd");
        config.frictionScale = yaml["friction_scale"].as<double>(1.0);
        require(std::isfinite(config.frictionScale) && config.frictionScale >= 0 &&
                    config.frictionScale <= 1,
                "friction_scale must be finite and in [0, 1]");
        require(yaml["velocity_limit"].IsDefined(), "Missing velocity_limit");
        config.velocityLimit = readVector(yaml["velocity_limit"], n, "velocity_limit");
        config.torqueLimit = readVector(yaml["torque_limit"], n, "torque_limit");
        config.torqueRateLimit = readVector(yaml["torque_rate_limit"], n, "torque_rate_limit");
        require((config.kp.array() >= 0).all(), "kp must be nonnegative");
        require((config.kd.array() >= 0).all(), "kd must be nonnegative");
        require((config.velocityLimit.array() > 0).all(), "velocity_limit must be positive");
        require((config.torqueLimit.array() > 0).all(), "Torque limits must be positive");
        require((config.torqueRateLimit.array() > 0).all(), "Torque rate limits must be positive");
    }

    void loadFrictionModel(const std::string &path) {
        const auto document = YAML::LoadFile(path);
        require(document["model"].as<std::string>() == "stribeck_load_temperature_v1",
                "Unsupported friction model");
        // 新版导出带有 URDF 指纹时必须匹配；兼容不含此字段的旧版参数库。
        if (document["urdf_hash"]) {
            require(document["urdf_hash"].as<std::string>() == sha256(config.urdf),
                    "Friction model/URDF hash mismatch");
        }
        const auto joints = document["joint_parameters"];
        require(joints.IsMap(), "Friction joint_parameters must be a map");
        require(joints.size() == config.jointNames.size(), "Friction joint count mismatch");
        for (const auto &name : config.jointNames) {
            friction.emplace_back(joints[name]);  // 按关节名匹配，不依赖 JSON 键的顺序。
            config.needsTemperature |= friction.back().temperatureSlope != 0;
        }
    }

    // 硬件映射与变换来自新版 YAML；URDF 仅提供模型和标准关节限位。
    void loadHardwareLimits() {
        config.lower = model.lowerPositionLimit;
        config.upper = model.upperPositionLimit;
        config.velocity = model.velocityLimit;
        require(config.lower.allFinite() && config.upper.allFinite(), "Nonfinite position limits");
        require(config.velocity.allFinite(), "Nonfinite velocity limits");
        const auto hardware = Hardware::LoadConfigFromYAML(config.hardware);
        const auto drives = experiment::orderedDrives(hardware, config.jointNames);
        tinyxml2::XMLDocument xml;
        require(xml.LoadFile(config.urdf.c_str()) == tinyxml2::XML_SUCCESS, "Cannot read URDF");
        const auto *robot = xml.FirstChildElement("robot");
        require(robot, "URDF has no robot element");
        for (size_t i = 0; i < drives.size(); ++i) {
            const auto &drive = drives[i];
            const auto *joint = findJoint(robot, config.jointNames[i]);
            require(!joint->FirstChildElement("mimic"),
                    "Mimic joints are not supported by Studio identification");
            require(joint->Attribute("type", "revolute"),
                    "Only bounded revolute joints are supported");
            config.hardwareIds.push_back(drive.id);
            config.lower[i] = std::max(config.lower[i], drive.limit.lower);
            config.upper[i] = std::min(config.upper[i], drive.limit.upper);
            config.velocity[i] = std::min(config.velocity[i], drive.limit.vel);
            require(config.lower[i] < config.upper[i], "Empty position limit interval");
            require(config.velocityLimit[i] <= config.velocity[i],
                    "velocity_limit exceeds model/hardware limit");
            const double effort = std::min(model.effortLimit[i], drive.limit.effort);
            require(std::isfinite(effort) && config.torqueLimit[i] <= effort,
                    "Torque limit exceeds model/hardware effort");
            require(config.torqueLimit[i] * std::abs(drive.transform.torque_per_unit) <= 32767,
                    "Torque limit exceeds int16 drive range");
        }
    }

    // 计算前检查完整输入，避免非有限值、越限目标或缺失温度进入补偿公式。
    void validateInput(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                       const Eigen::VectorXd &qDesired, const Eigen::VectorXd &temperature) const {
        const int n = config.jointNames.size();
        require(q.size() == n && dq.size() == n && qDesired.size() == n, "Wrong state/target size");
        require(q.allFinite() && dq.allFinite() && qDesired.allFinite(), "Nonfinite state/target");
        for (int i = 0; i < n; ++i) {
            // 使用实测速度的绝对值，正反转同样保护；不滤波、不等待连续超限。
            // 异常交给主程序下使能并退出，不会自动恢复，也不检查位置跟随误差。
            if (std::abs(dq[i]) > config.velocityLimit[i]) {
                throw std::runtime_error("Safety velocity limit exceeded: " + config.jointNames[i] +
                                         ", dq=" + std::to_string(dq[i]) + " rad/s, limit=" +
                                         std::to_string(config.velocityLimit[i]) +
                                         " rad/s; restart required");
            }
            require(q[i] >= config.lower[i] && q[i] <= config.upper[i],
                    "Position exceeds joint limits");
            require(qDesired[i] >= config.lower[i] && qDesired[i] <= config.upper[i],
                    "Target exceeds joint limits");
            require(std::abs(dq[i]) <= config.velocity[i], "Velocity exceeds joint limits");
        }
        if (temperature.size() == 0) {
            require(!config.needsTemperature,
                    "Friction model requires measured joint temperatures");
        } else {
            require(temperature.size() == n && temperature.allFinite(),
                    "Invalid measured temperatures");
        }
    }
};

IdentifiedJointImpedanceController::IdentifiedJointImpedanceController(const std::string &path)
    : impl_(new Impl(path)) {}
IdentifiedJointImpedanceController::IdentifiedJointImpedanceController(
    const std::string &path, const IdentifiedJointImpedanceTuning &tuning)
    : IdentifiedJointImpedanceController(path) {
    auto &config = impl_->config;
    const auto n = config.kp.size();
    require(tuning.kp.size() == n && tuning.kd.size() == n && tuning.kp.allFinite() &&
                tuning.kd.allFinite(),
            "Invalid override gain dimensions or nonfinite values");
    require((tuning.kp.array() >= 0).all() && (tuning.kd.array() >= 0).all(),
            "Override gains must be nonnegative");
    require(std::isfinite(tuning.frictionScale) && tuning.frictionScale >= 0 &&
                tuning.frictionScale <= 1,
            "Override friction_scale must be finite and in [0, 1]");
    config.kp = tuning.kp;
    config.kd = tuning.kd;
    config.frictionScale = tuning.frictionScale;
}
IdentifiedJointImpedanceController::~IdentifiedJointImpedanceController() = default;
const IdentifiedJointImpedanceConfig &IdentifiedJointImpedanceController::config() const {
    return impl_->config;
}
const Eigen::VectorXd &IdentifiedJointImpedanceController::rigidTorque() const {
    return impl_->rigidTorque;
}
const Eigen::VectorXd &IdentifiedJointImpedanceController::frictionTorque() const {
    return impl_->frictionTorque;
}

const Eigen::VectorXd &IdentifiedJointImpedanceController::computeTorque(
    const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &qDesired,
    const Eigen::VectorXd &temperature) {
    return computeTorque(q, dq, qDesired, temperature, true);
}

const Eigen::VectorXd &IdentifiedJointImpedanceController::computeTorque(
    const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &qDesired,
    const Eigen::VectorXd &temperature, bool frictionEnabled) {
    auto &state = *impl_;
    const auto &config = state.config;
    const int n = config.jointNames.size();
    state.validateInput(q, dq, qDesired, temperature);

    // 第三项输入零，仅提取科氏/离心和重力补偿，不抵消机械臂本身的惯量。
    // 这不表示实际加速度为零，也不需要在这里估计或反馈实际加速度。
    const auto &Y = pinocchio::computeJointTorqueRegressor(state.model, *state.data, q, dq,
                                                           state.zeroAcceleration);
    state.rigidTorque.noalias() = Y * state.parameters;

    for (int i = 0; i < n; ++i) {
        const auto &friction = state.friction[i];
        // 无温度项时使用参考温度使温度增量为零；有温度项时前面已强制实测输入。
        const double jointTemperature =
            temperature.size() ? temperature[i] : friction.referenceTemperature;
        // 缩放整个摩擦补偿，调试输出与最终力矩使用同一个缩放后的分量。
        state.frictionTorque[i] =
            frictionEnabled ? config.frictionScale *
                                  friction.compute(dq[i], state.rigidTorque[i], jointTemperature)
                            : 0.0;
    }

    // 保留机械臂自身惯量：只叠加模型补偿、位置弹簧和实测速度阻尼。
    // 限幅与变化率约束由主程序在调用 Hardware::SetTorque() 前统一执行。
    state.command = state.rigidTorque + state.frictionTorque +
                    config.kp.cwiseProduct(qDesired - q) - config.kd.cwiseProduct(dq);
    require(state.command.allFinite(), "Nonfinite impedance torque");
    return state.command;
}
}  // namespace rocos
