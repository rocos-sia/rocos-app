//
// Created by think on 6/30/26.
//

#include "hardware.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "ecat_type.hpp"
#include "logger.hpp"

namespace rocos {

// ==========================================================================
// 辅助函数：YAML 字段安全读取
// ==========================================================================

/// @brief 安全读取 YAML 节点的 string 字段，不存在时返回默认值
static std::string yamlGetStr(const YAML::Node& node,
                              const std::string& key,
                              const std::string& default_val = "") {
    if (node[key]) {
        return node[key].as<std::string>();
    }
    return default_val;
}

/// @brief 安全读取 YAML 节点的 double 字段
static double yamlGetDouble(const YAML::Node& node,
                            const std::string& key,
                            double default_val = 0.0) {
    if (node[key]) {
        return node[key].as<double>();
    }
    return default_val;
}

/// @brief 安全读取 YAML 节点的 int 字段
static int yamlGetInt(const YAML::Node& node,
                      const std::string& key,
                      int default_val = 0) {
    if (node[key]) {
        return node[key].as<int>();
    }
    return default_val;
}

static std::string formatDriveIdList(const std::vector<int32_t>& ids) {
    std::ostringstream oss;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << ids[i];
    }
    return oss.str();
}

// ==========================================================================
// 静态解析函数
// ==========================================================================

Drive Hardware::parseDrive(const YAML::Node& node) {
    Drive d;
    d.id = node["id"].as<int>();
    d.joint_name = yamlGetStr(node, "joint_name", "joint_" + std::to_string(d.id));

    // 解析关节力矩传感器数据来源
    std::string src = yamlGetStr(node, "torque_source", "load_torque");
    if (src == "secondary_position") {
        d.torque_source = TorqueSource::SecondaryPosition;
    } else {
        d.torque_source = TorqueSource::LoadTorque;
    }

    // 解析 limit 子节点
    if (node["limit"]) {
        const auto& lim = node["limit"];
        d.limit.lower  = yamlGetDouble(lim, "lower", -2.71);
        d.limit.upper  = yamlGetDouble(lim, "upper", 2.71);
        d.limit.vel    = yamlGetDouble(lim, "vel", 3.0);
        d.limit.acc    = yamlGetDouble(lim, "acc", 10.0);
        d.limit.jerk   = yamlGetDouble(lim, "jerk", 150.0);
        d.limit.effort = yamlGetDouble(lim, "effort", 30.0);
    }

    // 解析 transform 子节点
    if (node["transform"]) {
        const auto& trans = node["transform"];
        d.transform.ratio          = yamlGetDouble(trans, "ratio", 1.0);
        d.transform.offset_pos_cnt = yamlGetInt(trans, "offset_pos_cnt", 0);
        d.transform.cnt_per_unit   = yamlGetDouble(trans, "cnt_per_unit", 1.0);
        d.transform.torque_per_unit = yamlGetDouble(trans, "torque_per_unit", 1.0);
        d.transform.user_unit_name = yamlGetStr(trans, "user_unit_name", "rad");
    }

    // 解析 inputs 子节点（PDO 变量名映射）
    if (node["inputs"]) {
        const auto& in = node["inputs"];
        d.inputs.status_word              = yamlGetStr(in, "status_word");
        d.inputs.position_actual_value    = yamlGetStr(in, "position_actual_value");
        d.inputs.velocity_actual_value    = yamlGetStr(in, "velocity_actual_value");
        d.inputs.torque_actual_value      = yamlGetStr(in, "torque_actual_value");
        d.inputs.load_torque_value        = yamlGetStr(in, "load_torque_value");
        d.inputs.secondary_position_value = yamlGetStr(in, "secondary_position_value");
        d.inputs.digital_inputs           = yamlGetStr(in, "digital_inputs");
        d.inputs.digital_outputs          = yamlGetStr(in, "digital_outputs");
    }

    // 解析 outputs 子节点（PDO 变量名映射）
    if (node["outputs"]) {
        const auto& out = node["outputs"];
        d.outputs.control_word      = yamlGetStr(out, "control_word");
        d.outputs.mode_of_operation = yamlGetStr(out, "mode_of_operation");
        d.outputs.target_position   = yamlGetStr(out, "target_position");
        d.outputs.target_velocity   = yamlGetStr(out, "target_velocity");
        d.outputs.target_torque     = yamlGetStr(out, "target_torque");
        d.outputs.digital_outputs   = yamlGetStr(out, "digital_outputs");
    }

    return d;
}

FTSensor Hardware::parseFTSensor(const YAML::Node& node) {
    FTSensor ft;
    ft.id = node["id"].as<int>();

    // 解析 6 通道 PDO 变量名映射
    if (node["inputs"]) {
        const auto& in = node["inputs"];
        ft.inputs.fx = yamlGetStr(in, "fx");
        ft.inputs.fy = yamlGetStr(in, "fy");
        ft.inputs.fz = yamlGetStr(in, "fz");
        ft.inputs.tx = yamlGetStr(in, "tx");
        ft.inputs.ty = yamlGetStr(in, "ty");
        ft.inputs.tz = yamlGetStr(in, "tz");
    }

    // 解析标定偏置值
    if (node["offset"]) {
        const auto& off = node["offset"];
        ft.offset = Wrench(
            KDL::Vector(yamlGetDouble(off, "fx", 0.0),
                        yamlGetDouble(off, "fy", 0.0),
                        yamlGetDouble(off, "fz", 0.0)),
            KDL::Vector(yamlGetDouble(off, "tx", 0.0),
                        yamlGetDouble(off, "ty", 0.0),
                        yamlGetDouble(off, "tz", 0.0))
        );
    }

    return ft;
}

IO Hardware::parseIO(const YAML::Node& node) {
    IO io;
    io.id = node["id"].as<int>();

    // IO 通道数
    io.digital_in_channels  = yamlGetInt(node, "digital_in_channels", 0);
    io.digital_out_channels = yamlGetInt(node, "digital_out_channels", 0);
    io.analog_in_channels   = yamlGetInt(node, "analog_in_channels", 0);
    io.analog_out_channels  = yamlGetInt(node, "analog_out_channels", 0);

    // 输入 PDO 映射
    if (node["inputs"]) {
        const auto& in = node["inputs"];
        io.inputs.digital_inputs  = yamlGetStr(in, "digital_inputs");
        io.inputs.digital_outputs = yamlGetStr(in, "digital_outputs");
        io.inputs.analog_inputs   = yamlGetStr(in, "analog_inputs");
    }

    // 输出 PDO 映射
    if (node["outputs"]) {
        const auto& out = node["outputs"];
        io.outputs.digital_outputs = yamlGetStr(out, "digital_outputs");
        io.outputs.analog_outputs  = yamlGetStr(out, "analog_outputs");
    }

    return io;
}

// ==========================================================================
// 构造 / 析构
// ==========================================================================

Hardware::Hardware(const std::string& yaml_file_path, int ecat_id)
    : Hardware(loadFromYAML(yaml_file_path), ecat_id) {}

Hardware::Hardware(const HardwareConfig& config, int ecat_id) : config_(config) {
    auto log_ptr = Logger::getInstance("Hardware");

    // 2. 构建 ID → 索引 O(1) 查找表（vector 预分配 + -1 哨兵）
    buildIDLookupTables();

    // 3. 连接 EtherCAT 共享内存
    ec_ptr_ = EcatConfig::getInstance(ecat_id);
    if (ec_ptr_ == nullptr) {
        log_ptr->error("Failed to get EcatConfig instance (id={})", ecat_id);
        throw std::runtime_error("EcatConfig initialization failed");
    }

    // v2 连接已验证固定头部和 PDO 布局；等待首个周期通知。
    WaitForSignal();

    // 5. 初始化阶段：一次性解析所有 PDO 变量名 → 缓存索引
    //    运行阶段通过缓存索引读写，不再有字符串查找开销
    buildPDOCache();

    // 6. 初始化 CiA 402 状态机（与 drive_pdo_cache_ 并行，同一索引）
    drive_sm_state_.resize(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        if (drive_pdo_cache_[i].status_word) {
            Statusword sw;
            sw.setFromRawStatusword(drive_pdo_cache_[i].status_word.read());
            drive_sm_state_[i].targetState = sw.getDriveState();
        }

        // 上使能前将目标位置同步为当前位置，防止 CSP 模式下飞车
        if (drive_pdo_cache_[i].target_position && drive_pdo_cache_[i].position_actual_value) {
            drive_pdo_cache_[i].target_position.write(drive_pdo_cache_[i].position_actual_value.read());
        }
    }

    // 7. 启动状态机后台线程
    sm_thread_running_ = true;
    sm_thread_ = std::make_unique<std::thread>(&Hardware::smWorkingThread, this);

    log_ptr->info("Hardware initialized: {} drives, {} ft_sensors, {} io modules",
                  config_.drives.size(),
                  config_.ft_sensors.size(),
                  config_.ios.size());
}

Hardware::~Hardware() {
    sm_thread_running_ = false;
    if (sm_thread_ && sm_thread_->joinable()) {
        sm_thread_->join();
    }
    // EcatConfig 是单例，不由 Hardware 管理生命周期
}

void Hardware::WaitForSignal() {
    if (!ec_ptr_->waitForSignal(8)) {
        throw std::runtime_error("EtherCAT v2 cycle wait failed");
    }
}

uint32_t Hardware::GetDt() const {
    // mujoco 写 ecatBus->dt → 共享内存 → app Hardware::GetDt()
    if (ec_ptr_ == nullptr || ec_ptr_->ecatBus == nullptr) {
        return 0;
    }
    return ec_ptr_->getDt();
}

bool Hardware::Reset() {

  return true;
}

Result Hardware::ClearFault() {
    for (const auto& drive : config_.drives) {
        const auto idx = getDriveIdx(drive.id);
        if (idx < 0) {
            return Result::JointStateError;
        }

        if (!setDriverState(static_cast<size_t>(idx), DriveState::SwitchOnDisabled)) {
            return Result::JointStateError;
        }
    }

    return Result::NoError;
}

// ==========================================================================
// YAML 加载
// ==========================================================================

HardwareConfig Hardware::loadFromYAML(const std::string& yaml_file_path) {
    return LoadConfigFromNode(YAML::LoadFile(yaml_file_path));
}

HardwareConfig Hardware::LoadConfigFromNode(const YAML::Node& root) {
    // 纯解析阶段不初始化日志系统或连接共享内存。
    HardwareConfig config;
    const auto list = root["hardware"];
    if (!list || !list.IsSequence()) {
        throw std::runtime_error("Invalid hardware YAML: missing 'hardware' sequence");
    }
    for (const auto& node : list) {
        if (!node["type"]) continue;
        const auto type = node["type"].as<std::string>();
        if (type == "driver") config.drives.push_back(parseDrive(node));
        else if (type == "ft_sensor") config.ft_sensors.push_back(parseFTSensor(node));
        else if (type == "io") config.ios.push_back(parseIO(node));
    }
    return config;
}

HardwareConfig Hardware::LoadConfigFromYAML(const std::string& yaml_file_path) {
    return loadFromYAML(yaml_file_path);
}

// ==========================================================================
// 坐标变换辅助函数
// ==========================================================================

double Hardware::cntToUnit(const Drive& drive, int32_t raw_cnt) const {
    // 物理值 = (原始值 - 偏移) / cnt_per_unit / ratio
    double raw = static_cast<double>(raw_cnt - drive.transform.offset_pos_cnt);
    return raw / drive.transform.cnt_per_unit / drive.transform.ratio;
}

int32_t Hardware::unitToCnt(const Drive& drive, double unit_val) const {
    // 原始值 = 物理值 * cnt_per_unit * ratio + 偏移
    double raw = unit_val * drive.transform.cnt_per_unit * drive.transform.ratio;
    return static_cast<int32_t>(raw) + drive.transform.offset_pos_cnt;
}

double Hardware::torqueToUnit(const Drive& drive, int16_t raw_torque) const {
    // 力矩物理值 = 原始值 / torque_per_unit
    return static_cast<double>(raw_torque) / drive.transform.torque_per_unit;
}

// ==========================================================================
// DriveInterface — 批量操作
// ==========================================================================

JntArray Hardware::GetPosition() {
    JntArray q(ModelJointCount());
    for (size_t i = 0; i < ModelJointCount(); ++i) {
        q(i) = GetJointPosition(DriveIdByModelIndex(i));
    }
    return q;
}

JntArray Hardware::GetVelocity() {
    JntArray q_dot(ModelJointCount());
    for (size_t i = 0; i < ModelJointCount(); ++i) {
        q_dot(i) = GetJointVelocity(DriveIdByModelIndex(i));
    }
    return q_dot;
}

JntArray Hardware::GetTorque() {
    JntArray tau(ModelJointCount());
    for (size_t i = 0; i < ModelJointCount(); ++i) {
        tau(i) = GetJointTorque(DriveIdByModelIndex(i));
    }
    return tau;
}

JntArray Hardware::GetLoadTorque() {
    JntArray lt(ModelJointCount());
    for (size_t i = 0; i < ModelJointCount(); ++i) {
        lt(i) = GetJointLoadTorque(DriveIdByModelIndex(i));
    }
    return lt;
}

void Hardware::SetPosition(const JntArray& q) {
    for (size_t i = 0; i < ModelJointCount() && i < q.rows(); ++i) {
        SetJointPosition(DriveIdByModelIndex(i), q(i));
    }
}

void Hardware::SetVelocity(const JntArray& q_dot) {
    for (size_t i = 0; i < ModelJointCount() && i < q_dot.rows(); ++i) {
        SetJointVelocity(DriveIdByModelIndex(i), q_dot(i));
    }
}

void Hardware::SetTorque(const JntArray& tau) {
    for (size_t i = 0; i < ModelJointCount() && i < tau.rows(); ++i) {
        SetJointTorque(DriveIdByModelIndex(i), tau(i));
    }
}

void Hardware::SetMode(int8_t mode) {
    for (auto& drive : config_.drives) {
        SetJointMode(drive.id, mode);
    }
}

void Hardware::SetEnabled() {
    for (auto& drive : config_.drives) {
        SetJointEnabled(drive.id);
    }
}

void Hardware::SetDisabled() {
    for (auto& drive : config_.drives) {
        SetJointDisabled(drive.id);
    }
}

JntState Hardware::GetState() {
    JntState state = JntState::ENABLED;
    for (const auto& drive : config_.drives) {
        const JntState joint_state = GetJointState(drive.id);
        if (joint_state == JntState::ERROR) {
            return JntState::ERROR;
        }
        if (joint_state == JntState::DISABLED) {
            state = JntState::DISABLED;
        }
    }
    return state;
}

// ==========================================================================
// DriveInterface — 单关节操作（使用缓存 PDO 索引，无字符串查找）
// ==========================================================================

double Hardware::GetJointPosition(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.position_actual_value) return 0.0;
    return cntToUnit(config_.drives[static_cast<size_t>(idx)], cache.position_actual_value.read());
}

double Hardware::GetJointVelocity(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.velocity_actual_value) return 0.0;
    return cntToUnit(config_.drives[static_cast<size_t>(idx)], cache.velocity_actual_value.read());
}

double Hardware::GetJointTorque(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.torque_actual_value) return 0.0;
    return torqueToUnit(config_.drives[static_cast<size_t>(idx)], cache.torque_actual_value.read());
}

double Hardware::GetJointLoadTorque(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& drive = config_.drives[static_cast<size_t>(idx)];
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];

    // 根据 torque_source 选择读取通道，对调用者透明
    if (drive.torque_source == TorqueSource::SecondaryPosition) {
        if (!cache.secondary_position_value) return 0.0;
        return cntToUnit(drive, cache.secondary_position_value.read());
    } else {
        if (!cache.load_torque_value) return 0.0;
        return torqueToUnit(drive, cache.load_torque_value.read());
    }
}

void Hardware::SetJointPosition(int32_t id, double pos) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.target_position) return;
    cache.target_position.write(unitToCnt(config_.drives[static_cast<size_t>(idx)], pos));
}

void Hardware::SetJointVelocity(int32_t id, double vel) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.target_velocity) return;
    // 速度值也使用 cnt_per_unit 变换（单位: unit/s → cnt/s）
    double raw_val = vel * config_.drives[static_cast<size_t>(idx)].transform.cnt_per_unit
                     * config_.drives[static_cast<size_t>(idx)].transform.ratio;
    cache.target_velocity.write(static_cast<int32_t>(raw_val));
}

void Hardware::SetJointTorque(int32_t id, double tau) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.target_torque) return;
    cache.target_torque.write(
        static_cast<int16_t>(tau * config_.drives[static_cast<size_t>(idx)].transform.torque_per_unit));
}

void Hardware::SetJointMode(int32_t id, int8_t mode) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.mode_of_operation) return;
    cache.mode_of_operation.write(mode);
}

JntState Hardware::GetJointState(int32_t id) {
    const auto idx = getDriveIdx(id);
    if (idx < 0) {
        return JntState::ERROR;
    }

    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (!cache.status_word) {
        return JntState::ERROR;
    }

    Statusword statusword;
    statusword.setFromRawStatusword(cache.status_word.read());
    switch (statusword.getDriveState()) {
        case DriveState::OperationEnabled:
            return JntState::ENABLED;
        case DriveState::FaultReactionActive:
        case DriveState::Fault:
        case DriveState::NA:
            return JntState::ERROR;
        default:
            return JntState::DISABLED;
    }
}

// ==========================================================================
// CiA 402 状态机 
// ==========================================================================

Controlword Hardware::getNextStateTransitionControlword(
    const DriveState& requestedDriveState,
    const DriveState& currentDriveState) {
    Controlword controlword;
    controlword.setAllFalse();
    switch (requestedDriveState) {
        case DriveState::SwitchOnDisabled:
            switch (currentDriveState) {
                case DriveState::SwitchOnDisabled:
                    break;
                case DriveState::ReadyToSwitchOn:
                    controlword.setStateTransition7(); break;
                case DriveState::SwitchedOn:
                    controlword.setStateTransition10(); break;
                case DriveState::OperationEnabled:
                    controlword.setStateTransition9(); break;
                case DriveState::QuickStopActive:
                    controlword.setStateTransition12(); break;
                case DriveState::Fault:
                    controlword.setStateTransition15(); break;
                default: break;
            }
            break;

        case DriveState::ReadyToSwitchOn:
            switch (currentDriveState) {
                case DriveState::SwitchOnDisabled:
                    controlword.setStateTransition2(); break;
                case DriveState::ReadyToSwitchOn:
                    break;
                case DriveState::SwitchedOn:
                    controlword.setStateTransition6(); break;
                case DriveState::OperationEnabled:
                    controlword.setStateTransition8(); break;
                case DriveState::QuickStopActive:
                    controlword.setStateTransition12(); break;
                case DriveState::Fault:
                    controlword.setStateTransition15(); break;
                default: break;
            }
            break;

        case DriveState::SwitchedOn:
            switch (currentDriveState) {
                case DriveState::SwitchOnDisabled:
                    controlword.setStateTransition2(); break;
                case DriveState::ReadyToSwitchOn:
                    controlword.setStateTransition3(); break;
                case DriveState::SwitchedOn:
                    break;
                case DriveState::OperationEnabled:
                    controlword.setStateTransition5(); break;
                case DriveState::QuickStopActive:
                    controlword.setStateTransition12(); break;
                case DriveState::Fault:
                    controlword.setStateTransition15(); break;
                default: break;
            }
            break;

        case DriveState::OperationEnabled:
            switch (currentDriveState) {
                case DriveState::SwitchOnDisabled:
                    controlword.setStateTransition2(); break;
                case DriveState::ReadyToSwitchOn:
                    controlword.setStateTransition3(); break;
                case DriveState::SwitchedOn:
                    controlword.setStateTransition4(); break;
                case DriveState::OperationEnabled:
                    break;
                case DriveState::QuickStopActive:
                    controlword.setStateTransition12(); break;
                case DriveState::Fault:
                    controlword.setStateTransition15(); break;
                default: break;
            }
            break;

        case DriveState::QuickStopActive:
            switch (currentDriveState) {
                case DriveState::SwitchOnDisabled:
                    controlword.setStateTransition2(); break;
                case DriveState::ReadyToSwitchOn:
                    controlword.setStateTransition3(); break;
                case DriveState::SwitchedOn:
                    controlword.setStateTransition4(); break;
                case DriveState::OperationEnabled:
                    controlword.setStateTransition11(); break;
                case DriveState::QuickStopActive:
                    break;
                case DriveState::Fault:
                    controlword.setStateTransition15(); break;
                default: break;
            }
            break;

        default: break;
    }
    return controlword;
}

// ==========================================================================
// 驱动器状态切换 
// ==========================================================================

void Hardware::engageStateMachine(size_t idx) {
    auto& sm = drive_sm_state_[idx];
    auto& cache = drive_pdo_cache_[idx];
    if (!cache.status_word || !cache.control_word) return;

    Statusword statusword;
    statusword.setFromRawStatusword(cache.status_word.read());
    DriveState currentState = statusword.getDriveState();

    // 已达目标状态，连续确认 3 次后标记成功（与 old_drive 一致）
    if (currentState == sm.targetState) {
        sm.numOfSuccessfulReadings++;
        if (sm.numOfSuccessfulReadings >= 3) {
            sm.conductStateChange = false;
            sm.numOfSuccessfulReadings = 0;
            sm.stateChangeSuccessful = true;
        }
        return;  // 状态已达目标，不再发送控制字
    }

    sm.numOfSuccessfulReadings = 0;

    // 距上次控制字变更需等待 20ms（与 old_drive 20000us 一致）
    auto now = std::chrono::system_clock::now();
    auto since = std::chrono::duration_cast<std::chrono::microseconds>(
        now - sm.lastCwChangeTime).count();
    if (since < 20000) return;

    Controlword controlword =
        getNextStateTransitionControlword(sm.targetState, currentState);
    cache.control_word.write(controlword.getRawControlword());
    sm.lastCwChangeTime = now;
}

bool Hardware::setDriverState(size_t idx, const DriveState targetState) {
    auto& sm = drive_sm_state_[idx];
    sm.stateChangeSuccessful = false;
    sm.conductStateChange = true;
    sm.targetState = targetState;
    sm.lastCwChangeTime = std::chrono::system_clock::now();

    // 阻塞等待状态切换完成
    auto start = std::chrono::system_clock::now();
    while (true) {
        if (sm.stateChangeSuccessful) return true;
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now() - start).count();
        if (elapsed > 150000) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Hardware::smWorkingThread() {
    while (sm_thread_running_) {
        for (size_t i = 0; i < drive_sm_state_.size(); ++i) {
            if (drive_sm_state_[i].conductStateChange) {
                engageStateMachine(i);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Hardware::SetJointEnabled(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    setDriverState(static_cast<size_t>(idx), DriveState::OperationEnabled);
}

void Hardware::SetJointDisabled(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    setDriverState(static_cast<size_t>(idx), DriveState::SwitchOnDisabled);
}

std::string Hardware::getJointName(int32_t model_index) {
    if (model_index < 0 || static_cast<size_t>(model_index) >= ModelJointCount()) {
        return "";
    }
    const int32_t drive_id = DriveIdByModelIndex(static_cast<size_t>(model_index));
    auto idx = getDriveIdx(drive_id);
    if (idx < 0) return "";
    return config_.drives[static_cast<size_t>(idx)].joint_name;
}

// ==========================================================================
// Joint binding 接口实现
// ==========================================================================

Result Hardware::SetJointBinding(const std::vector<int32_t>& model_index_to_drive_id) {
    auto log_ptr = Logger::getInstance("Hardware");
    if (model_index_to_drive_id.empty()) {
        log_ptr->error("SetJointBinding failed: mapping table is empty");
        return Result::IllegalParameter;
    }
    if (model_index_to_drive_id.size() > config_.drives.size()) {
        log_ptr->error("SetJointBinding failed: model joint count ({}) > hardware drive count ({})",
                       model_index_to_drive_id.size(), config_.drives.size());
        return Result::IllegalParameter;
    }

    // 检查每个 drive id 都存在且不重复
    std::set<int32_t> seen;
    for (size_t model_index = 0; model_index < model_index_to_drive_id.size(); ++model_index) {
        const int32_t drive_id = model_index_to_drive_id[model_index];
        auto idx = getDriveIdx(drive_id);
        if (idx < 0) {
            log_ptr->error("SetJointBinding failed: model index {} references unknown drive id {}",
                           model_index, drive_id);
            return Result::IllegalParameter;
        }
        if (!seen.insert(drive_id).second) {
            log_ptr->error("SetJointBinding failed: duplicated drive id {} at model index {}",
                           drive_id, model_index);
            return Result::IllegalParameter;
        }
    }

    model_index_to_drive_id_ = model_index_to_drive_id;
    log_ptr->info("SetJointBinding success: model_index_to_drive_id=[{}]",
                  formatDriveIdList(model_index_to_drive_id_));
    return Result::NoError;
}

void Hardware::ClearJointBinding() {
    model_index_to_drive_id_.clear();
    auto log_ptr = Logger::getInstance("Hardware");
    log_ptr->info("Joint binding cleared; hardware will use default drive order");
}

std::vector<int32_t> Hardware::GetJointBinding() const {
    return model_index_to_drive_id_;
}

int Hardware::GetDriveNum() const {
    return static_cast<int>(config_.drives.size());
}

std::vector<int32_t> Hardware::GetDriveIds() const {
    std::vector<int32_t> ids;
    ids.reserve(config_.drives.size());
    for (const auto& drive : config_.drives) {
        ids.push_back(drive.id);
    }
    return ids;
}

// ==========================================================================
// FTSensorInterface（使用缓存 PDO 索引，无字符串查找）
// ==========================================================================

Wrench Hardware::GetWrench() {
    if (ft_sensor_pdo_cache_.empty()) {
        return Wrench::Zero();
    }

    const auto& cache = ft_sensor_pdo_cache_[0];
    const auto& ft = config_.ft_sensors[0];

    // 安全读取 PDO 通道：未绑定 → 返回 0.0
    auto safeRead = [](const PdoEntry<int16_t>& entry) -> double {
        return entry ? static_cast<double>(entry.read()) : 0.0;
    };

    return Wrench(
        KDL::Vector(safeRead(cache.fx) + ft.offset.force.x(),
                    safeRead(cache.fy) + ft.offset.force.y(),
                    safeRead(cache.fz) + ft.offset.force.z()),
        KDL::Vector(safeRead(cache.tx) + ft.offset.torque.x(),
                    safeRead(cache.ty) + ft.offset.torque.y(),
                    safeRead(cache.tz) + ft.offset.torque.z())
    );
}

// ==========================================================================
// IOInteface（使用缓存 PDO 索引，无字符串查找）
// ==========================================================================

bool Hardware::GetDigitalInput(int32_t id, int32_t channel) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.digital_in_channels) return false;
            const auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (!cache.digital_inputs) return false;
            return (cache.digital_inputs.read() >> channel) & 0x01;
        }
    }

    // 2. 在驱动器中查找（驱动器自带 DI）
    {
        auto idx = getDriveIdx(id);
        if (idx >= 0) {
            const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
            if (!cache.digital_inputs) return false;
            return (cache.digital_inputs.read() >> channel) & 0x01;
        }
    }

    return false;
}

void Hardware::SetDigitalOutput(int32_t id, int32_t channel, bool value) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.digital_out_channels) return;
            auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (!cache.digital_outputs_input || !cache.digital_outputs) return;
            int32_t current = cache.digital_outputs_input.read();
            if (value) {
                current |= (1 << channel);
            } else {
                current &= ~(1 << channel);
            }
            cache.digital_outputs.write(current);
            return;
        }
    }

    // 2. 在驱动器中查找（驱动器自带 DO）
    {
        auto idx = getDriveIdx(id);
        if (idx >= 0) {
            auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
            if (!cache.digital_outputs_input || !cache.digital_outputs) return;
            int32_t current = cache.digital_outputs_input.read();
            if (value) {
                current |= (1 << channel);
            } else {
                current &= ~(1 << channel);
            }
            cache.digital_outputs.write(current);
            return;
        }
    }
}

double Hardware::GetAnalogInput(int32_t id, int32_t channel) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.analog_in_channels) return 0.0;
            const auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (!cache.analog_inputs) return 0.0;
            return static_cast<double>(cache.analog_inputs.read());
        }
    }

    return 0.0;
}

void Hardware::SetAnalogOutput(int32_t id, int32_t channel, double value) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.analog_out_channels) return;
            auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (!cache.analog_outputs) return;
            cache.analog_outputs.write(static_cast<int16_t>(value));
            return;
        }
    }
}

// ==========================================================================
// 查找表构建（初始化阶段）
// ==========================================================================

void Hardware::buildIDLookupTables() {
    // Lambda: 根据 items 的 id 构建 max_id+1 大小的 vector，哨兵值 -1
    auto build = [](const auto& items, std::vector<int32_t>& table) {
        table.clear();
        if (items.empty()) return;
        int32_t max_id = -1;
        for (const auto& item : items) {
            if (item.id > max_id) max_id = item.id;
        }
        if (max_id < 0) return;
        table.assign(static_cast<size_t>(max_id) + 1, -1);
        for (size_t i = 0; i < items.size(); ++i) {
            table[static_cast<size_t>(items[i].id)] = static_cast<int32_t>(i);
        }
    };

    build(config_.drives, drive_id_to_index_);
    build(config_.ft_sensors, ft_sensor_id_to_index_);
    build(config_.ios, io_id_to_index_);

    // 名称查找仍用 unordered_map（string key 无法用 vector）
    drive_name_to_index_.clear();
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        drive_name_to_index_[config_.drives[i].joint_name] = i;
    }
}

void Hardware::buildPDOCache() {
    auto log_ptr = Logger::getInstance("Hardware");

    // ========== 1. 驱动器 PDO 索引缓存 ==========
    drive_pdo_cache_.resize(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        const auto& drive = config_.drives[i];
        auto& cache = drive_pdo_cache_[i];
        const int sid = drive.id;

        // 输入 PDO（从站 → 主站）
        if (!drive.inputs.status_word.empty())
            cache.status_word = PdoEntry<uint16_t>::input(*ec_ptr_, sid, drive.inputs.status_word);
        if (!drive.inputs.position_actual_value.empty())
            cache.position_actual_value =
                PdoEntry<int32_t>::input(*ec_ptr_, sid, drive.inputs.position_actual_value);
        if (!drive.inputs.velocity_actual_value.empty())
            cache.velocity_actual_value =
                PdoEntry<int32_t>::input(*ec_ptr_, sid, drive.inputs.velocity_actual_value);
        if (!drive.inputs.torque_actual_value.empty())
            cache.torque_actual_value =
                PdoEntry<int16_t>::input(*ec_ptr_, sid, drive.inputs.torque_actual_value);
        if (!drive.inputs.load_torque_value.empty())
            cache.load_torque_value = PdoEntry<int16_t>::input(*ec_ptr_, sid, drive.inputs.load_torque_value);
        if (!drive.inputs.secondary_position_value.empty())
            cache.secondary_position_value =
                PdoEntry<int32_t>::input(*ec_ptr_, sid, drive.inputs.secondary_position_value);
        if (!drive.inputs.digital_inputs.empty())
            cache.digital_inputs = PdoEntry<int32_t>::input(*ec_ptr_, sid, drive.inputs.digital_inputs);
        if (!drive.inputs.digital_outputs.empty())
            cache.digital_outputs_input =
                PdoEntry<int32_t>::input(*ec_ptr_, sid, drive.inputs.digital_outputs);

        // 输出 PDO（主站 → 从站）
        if (!drive.outputs.control_word.empty())
            cache.control_word = PdoEntry<uint16_t>::output(*ec_ptr_, sid, drive.outputs.control_word);
        if (!drive.outputs.mode_of_operation.empty())
            cache.mode_of_operation =
                PdoEntry<int8_t>::output(*ec_ptr_, sid, drive.outputs.mode_of_operation);
        if (!drive.outputs.target_position.empty())
            cache.target_position = PdoEntry<int32_t>::output(*ec_ptr_, sid, drive.outputs.target_position);
        if (!drive.outputs.target_velocity.empty())
            cache.target_velocity = PdoEntry<int32_t>::output(*ec_ptr_, sid, drive.outputs.target_velocity);
        if (!drive.outputs.target_torque.empty())
            cache.target_torque = PdoEntry<int16_t>::output(*ec_ptr_, sid, drive.outputs.target_torque);
        if (!drive.outputs.digital_outputs.empty())
            cache.digital_outputs = PdoEntry<int32_t>::output(*ec_ptr_, sid, drive.outputs.digital_outputs);

        // 关键 PDO 变量未找到时记录警告
        if (!cache.position_actual_value) {
            log_ptr->warn("Drive {}: position_actual_value PDO entry is missing (var='{}')", sid,
                          drive.inputs.position_actual_value);
        }
        if (!cache.control_word) {
            log_ptr->warn("Drive {}: control_word PDO entry is missing (var='{}')", sid,
                          drive.outputs.control_word);
        }
    }

    // ========== 2. 力传感器 PDO 索引缓存 ==========
    ft_sensor_pdo_cache_.resize(config_.ft_sensors.size());
    for (size_t i = 0; i < config_.ft_sensors.size(); ++i) {
        const auto& ft = config_.ft_sensors[i];
        auto& cache = ft_sensor_pdo_cache_[i];
        const int sid = ft.id;

        if (!ft.inputs.fx.empty()) cache.fx = PdoEntry<int16_t>::input(*ec_ptr_, sid, ft.inputs.fx);
        if (!ft.inputs.fy.empty()) cache.fy = PdoEntry<int16_t>::input(*ec_ptr_, sid, ft.inputs.fy);
        if (!ft.inputs.fz.empty()) cache.fz = PdoEntry<int16_t>::input(*ec_ptr_, sid, ft.inputs.fz);
        if (!ft.inputs.tx.empty()) cache.tx = PdoEntry<int16_t>::input(*ec_ptr_, sid, ft.inputs.tx);
        if (!ft.inputs.ty.empty()) cache.ty = PdoEntry<int16_t>::input(*ec_ptr_, sid, ft.inputs.ty);
        if (!ft.inputs.tz.empty()) cache.tz = PdoEntry<int16_t>::input(*ec_ptr_, sid, ft.inputs.tz);
    }

    // ========== 3. IO 模块 PDO 索引缓存 ==========
    io_pdo_cache_.resize(config_.ios.size());
    for (size_t i = 0; i < config_.ios.size(); ++i) {
        const auto& io = config_.ios[i];
        auto& cache = io_pdo_cache_[i];
        const int sid = io.id;

        if (!io.inputs.digital_inputs.empty())
            cache.digital_inputs = PdoEntry<int32_t>::input(*ec_ptr_, sid, io.inputs.digital_inputs);
        if (!io.inputs.digital_outputs.empty())
            cache.digital_outputs_input = PdoEntry<int32_t>::input(*ec_ptr_, sid, io.inputs.digital_outputs);
        if (!io.inputs.analog_inputs.empty())
            cache.analog_inputs = PdoEntry<int16_t>::input(*ec_ptr_, sid, io.inputs.analog_inputs);
        if (!io.outputs.digital_outputs.empty())
            cache.digital_outputs = PdoEntry<int32_t>::output(*ec_ptr_, sid, io.outputs.digital_outputs);
        if (!io.outputs.analog_outputs.empty())
            cache.analog_outputs = PdoEntry<int16_t>::output(*ec_ptr_, sid, io.outputs.analog_outputs);
    }
}

// ==========================================================================
// 配置查询
// ==========================================================================

const Drive* Hardware::findDriveById(int32_t id) const {
    auto idx = getDriveIdx(id);
    return (idx >= 0) ? &config_.drives[static_cast<size_t>(idx)] : nullptr;
}

const Drive* Hardware::findDriveByName(const std::string& name) const {
    auto it = drive_name_to_index_.find(name);
    if (it == drive_name_to_index_.end()) {
        return nullptr;
    }
    return &config_.drives[it->second];
}

const FTSensor* Hardware::findFTSensorById(int32_t id) const {
    auto idx = getFTSensorIdx(id);
    return (idx >= 0) ? &config_.ft_sensors[static_cast<size_t>(idx)] : nullptr;
}

const IO* Hardware::findIOById(int32_t id) const {
    auto idx = getIOIdx(id);
    return (idx >= 0) ? &config_.ios[static_cast<size_t>(idx)] : nullptr;
}

HardwareType Hardware::getHardwareType(int32_t slave_id) const {
    if (getDriveIdx(slave_id) >= 0) {
        return HardwareType::Driver;
    }
    if (getFTSensorIdx(slave_id) >= 0) {
        return HardwareType::FTSensor;
    }
    if (getIOIdx(slave_id) >= 0) {
        return HardwareType::IO;
    }
    // 默认返回 Driver 类型
    return HardwareType::Driver;
}

}  // namespace rocos
