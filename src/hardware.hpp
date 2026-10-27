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
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ecat_type.hpp"
#include "hardware_interface.hpp"
#include "pdo_entry.hpp"

// 前向声明，避免在头文件中引入 yaml-cpp
namespace YAML {
class Node;
}

namespace rocos {

// ==========================================================================
// 枚举定义
// ==========================================================================

/// @brief 扭矩传感器数据来源 —— 对上层使用者透明
enum class TorqueSource {
    LoadTorque,         // 通过 load_torque_value（int16 Analog Input 原始值）
    SecondaryPosition   // 通过 secondary_position_value（底层已做滤波）
};

/// @brief 硬件模块类型
enum class HardwareType {
    Driver,     // 伺服驱动器
    FTSensor,   // 力/力矩传感器
    IO          // 独立 IO 模块
};

// ==========================================================================
// 驱动器配置结构体
// ==========================================================================
struct Drive {
    int id{-1};                        // EtherCAT 从站 ID
    std::string joint_name;            // 对应的关节名称

    /// @brief 关节力矩传感器数据来源 —— 对使用者透明
    /// load_torque:    从 load_torque_value 读取 int16 原始 Analog Input
    /// secondary_pos:  从 secondary_position_value 读取底层滤波后的数据
    TorqueSource torque_source = TorqueSource::LoadTorque;

    /// @brief 运动限位参数
    struct Limit {
        double lower = -2.71;          // 位置下限 [rad] 或 [m]
        double upper = 2.71;           // 位置上限 [rad] 或 [m]
        double vel = 3.0;              // 最大速度
        double acc = 10.0;             // 最大加速度
        double jerk = 150.0;           // 最大加加速度
        double effort = 30.0;          // 最大力矩/力
    } limit;

    /// @brief 编码器/力矩变换参数（cnt ↔ 物理单位）
    struct Transform {
        double ratio = 1.0;            // 传动比（关节侧/电机侧）
        int offset_pos_cnt = 0;        // 编码器零点偏移 [cnt]
        double cnt_per_unit = 1.0;     // 每物理单位对应编码器计数
        double torque_per_unit = 1.0;  // 每物理单位对应力矩值
        std::string user_unit_name = "rad";  // 用户单位名称
    } transform;

    /// @brief 输入 PDO 变量名（从站 → 主站），需与 ethercat 配置中的名称一致
    struct InputVar {
        std::string status_word;
        std::string position_actual_value;
        std::string velocity_actual_value;
        std::string torque_actual_value;
        std::string load_torque_value;           // 负载力矩（可用于力矩传感器）
        std::string secondary_position_value;    // 辅助位置（可用于底层滤波后的力矩）
        std::string digital_inputs;              // 数字输入（DI）
        std::string digital_outputs;             // 数字输出回读（DO 状态反馈）
    } inputs;

    /// @brief 输出 PDO 变量名（主站 → 从站），需与 ethercat 配置中的名称一致
    struct OutputVar {
        std::string control_word;
        std::string mode_of_operation;
        std::string target_position;
        std::string target_velocity;
        std::string target_torque;
        std::string digital_outputs;             // 数字输出（DO）
    } outputs;

};

// ==========================================================================
// 力/力矩传感器配置结构体
// ==========================================================================
struct FTSensor {
    int id{-1};                        // EtherCAT 从站 ID

    /// @brief 输入 PDO 变量名（从站 → 主站），6 通道映射
    struct InputVar {
        std::string fx;    // Fx 通道
        std::string fy;    // Fy 通道
        std::string fz;    // Fz 通道
        std::string tx;    // Tx 通道
        std::string ty;    // Ty 通道
        std::string tz;    // Tz 通道
    } inputs;

    /// @brief 六维力/力矩标定偏置值 (Fx,Fy,Fz,Tx,Ty,Tz)
    Wrench offset;
};

// ==========================================================================
// IO 模块配置结构体
// ==========================================================================
struct IO {
    int id{-1};                        // EtherCAT 从站 ID

    /// @brief IO 通道数量（用于参数校验与能力查询）
    int digital_in_channels = 0;       // DI 通道数
    int digital_out_channels = 0;      // DO 通道数
    int analog_in_channels = 0;        // AI 通道数
    int analog_out_channels = 0;       // AO 通道数

    /// @brief 输入 PDO 变量名（从站 → 主站）
    struct InputVar {
        std::string digital_inputs;    // DI 值（位掩码）
        std::string digital_outputs;   // DO 回读（位掩码）
        std::string analog_inputs;     // AI 值（通常为 int16 数组）
    } inputs;

    /// @brief 输出 PDO 变量名（主站 → 从站）
    struct OutputVar {
        std::string digital_outputs;   // DO 值（位掩码）
        std::string analog_outputs;    // AO 值（通常为 int16 数组）
    } outputs;
};

// ==========================================================================
// 硬件配置（YAML 顶层结构）
// ==========================================================================
struct HardwareConfig {
    std::vector<Drive> drives;         // 所有伺服驱动器
    std::vector<FTSensor> ft_sensors;  // 所有力/力矩传感器
    std::vector<IO> ios;               // 所有独立 IO 模块
};

// ==========================================================================
// 硬件抽象层
// ==========================================================================
class Hardware : public HardwareInterface {
public:
    /// @brief 从 YAML 配置文件构造硬件抽象层
    /// @param yaml_file_path 硬件描述 YAML 文件路径
    /// @param ecat_id         EtherCAT 主站 ID（默认 0）
    explicit Hardware(const std::string& yaml_file_path, int ecat_id = 0);
    explicit Hardware(const HardwareConfig& config, int ecat_id = 0);
    ~Hardware();


    bool Reset() override;

    /// @brief 清除驱动报警，使 Fault 驱动回到可重新使能的安全状态
    Result ClearFault() override;

    /// @brief 等待 EtherCAT 周期同步信号（阻塞直至下一 PDO 交换完成，运动控制用）
    void WaitForSignal() override;

    /// @brief 从共享内存读取控制周期 [us]（mujoco/主站写入的 EcatBus::dt）
    uint32_t GetDt() const override;

    // ========== DriveInterface 接口实现 ==========
    JntArray GetPosition() override;
    JntArray GetVelocity() override;
    JntArray GetTorque() override;
    JntArray GetLoadTorque() override;
    void SetPosition(const JntArray& q) override;
    void SetVelocity(const JntArray& q_dot) override;
    void SetTorque(const JntArray& tau) override;
    void SetMode(int8_t mode) override;
    void SetEnabled() override;
    void SetDisabled() override;
    JntState GetState() override;

    // ========== Joint binding 接口 ==========
    Result SetJointBinding(const std::vector<int32_t>& model_index_to_drive_id) override;
    void ClearJointBinding() override;
    std::vector<int32_t> GetJointBinding() const override;
    int GetDriveNum() const override;
    std::vector<int32_t> GetDriveIds() const override;

    double GetJointPosition(int32_t id) override;
    double GetJointVelocity(int32_t id) override;
    double GetJointTorque(int32_t id) override;
    double GetJointLoadTorque(int32_t id) override;
    void SetJointPosition(int32_t id, double pos) override;
    void SetJointVelocity(int32_t id, double vel) override;
    void SetJointTorque(int32_t id, double tau) override;
    void SetJointMode(int32_t id, int8_t mode) override;
    void SetJointEnabled(int32_t id) override;
    void SetJointDisabled(int32_t id) override;
    std::string getJointName(int32_t id) override;

    JntState GetJointState(int32_t id) override;

    // ========== FTSensorInterface 接口实现 ==========
    Wrench GetWrench() override;

    // ========== IOInteface 接口实现 ==========
    bool GetDigitalInput(int32_t id, int32_t channel) override;
    void SetDigitalOutput(int32_t id, int32_t channel, bool value) override;
    double GetAnalogInput(int32_t id, int32_t channel) override;
    void SetAnalogOutput(int32_t id, int32_t channel, double value) override;

    // ========== 配置访问 ==========
    const HardwareConfig& getConfig() const { return config_; }

    /// @brief 根据从站 ID 查找驱动
    const Drive* findDriveById(int32_t id) const;
    /// @brief 根据关节名查找驱动
    const Drive* findDriveByName(const std::string& name) const;
    /// @brief 根据从站 ID 查找力矩传感器
    const FTSensor* findFTSensorById(int32_t id) const;
    /// @brief 根据从站 ID 查找 IO 模块
    const IO* findIOById(int32_t id) const;
    /// @brief 获取硬件类型
    HardwareType getHardwareType(int32_t slave_id) const;

    /// @brief [测试用] 从 YAML 文件加载并返回配置，不初始化 EcatConfig
    static HardwareConfig LoadConfigFromYAML(const std::string& yaml_file_path);
    static HardwareConfig LoadConfigFromNode(const YAML::Node& root);

private:
    /// @brief 从 YAML 文件加载硬件配置
    static HardwareConfig loadFromYAML(const std::string& yaml_file_path);

    /// @brief 从 YAML 节点解析单个 Drive
    static Drive parseDrive(const YAML::Node& node);
    /// @brief 从 YAML 节点解析单个 FTSensor
    static FTSensor parseFTSensor(const YAML::Node& node);
    /// @brief 从 YAML 节点解析单个 IO 模块
    static IO parseIO(const YAML::Node& node);

    /// @brief 将编码器原始值转换为物理单位
    double cntToUnit(const Drive& drive, int32_t raw_cnt) const;
    /// @brief 将物理单位转换为编码器原始值
    int32_t unitToCnt(const Drive& drive, double unit_val) const;

    /// @brief 将力矩原始值转换为物理单位
    double torqueToUnit(const Drive& drive, int16_t raw_torque) const;

    // ========== PDO 索引缓存结构体（初始化阶段一次性解析，运行阶段按索引读写）==========

    /// @brief 驱动器 PDO 索引缓存
    struct DrivePDOCache {
        // 输入 PDO 索引（从站 → 主站）
        PdoEntry<uint16_t> status_word;
        PdoEntry<int32_t> position_actual_value;
        PdoEntry<int32_t> velocity_actual_value;
        PdoEntry<int16_t> torque_actual_value;
        PdoEntry<int16_t> load_torque_value;
        PdoEntry<int32_t> secondary_position_value;
        PdoEntry<int32_t> digital_inputs;
        PdoEntry<int32_t> digital_outputs_input;  // DO 状态回读（输入 PDO 区）

        // 输出 PDO 索引（主站 → 从站）
        PdoEntry<uint16_t> control_word;
        PdoEntry<int8_t> mode_of_operation;
        PdoEntry<int32_t> target_position;
        PdoEntry<int32_t> target_velocity;
        PdoEntry<int16_t> target_torque;
        PdoEntry<int32_t> digital_outputs;
    };

    /// @brief 力传感器 PDO 索引缓存
    struct FTSensorPDOCache {
        PdoEntry<int16_t> fx;
        PdoEntry<int16_t> fy;
        PdoEntry<int16_t> fz;
        PdoEntry<int16_t> tx;
        PdoEntry<int16_t> ty;
        PdoEntry<int16_t> tz;
    };

    /// @brief IO 模块 PDO 索引缓存
    struct IOPDOCache {
        // 输入 PDO 索引
        PdoEntry<int32_t> digital_inputs;
        PdoEntry<int32_t> digital_outputs_input;  // DO 回读
        PdoEntry<int16_t> analog_inputs;

        // 输出 PDO 索引
        PdoEntry<int32_t> digital_outputs;
        PdoEntry<int16_t> analog_outputs;
    };

    /// @brief 构建 ID → 索引 O(1) 查找表（vector 预分配 + -1 哨兵）
    void buildIDLookupTables();

    /// @brief 一次性解析所有 PDO 变量名 → 缓存索引（在 EcatConfig 就绪后调用）
    void buildPDOCache();

    // ========== CiA 402 状态机（与 old_drive 完全一致）==========

    /// @brief 单驱动器状态机状态
    struct DriveSMState {
        DriveState targetState{DriveState::NA};
        bool conductStateChange{false};
        bool stateChangeSuccessful{false};  // 仅 bg 线程写 / 调用线程读，x86 上 bool 读写原子
        std::chrono::system_clock::time_point lastCwChangeTime;
        int numOfSuccessfulReadings{0};
    };

    /// @brief 获取下一个状态转换控制字（与 old_drive getNextStateTransitionControlword 一致）
    /// @param requestedDriveState 请求的目标状态
    /// @param currentDriveState  当前驱动器状态
    /// @return 对应的控制字
    static Controlword getNextStateTransitionControlword(
        const DriveState& requestedDriveState,
        const DriveState& currentDriveState);

    /// @brief 状态机单次迭代（由后台线程循环调用，对应 old_drive engageStateMachine）
    void engageStateMachine(size_t idx);

    /// @brief 启动状态切换并阻塞等待完成（对应 old_drive setDriverState + setEnabled/setDisabled）
    /// @return true=切换成功, false=超时
    bool setDriverState(size_t idx, const DriveState targetState);

    /// @brief 状态机后台工作线程（对应 old_drive DriveGuard::workingThread）
    void smWorkingThread();

    /// @brief 是否存在有效的轴绑定映射表
    bool HasJointBinding() const {
      return !model_index_to_drive_id_.empty();
    }

    /// @brief 返回建模层的关节数量（有绑定则以绑定表长度为准，否则以硬件驱动数）
    size_t ModelJointCount() const {
      return HasJointBinding() ? model_index_to_drive_id_.size() : config_.drives.size();
    }

    /// @brief 按 model index 查询对应的真实 drive id
    int32_t DriveIdByModelIndex(size_t model_index) const {
      if (HasJointBinding()) {
        return model_index_to_drive_id_[model_index];
      }
      return config_.drives[model_index].id;
    }

    /// @brief O(1) 查找：从站 ID → config_.drives 中的索引，未找到返回 -1
    int32_t getDriveIdx(int32_t id) const {
        if (id < 0 || static_cast<size_t>(id) >= drive_id_to_index_.size()) return -1;
        return drive_id_to_index_[static_cast<size_t>(id)];
    }

    /// @brief O(1) 查找：从站 ID → config_.ft_sensors 中的索引，未找到返回 -1
    int32_t getFTSensorIdx(int32_t id) const {
        if (id < 0 || static_cast<size_t>(id) >= ft_sensor_id_to_index_.size()) return -1;
        return ft_sensor_id_to_index_[static_cast<size_t>(id)];
    }

    /// @brief O(1) 查找：从站 ID → config_.ios 中的索引，未找到返回 -1
    int32_t getIOIdx(int32_t id) const {
        if (id < 0 || static_cast<size_t>(id) >= io_id_to_index_.size()) return -1;
        return io_id_to_index_[static_cast<size_t>(id)];
    }

    HardwareConfig config_;           // 硬件配置
    EcatConfig* ec_ptr_{nullptr};     // EtherCAT 配置单例指针（非拥有）

    std::vector<int32_t> model_index_to_drive_id_;  // model index → real drive id 映射表

    // O(1) ID → 索引查找表（vector 预分配 + -1 哨兵，参考 PerformanceProfiler::channel_to_index_）
    std::vector<int32_t> drive_id_to_index_;
    std::vector<int32_t> ft_sensor_id_to_index_;
    std::vector<int32_t> io_id_to_index_;

    // 名称 → 索引（string key 保留 unordered_map）
    std::unordered_map<std::string, size_t> drive_name_to_index_;

    // PDO 索引缓存（与 config_ 中对应 vector 等长，同一索引共用）
    std::vector<DrivePDOCache> drive_pdo_cache_;
    std::vector<FTSensorPDOCache> ft_sensor_pdo_cache_;
    std::vector<IOPDOCache> io_pdo_cache_;

    // CiA 402 状态机（与 drive_pdo_cache_ 并行，同一索引对应同一驱动器）
    std::vector<DriveSMState> drive_sm_state_;
    std::unique_ptr<std::thread> sm_thread_;
    std::atomic<bool> sm_thread_running_{false};
};

}  // namespace rocos
