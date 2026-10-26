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

#include <unordered_map>
#include <string>
#include <thread>
#include <vector>
#include <functional>
#include <atomic>

#include <json.hpp>

#include "types.hpp"
#include "result.hpp"
#include "servo_type.hpp"
#include "logger.hpp"
#include "frame_registry.hpp"


// #include "dh_params_loader.hpp"
#include "model_interface.hpp"
#include "motion_interface.hpp"
#include "hardware_interface.hpp"
#include "hardware.hpp"

#include "executor.hpp"
#include "joint_binding.hpp"

#include <random>

namespace rocos {

class Robot {

 public:
  enum class JogFrame {
    BASE,
    FLANGE,
    TOOL,
    OBJECT
  };

  struct JointInfo {
    int id{-1};
    std::string name;
    double cnt_per_unit{1.0};
    double torque_per_unit{1.0};
    double ratio{1.0};
    std::string unit_name{"rad"};
    int32_t zero_offset{0};
  };

  /// @brief 机器人状态快照，一次性打包所有可视化页面需要的状态信息
  struct RobotStateSnapshot {
    // --- FSM 状态 ---
    std::string state_string;
    bool is_enabled{false};
    bool is_running{false};
    bool control_active{false};
    bool motion_busy{false};
    std::string work_mode;

    // --- 单轴状态 ---
    struct JointState {
      int id{-1};
      std::string name;
      double position{0.0};
      double velocity{0.0};
      double torque{0.0};
      double load_torque{0.0};
      int status{0};
    };
    std::vector<JointState> joints;

    // --- 笛卡尔位姿 ---
    Frame flange;
    Frame tcp;  // 基座到当前 TCP，与 flange 使用同一关节采样和注册表快照
    bool tcp_valid{false};
    std::string active_tool_frame_name;
    std::optional<ToolPackage> active_tool_package;
    std::string active_object_frame_name;
    Frame active_tool_frame;
    Frame active_object_frame;

    // --- 硬件摘要 ---
    int joint_num{0};
    int hardware_state{0};

    // --- 元信息 ---
    double timestamp{0.0};  // 快照生成时刻 (epoch seconds)
  };

  struct Config {
    std::string hardware_config{"hardware_talon_config.yaml"};
    int ecat_id{0};
    std::string urdf{"robot.urdf"};
    std::string base_link{"base_link"};
    std::string tip_link{"link_7"};
    std::string joint_binding{"joint_binding.yaml"};
    std::string payload{"payload.yaml"};
    std::string frames{"frames.yaml"};
    std::string tool_library{"../tools"};
    std::string id{"default"};
    bool composed_configuration{false};
    std::optional<HardwareConfig> hardware;
    std::optional<ToolPackage> tool;
    std::optional<std::pair<double, Vector>> initial_payload;
    YAML::Node payload_metadata;
    bool payload_overridden{false};
    int master_cycle_us{0};
    std::string master_state{"op"};
  };

  explicit Robot();
  explicit Robot(const Config& config);

  ~Robot();

  [[nodiscard]] std::string GetStateString() const;

  /// @brief 获取机器人状态快照，供前端可视化页面一次性拉取全部状态
  [[nodiscard]] RobotStateSnapshot GetRobotStateSnapshot() const;

  // //! \brief 启动机器人运动，使机器人状态机进入 Running 状态
  // Result Start();

  //! \brief 启动机器人运动，使机器人状态机进入 Running 状态
  Result Start();
  //! \brief 机器人上使能请求
  Result SetEnabled();
  //! \brief 机器人下使能请求
  Result SetDisabled();
  //! \brief 清除报警/错误状态，成功后状态机回到 STOPPED
  Result ResetFault();

  /// @brief 切换工作模式（控制器）
  /// @param mode 模式字符串: "position" | "jnt_imp" | "jnt_admit_teach" | "cart_imp" | "zero_force_drag" | "ee_admit_teach"
  /// @note 仅允许在 IDLE 或 STOPPED 状态下调用
  Result SetWorkMode(const std::string& mode);

  /// @brief 设置已知负载参数（质量 + 质心），供零力拖动控制器跳过辨识直接使用
  /// @param mass 负载质量 [kg]，必须 >= 0（0 表示无负载）
  /// @param com  负载质心（末端坐标系下）[m]
  /// @note 切换到 "zero_force_drag" 模式前调用；若未设置则进入辨识流程
  Result SetLoadParameters(double mass, const Vector& com);

  /// @brief 从 YAML 文件加载已知负载参数（质量 + 质心）
  /// @param path 配置文件路径，为空时使用启动配置指定的负载文件
  Result LoadLoadParameters(const std::string& path = "");

  /// @brief 将当前负载参数保存到 YAML 文件
  /// @param path 配置文件路径，为空时使用启动配置指定的负载文件
  Result SaveLoadParameters(const std::string& path = "") const;

  [[nodiscard]] const std::string& GetUrdfPath() const { return urdf_path_; }
  [[nodiscard]] const std::string& GetLoadParametersPath() const { return load_params_path_; }

  /// @brief 查询当前是否已持有有效的负载参数（预置或辨识得到）
  [[nodiscard]] bool HasLoadParameters() const { return load_params_valid_; }
  /// @brief 查询当前负载质量 [kg]
  [[nodiscard]] double GetLoadMass() const { return load_mass_; }
  /// @brief 查询当前负载质心（末端坐标系下）[m]
  [[nodiscard]] Vector GetLoadCom() const { return load_com_; }


  //! \brief 获取机器人当前是否上使能状态
  [[nodiscard]] bool IsEnabled() const;
  //！ \brief 获取机器人当前是否上使能状态
  [[nodiscard]] bool IsDisabled() const;
  //！ \brief 获取机器人当前是否正在运动
  [[nodiscard]] bool IsRunning() const;

  /// @brief 是否处于需要控制循环的状态（RUNNING/PAUSING/PAUSED/RESUMING/STOPPING）
  [[nodiscard]] bool IsControlActive() const;
  /// @brief 是否处于运动相关的忙碌状态（STARTING/RUNNING/PAUSING/PAUSED/RESUMING/STOPPING）
  [[nodiscard]] bool IsMotionBusy() const;
  //! \brief 等待当前运动结束，先延时20ms，再轮询状态机直到完成或错误
  Result WaitMove();

  [[nodiscard]] std::vector<JointInfo> GetJointInfo() const;

  // TODO(HTTP API): 下面这些接口目前还没有收口到 Robot 层，HTTP 不应该长期直接解析文件或访问底层指针。
  // - GetRobotModelInfo(): 对应 GET /api/robot/model，返回 URDF model name、links、joint origin、
  //   joint axis、visual origin、mesh path 等前端建模数据。
  // - GetRobotModelMesh(path): 对应 GET /api/robot/model/mesh，由 Robot 根据当前 URDF 路径解析并读取 mesh。
  // ✓ GetUrdfPath(): 返回启动配置指定的模型文件。
  // - GetModelBaseLink()/GetModelTipLink(): 暴露当前模型的基座和末端 link。
  // ✓ GetRobotStateSnapshot(): 对应 GET /api/robot/state，已实现。
  // - GetMotionStatus()/GetCurrentTaskInfo(): 对应 GET /api/robot/move_status，封装当前任务状态而不是 HTTP 自己拼。
  //
  /// @brief 获取当前阻抗控制参数（关节空间 + 笛卡尔空间）
  /// @return JSON 含 work_mode、joint_space、cartesian_space 三个字段；
  ///         仅当前控制器类型对应的空间有效，另一个空间标记 valid:false
  [[nodiscard]] nlohmann::json GetImpedanceParams() const;

  /// @brief 控制循环主函数，每周期调用 (1000Hz)
  void RunCycle();

  Result MoveJogging(const JogVec& direction, double speed,
                       double timeout = 0.1, double dir_threshold = 0.99);

  Result MoveJogging(const Twist& direction,
                     JogFrame frame,
                     double speed,
                     double timeout = 0.1,
                     double dir_threshold = 0.99);

  Result MoveNullJogging(const JntArray& intent_direction, double speed,
                           double timeout = 0.1, double dir_threshold = 0.99);

  Result MoveSvdJogging(const std::vector<double>& dim_speeds,
                          double timeout = 0.1, double dir_threshold = 0.99);

  Result MoveServoing(uint16_t port = 8081);

  /// @brief 设置伺服运动模式（joint / cartesian），可在启动前或运行中调用
  Result SetServoMode(const std::string& mode_str);

  Result MoveJ(const JntArray& q_goal,
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  Result MoveL(const Frame& pose_goal,
               const std::string& tool_name = "",
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  Result MoveJ_IK(const Frame& pose_goal,
                  double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  Result MoveL_FK(const JntArray& q_goal,
                  const std::string& tool_name = "",
                  double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  // 命名坐标系管理：上位机通过 name 列表展示可选工具系/工件系，
  // 再按 name 查询、设置、新增或删除对应的 Frame。
  std::vector<std::string> GetToolFrameNames() const;
  std::vector<std::string> GetObjectFrameNames() const;
  bool HasToolFrame(const std::string& name) const;
  bool HasObjectFrame(const std::string& name) const;
  bool IsValidFrameName(const std::string& name) const;
  Result GetToolFrame(const std::string& name, Frame& frame) const;
  /// 基座到 TCP 的实时位姿；空名称使用当前工具，无活动工具时为法兰。
  /// 指定工具不存在或 FK 失败时返回错误，不改变输出或活动工具。
  Result GetTcpPose(Frame& tcp, const std::string& tool_name = "") const;
  Result GetObjectFrame(const std::string& name, Frame& frame) const;
  Frame GetToolFrame(const std::string& name) const;
  Frame GetObjectFrame(const std::string& name) const;
  Result SetToolFrame(const std::string& name, const Frame& T_tool);
  Result SetToolFrame(const std::string& name, const Frame& frame, bool persist);
  Result SetObjectFrame(const std::string& name, const Frame& T_object);
  Result AddToolFrame(const std::string& name, const Frame& T_tool);
  Result AddObjectFrame(const std::string& name, const Frame& T_object);
  Result RemoveToolFrame(const std::string& name);
  Result RemoveToolFrame(const std::string& name, bool persist);
  Result RemoveObjectFrame(const std::string& name);
  Result SetActiveToolFrame(const std::string& name);
  Result SetActiveToolFrame(const std::string& name, bool persist);
  Result SetActiveObjectFrame(const std::string& name);
  std::string GetActiveToolFrameName() const;
  std::string GetActiveObjectFrameName() const;
  Result GetActiveToolFrame(Frame& frame) const;
  Result GetActiveObjectFrame(Frame& frame) const;
  Frame GetActiveToolFrame() const;
  Frame GetActiveObjectFrame() const;
  Result LoadFrames(const std::string& path = "");
  Result SaveFrames(const std::string& path = "") const;
  const std::string& GetFramesPath() const { return frames_.path(); }
  Result CaptureCalibrationPose(Frame& flange, std::vector<double>& joints, double& timestamp) const;

  // MoveC 圆心+角度
  Result MoveC(const Frame& pose_start, const Frame& center_frame, double theta,
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  // MoveC 三点圆弧（重载：第三个参数是 Frame）
  Result MoveC(const Frame& pose_start, const Frame& pose_via,
               const Frame& pose_goal,
               double v_limit = 1.0, double a_limit = 2.0, double j_limit = 10.0);

  //! \brief 暂停机器人运动，使机器人状态机进入 Pause 状态
  Result PauseMotion();
  //! \brief 停止机器人运动，使机器人状态机进入 Stopped 状态
  Result StopMotion();
  //! \brief 继续机器人运动，使机器人状态机进入 Running 状态
  Result ResumeMotion();





























  void waitControlCycle();

  void setEnabled();

  void setDisabled();

  // inline void setJointMode(int id, ModeOfOperation mode) { }

  inline int getJointNum() const {
    return model ? model->GetJointNum() : 0;
  }

  inline std::string getJointName(int id) const {
    return hardware ? hardware->getJointName(id) : "";
  }

  inline int getJointStatus(int id) const { return 0; }

  ///////////////////用户单位信息///////////////////////
  inline double getJointPosition(int id) const {
    if (!hardware || id < 0) return 0.0;
    auto q = hardware->GetPosition();
    return static_cast<unsigned int>(id) < q.rows() ? q(id) : 0.0;
  }

  inline double getJointVelocity(int id) const {
    if (!hardware || id < 0) return 0.0;
    auto q_dot = hardware->GetVelocity();
    return static_cast<unsigned int>(id) < q_dot.rows() ? q_dot(id) : 0.0;
  }

  inline double getJointTorque(int id) const {
    if (!hardware || id < 0) return 0.0;
    auto tau = hardware->GetTorque();
    return static_cast<unsigned int>(id) < tau.rows() ? tau(id) : 0.0;
  }

  inline double getJointLoadTorque(int id) const {
    if (!hardware || id < 0) return 0.0;
    auto tau_load = hardware->GetLoadTorque();
    return static_cast<unsigned int>(id) < tau_load.rows() ? tau_load(id) : 0.0;
  }
  // 获取滤波后的数据
  inline double getJointTorqueFilter(int id) { return getJointLoadTorque(id); }
  inline double getJointSecondaryPositionInCnt(int id) { return 0.0; }

  inline void setJointPosition(int id, double pos) {
    if (!hardware || id < 0) return;
    auto q = hardware->GetPosition();
    if (static_cast<unsigned int>(id) >= q.rows()) return;
    q(id) = pos;
    hardware->SetPosition(q);
  }

  inline void setJointVelocity(int id, double vel) {
    if (!hardware || id < 0) return;
    auto q_dot = hardware->GetVelocity();
    if (static_cast<unsigned int>(id) >= q_dot.rows()) return;
    q_dot(id) = vel;
    hardware->SetVelocity(q_dot);
  }

  inline void setJointTorque(int id, double tor) {
    if (!hardware || id < 0) return;
    auto tau = hardware->GetTorque();
    if (static_cast<unsigned int>(id) >= tau.rows()) return;
    tau(id) = tor;
    hardware->SetTorque(tau);
  }

  inline Frame getFlange() const {
    Frame flange;
    ReadFlangePose(flange);
    return flange;
  }



 public:
  std::unique_ptr<ModelInterface> model {nullptr};  //TODO: 机器人里可以保存子类指针，因为机器人会用到很多子类的功能，但是其他的模块接收接口指针即可
  std::unique_ptr<HardwareInterface> hardware {nullptr};  //TODO: 同理，机器人里直接保存子类指针
  std::unique_ptr<MotionInterface> motion {nullptr}; // movej, movel, movec
  std::unique_ptr<ControllerInterface> controller {nullptr}; // position, CartAdmit, JntAdmit, JntImp, CartImp

  std::unique_ptr<Executor> executor {nullptr}; //执行器

 public:

  std::unique_lock<std::recursive_mutex> LockToolState() const;
  bool CanChangeToolState() const;
  Result ModifyFrames(const std::function<Result()>& change) const;
  friend class RobotHttpServer;  // 允许 Server 直接访问 Robot 的私有/保护成员
                                 // TODO: ================================

 private:

  Result ComputeFlangePose(const JntArray& joints, Frame& flange) const;
  Result ReadFlangePose(Frame& flange) const;
  // void controlLoop();                        // 控制线程函数
  void startControlThread();
  void stopControlThread();
  void joinControlThreadIfJoinable();
  void jointBinding();
  std::thread control_thread_;               // 控制线程句柄
  mutable std::mutex mtx_;

  // 命名坐标系注册表：key 是上位机展示和选择的坐标系 name，value 是对应位姿。
  FrameRegistry frames_;

  std::string work_mode_ {"position"};  // 当前工作模式字符串: "position" | "jnt_imp" | "jnt_admit_teach" | "cart_imp" | "zero_force_drag" | "ee_admit_teach"

  // 已知负载参数（供零力拖动控制器跳过辨识），由 SetLoadParameters() 设置
  double load_mass_{0.0};
  Vector load_com_;
  bool load_params_valid_{false};
  std::string load_params_path_{"payload.yaml"};  // 负载参数持久化文件
  YAML::Node load_params_metadata_;
  bool load_params_overridden_{false};
  std::string urdf_path_{"robot.urdf"};

  MotionMode servo_mode_{MotionMode::kJointPosition};  // 伺服运动模式: kJointPosition | kCartesianPosition

  //// Joint binding
  std::unique_ptr<JointBinding> joint_binding_;
  std::string joint_binding_path_{"joint_binding.yaml"};

  std::shared_ptr<RtLogProducer> rt_log_producer_{
      LogManager::instance().createRtProducer(),
      [](RtLogProducer* producer) { LogManager::instance().releaseRtProducer(producer); }};
  Logger::logger_ptr log_ptr_ = nullptr;

  // FSM 初始状态是 ERROR_STATE，构造时会进入一次；用它区分“初始进入”和“真正的错误进入”
  bool fsm_error_entered_once_{false};

  //// 机器人状态机封装
  struct Impl;
  std::unique_ptr<Impl> impl_;


  std::function<Result()> data_ready_callback_ = nullptr;

  double dt_=0.001;
  //////////FSM Related function (INTERNAL) ///////////////
 public:
  void on_fsm_resetting();

  void on_fsm_enabling();

  void on_fsm_disabling();

  void on_fsm_starting();

  void on_fsm_running();

  void on_fsm_stopping();

  void on_fsm_pausing();

  void on_fsm_resuming();

  void on_fsm_identify();

  void on_fsm_servoing();

  void on_fsm_error();

  void on_fsm_stopped();


  //////////测试用///////////
  /// 使用 Bernoulli 分布随机生成一个 bool 值（概率各 50%）
  inline bool randomBool() noexcept {
    static std::mt19937 rng_{std::random_device{}()};
    static std::bernoulli_distribution dist_(0.5);
    return dist_(rng_);
  }



};



}  // namespace rocos
