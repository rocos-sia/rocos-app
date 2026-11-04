#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace rocos {

struct IdentifiedJointImpedanceConfig {
    std::string urdf;
    std::string hardware;  // 独立硬件 YAML，路径相对于控制配置。
    std::vector<std::string> jointNames;
    std::vector<int> hardwareIds;
    Eigen::VectorXd kp, kd;
    Eigen::VectorXd lower, upper, velocity;
    Eigen::VectorXd velocityLimit;  // YAML 独立保护阈值，rad/s，不高于 URDF 速度限制。
    Eigen::VectorXd torqueLimit, torqueRateLimit;
    double frictionScale = 1.0;  // 摩擦补偿比例 [0,1]；旧配置未指定时保持全量补偿。
    bool needsTemperature = false;
};

// 启动时覆盖控制增益与摩擦比例；模型、单位及保护限制仍从 YAML 加载。
struct IdentifiedJointImpedanceTuning {
    Eigen::VectorXd kp, kd;
    double frictionScale = 1.0;
};

// 向量统一按 config().jointNames 排列，单位为 rad、rad/s、Nm 和 °C。
// 每个实例由一个控制线程使用，qDesired 也应在该线程更新。
class IdentifiedJointImpedanceController {
   public:
    explicit IdentifiedJointImpedanceController(const std::string &yamlPath);
    IdentifiedJointImpedanceController(const std::string &yamlPath,
                                       const IdentifiedJointImpedanceTuning &tuning);
    ~IdentifiedJointImpedanceController();
    const IdentifiedJointImpedanceConfig &config() const;

    // 最近一次成功计算的补偿分量；frictionTorque 已乘 frictionScale。
    const Eigen::VectorXd &rigidTorque() const;
    const Eigen::VectorXd &frictionTorque() const;

    // Y(q,dq,0) P beta + frictionScale * friction + Kp(qDesired-q) - Kd*dq.
    // 返回未限幅力矩，由调用方统一限制力矩幅值和变化率。
    // 仅当全部温度斜率为零时，temperature 才允许为空。
    // 返回值引用内部缓冲，下次调用会覆盖；模型及缓冲只在初始化时创建。
    const Eigen::VectorXd &computeTorque(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                         const Eigen::VectorXd &qDesired,
                                         const Eigen::VectorXd &temperature = Eigen::VectorXd());

    // 按周期选择是否补偿摩擦；不修改 YAML 系数，原有接口仍默认开启。
    const Eigen::VectorXd &computeTorque(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                         const Eigen::VectorXd &qDesired,
                                         const Eigen::VectorXd &temperature, bool frictionEnabled);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rocos
