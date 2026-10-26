# Robotiq 2F-85 工具模型

来源：PickNik Robotics 的 ROS 2 Robotiq 模型库（非 Robotiq 官方维护）。

- 仓库：https://github.com/PickNikRobotics/ros2_robotiq_gripper
- 固定来源提交：a74d007d8f2f06dc6a503ad21038ba869d4999a6
- 原始 URDF 宏：robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro
- STL：robotiq_description/meshes/collision/2f_85/（引用的 9 个零件）
- 许可证：BSD-3-Clause，原始全文见 LICENSE。
- 整理日期：2026-10-23。

## 本包调整

将上游 Xacro 展开为不依赖 ROS 的 gripper.urdf；保留关节原点、轴、限位、mimic 和惯性参数，去除控制插件。新增位于原始夹爪基座的 tool_base 根节点，不包含机器人或法兰转接板。

当前 ROCOS 工具包支持 STL，因此将原始 DAE 外观资源替换为上游随附的碰撞 STL，并添加黑灰纯色材质。这是简化外观的真实夹爪模型。STL 坐标直接使用米，无额外缩放。零位为张开姿态；本包仅用于模型展示，不提供夹爪驱动或开合控制。

## 导入与设置

在 Viz 的「IO 与设备 → 末端工具 → 工具包」导入同目录旁的 robotiq_2f85.zip。也可将完整 robotiq_2f85 文件夹复制到 App 工具库后扫描。

安装矩阵 mount 默认为单位阵；必须根据实际法兰/转接板到夹爪基座的关系设置。TCP 尚未标定，初值为单位阵，与夹爪基座重合，不代表指尖或夹持中心。请在工具管理中填写实际 TCP，或使用六点法标定后保存。这里没有虚构指尖标定值。

导入只注册工具，选为当前工具后才在机器人三维视图中显示。有效 TCP 为法兰位姿 × 安装矩阵 × 局部 TCP 矩阵。
