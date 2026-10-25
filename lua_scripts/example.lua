local home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
local pos1 = {0.0, 0.78, 0.0, 1.57, 0.0, -0.78, 0.0}

robot.SetEnabled()

-- 关节运动到 pos1
robot.MoveJ(pos1, 1.0, 2.0, 10.0)

-- 获取当前法兰位姿（GetFlange 返回 {x, y, z, qx, qy, qz, qw}）
local pose1 = robot.GetFlange()

-- 在 pose1 基础上沿 z 轴向上移动 0.1 m（保持姿态不变）
local pose2 = {
    x  = pose1.x,
    y  = pose1.y,
    z  = pose1.z + 0.1,
    qx = pose1.qx,
    qy = pose1.qy,
    qz = pose1.qz,
    qw = pose1.qw,
}
robot.MoveL(pose2, "", 0.1, 0.5, 5.0)

-- 返回 Home 位置
robot.MoveJ(home, 1.0, 2.0, 10.0)

robot.SetDisabled()
