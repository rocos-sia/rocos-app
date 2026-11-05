-- 只读状态示例：同时验证子目录与中文文件名的列表和下载。
-- 不改变使能状态，不执行运动。
local state = robot.GetState()
local count = robot.GetJointCount()

print(string.format("机器人状态：%s，关节数量：%d", state, count))
for index = 1, count do
    local position = robot.GetJointPosition(index)
    print(string.format("关节 %d：%.6f rad", index, position))
end
print("机器人状态读取完成")
