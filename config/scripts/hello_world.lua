-- 入门示例：输出日志，不使能机器人，不执行运动。
print("Hello, ROCOS!")

local name = "Lua"
local message = string.format("%s 脚本已从控制器程序库加载", name)
print(message)
assert(#message > 0, "消息不应为空")
print("入门示例完成")
