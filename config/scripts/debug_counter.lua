-- 调试示例：可在循环中设置断点，观察暂停、恢复和单步执行。
-- 仅计数和等待，不使能机器人，不执行运动。总时长约 10 秒。
local total = 0
local rounds = 20

for index = 1, rounds do
    total = total + index
    print(string.format("第 %d / %d 轮，累计值 = %d", index, rounds, total))
    robot.Sleep(500)
end

assert(total == rounds * (rounds + 1) / 2, "累计结果错误")
print(string.format("调试示例完成，最终累计值 = %d", total))
