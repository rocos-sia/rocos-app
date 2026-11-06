-- Lua 基础示例：函数、表、循环和断言，不调用机器人控制接口。
local function average(values)
    assert(#values > 0, "输入数组不能为空")
    local sum = 0
    for _, value in ipairs(values) do
        sum = sum + value
    end
    return sum / #values
end

local samples = {10, 20, 30, 40, 50}
local result = average(samples)
assert(result == 30, "平均值计算错误")
print(string.format("样本数 = %d，平均值 = %.2f", #samples, result))
print(string.format("平方根示例：sqrt(81) = %.0f", math.sqrt(81)))
