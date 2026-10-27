// Copyright 2026, Yang Luo
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "shared_memory_config.hpp"

namespace rocos {

// 初始化时缓存索引；运行时使用公共库的 O(1) 检查和 memcpy，允许非自然对齐 PDO。
// 此对象不拥有连接，连接和静态 PDO 映射必须在使用期间保持不变。
template <typename T>
class PdoEntry {
public:
    static PdoEntry input(SharedMemoryConfig& config, int slave, const std::string& name) {
        return bind(config, slave, name, false);
    }

    static PdoEntry output(SharedMemoryConfig& config, int slave, const std::string& name) {
        return bind(config, slave, name, true);
    }

    explicit operator bool() const { return config_ != nullptr; }

    T read() const {
        if (!config_) throw std::logic_error("Unbound PDO entry");
        return output_ ? config_->getSlaveOutputVarValue<T>(slave_, variable_)
                       : config_->getSlaveInputVarValue<T>(slave_, variable_);
    }

    void write(T value) const {
        if (!config_ || !output_) throw std::logic_error("PDO entry is not an output");
        config_->setSlaveOutputVarValue<T>(slave_, variable_, value);
    }

private:
    static PdoEntry bind(SharedMemoryConfig& config, int slave, const std::string& name, bool output) {
        PdoEntry entry;
        const int variable = output ? config.findSlaveOutputVarIdByName(slave, name)
                                    : config.findSlaveInputVarIdByName(slave, name);
        if (variable < 0) return entry;
        entry.config_ = &config;
        entry.slave_ = slave;
        entry.variable_ = variable;
        entry.output_ = output;
        // 提前验证类型宽度和映射边界；错误在硬件初始化阶段报告。
        (void)entry.read();
        return entry;
    }

    SharedMemoryConfig* config_{nullptr};
    int slave_{-1};
    int variable_{-1};
    bool output_{false};
};

}  // namespace rocos
