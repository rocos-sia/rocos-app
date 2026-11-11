#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace rocos {
struct ServoSession {
    std::array<uint8_t, 16> id{};
    uint32_t client_ipv4{0}; // 主机字节序，0 表示由首个有效包确定来源。
    std::atomic<bool> valid{true};
    std::atomic<int64_t> expires_ns{INT64_MAX};
    static int64_t Now() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
    bool Active() const { return valid.load() && Now() < expires_ns.load(); }
    std::string Hex() const {
        constexpr char digits[] = "0123456789abcdef";
        std::string s;
        for (auto byte : id) {
            s += digits[byte >> 4];
            s += digits[byte & 15];
        }
        return s;
    }
};
}  // namespace rocos
