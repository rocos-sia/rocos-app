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

#include "move_servo.hpp"

#include <algorithm>
#include <chrono>

namespace rocos {

// ============================================================================
// 构造函数
// ============================================================================

MoveServo::MoveServo(HardwareInterface* hw,
                     ModelInterface* model,
                     uint16_t listen_port)
    : MotionInterface(model)
    , hw_(hw)
    , listen_port_(listen_port)
{
    sockpp::initialize();

    sockpp::inet_address addr("localhost", listen_port);
    sock_.bind(addr);  // 失败不抛异常，Reset() 中检查

    if (hw_) {
        joint_count_ = std::max(0, hw_->GetDriveNum());
    }
    if (joint_count_ <= 0 && model_) {
        joint_count_ = model_->GetJointNum();
    }

    cmd_ = MotionGeneratorCommand{};
}

// ============================================================================
// 析构函数
// ============================================================================

MoveServo::~MoveServo() {
    running_.store(false);
    stopped_.store(true);
    if (thread_.joinable() &&
        thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

// ============================================================================
// ValidateParameters
// ============================================================================

Result MoveServo::ValidateParameters() const {
    if (!hw_) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (joint_count_ <= 0) {
        return Result::UnmatchedJointsNumber;
    }
    if (listen_port_ == 0) {
        return Result::IllegalParameter;
    }
    return Result::NoError;
}

// ============================================================================
// Reset —— 初始化状态并启动 UDP 接收线程
// ============================================================================

Result MoveServo::Reset() {
    Result rc = ValidateParameters();
    if (rc != Result::NoError) {
        return rc;
    }

    if (!sock_.is_open()) {
        return Result::ServoBindFail;
    }

    // 刷新 joint_count（硬件可能在构造后才就绪）
    if (hw_ && hw_->GetDriveNum() > 0) {
        joint_count_ = hw_->GetDriveNum();
    }
    if (joint_count_ <= 0 && model_) {
        joint_count_ = model_->GetJointNum();
    }
    if (joint_count_ <= 0) {
        return Result::UnmatchedJointsNumber;
    }

    // 重置共享缓冲区
    {
        std::lock_guard<std::mutex> lock(mtx_);
        cmd_ = MotionGeneratorCommand{};
        has_new_cmd_ = false;
    }
    stopped_.store(false);
    running_.store(true);
    message_id_counter_ = 0;

    // 重置滑动窗口
    success_window_.fill(false);
    success_index_ = 0;
    success_count_ = 0;

    if (current_mode_ == MotionMode::kNone) {
        current_mode_ = MotionMode::kJointPosition;
    }

    // join 旧线程（Reset 复用场景）
    if (thread_.joinable()) {
        running_.store(false);
        thread_.join();
        running_.store(true);
    }

    // 启动 UDP 接收线程
    try {
        thread_ = std::thread(&MoveServo::udpReceiveLoop, this);
    } catch (const std::system_error&) {
        return Result::ServoThreadStartFailed;
    }

    return Result::NoError;
}

// ============================================================================
// Pause / Resume / Stop
// ============================================================================

Result MoveServo::Pause() {
    return Result::FunctionNotSupported;
}

Result MoveServo::Resume() {
    return Result::FunctionNotSupported;
}

Result MoveServo::Stop() {
    stopped_.store(true);
    running_.store(false);

    if (thread_.joinable() &&
        thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }

    return Result::NoError;
}


// ============================================================================
// GenerateRef —— 将最新收到的指令转为 Reference
// ============================================================================

Result MoveServo::GenerateRef(Reference& ref_out) {

    MotionGeneratorCommand local_cmd;
    bool                   has_cmd = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        has_cmd = has_new_cmd_;
        if (has_cmd) {
            local_cmd = cmd_;
        }
    }

    // 尚未收到任何指令：必须回填当前位置。Reference 默认构造持有空 JntArray，
    // 直接返回会让下游 Controller 拿到零维参考而失去位置锁定。
    if (!has_cmd) {
        Result rc = fillCurrentReference(ref_out);
        if (static_cast<int>(rc) < 0) {
            return rc;
        }
    } else {
        switch (current_mode_) {
        case MotionMode::kJointPosition: {
            JntArray q_ref(static_cast<unsigned int>(joint_count_));
            for (int i = 0; i < joint_count_; ++i) {
                q_ref(i) = local_cmd.q_c[static_cast<size_t>(i)];
            }

            ref_out = std::move(q_ref);
            break;
        }
        case MotionMode::kCartesianPosition: {
            Frame f = matrixToFrame(local_cmd.tcp_c);
            ref_out = std::move(f);
            break;
        }
        case MotionMode::kNone:
        default: {
            Result rc = fillCurrentReference(ref_out);
            if (static_cast<int>(rc) < 0) {
                return rc;
            }
            break;
        }
        }
    }

    if (stopped_.load(std::memory_order_relaxed)) {
        return Result::PlanFinished;
    }

    return Result::NoError;
}

// ============================================================================
// fillCurrentReference —— 按当前模式把机器人实际位置填入 Reference
// ============================================================================

Result MoveServo::fillCurrentReference(Reference& ref_out) {
    if (hw_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    if (joint_count_ <= 0) {
        return Result::UnmatchedJointsNumber;
    }

    JntArray q_now = hw_->GetPosition();
    if (static_cast<int>(q_now.rows()) < joint_count_) {
        return Result::UnmatchedJointsNumber;
    }

    if (current_mode_ == MotionMode::kCartesianPosition) {
        if (model_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }
        Frame  f;
        Result rc = model_->ForwardKinematics(q_now, f);
        if (static_cast<int>(rc) < 0) {
            return rc;
        }
        ref_out = std::move(f);
        return Result::NoError;
    }

    // kJointPosition / kNone：截断到实际轴数，保持与指令分支一致的维度
    JntArray q_ref(static_cast<unsigned int>(joint_count_));
    for (int i = 0; i < joint_count_; ++i) {
        auto ui  = static_cast<unsigned int>(i);
        q_ref(ui) = q_now(ui);
    }
    ref_out = std::move(q_ref);
    return Result::NoError;
}

// ============================================================================
// buildRobotState —— 从硬件读取状态并构建 RobotState（UDP 线程内调用）
// ============================================================================

RobotState MoveServo::buildRobotState() {
    RobotState state{};

    if (!hw_) {
        return state;
    }

    auto q    = hw_->GetPosition();
    auto dq   = hw_->GetVelocity();
    auto tau  = hw_->GetTorque();
    auto load = hw_->GetLoadTorque();

    state.message_id = ++message_id_counter_;

    int n = std::min(joint_count_, static_cast<int>(MAX_DOF));
    for (int i = 0; i < n; ++i) {
        auto ui = static_cast<unsigned int>(i);
        state.q[i]    = q(ui);
        state.dq[i]   = dq(ui);
        state.tau[i]  = tau(ui);
        state.load[i] = load(ui);
    }

    // 正运动学计算法兰位姿
    if (model_) {
        Frame flange;
        model_->ForwardKinematics(q, flange);
        frameToMatrix(flange, state.flange_to_base);
    }
    state.tcp_to_base = state.flange_to_base;  // TODO:  目前无法获取tcp pose by think

    state.errors = {};
    state.mode   = current_mode_;
    state.control_command_success_rate = computeSuccessRate();

    return state;
}

// ============================================================================
// UDP 收发线程 —— 一次 loop：接收 MotionGeneratorCommand → 回复 RobotState
// ============================================================================

void MoveServo::udpReceiveLoop() {
    // clone() 创建独立 socket 句柄，避免与主 socket 竞争
    auto sock = sock_.clone();
    sock.set_non_blocking(true);

    char                       buf[sizeof(MotionGeneratorCommand)];
    MotionGeneratorCommand     best_cmd{};   // 保存 message_id 最大的完整指令
    sockpp::inet_address       addr;
    bool                       received = false;

    while (running_.load(std::memory_order_relaxed)) {
        // ================================================================
        // Phase 1: 非阻塞排空积压包，保留 message_id 最大的完整指令
        // ================================================================
        while (running_.load(std::memory_order_relaxed)) {
            ssize_t n = sock.recv_from(buf, sizeof(buf), &addr);
            if (n <= 0) {
                break;  // 缓冲区已空
            }
            if (n != sizeof(MotionGeneratorCommand)) {
                continue;  // 畸形包，丢弃
            }

            auto* cmd = reinterpret_cast<MotionGeneratorCommand*>(buf);
            if (!received || cmd->message_id > best_cmd.message_id) {
                // 保存完整指令数据，避免 buf 被后续 recv_from 覆盖后丢失
                best_cmd = *cmd;
                received = true;
            }
        }

        // ================================================================
        // 收到有效指令 → 回复 RobotState；仅 message_id > 0 时更新运动指令
        // message_id == 0 为纯查询包，不写入共享缓冲区
        // ================================================================
        if (received) {
            if (best_cmd.message_id > 0) {
                std::lock_guard<std::mutex> lock(mtx_);
                cmd_ = best_cmd;
                has_new_cmd_ = true;
            }

            RobotState state = buildRobotState();
            ssize_t sent = sock.send_to(&state, sizeof(RobotState), addr);
            recordSuccess(sent == sizeof(RobotState));

            received = false;
        }

        // ================================================================
        // Phase 2: 无包时短暂休眠，避免忙等
        // ================================================================
        if (!received && running_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}

// ============================================================================
// 矩阵转换辅助
// ============================================================================

Frame MoveServo::matrixToFrame(const std::array<double, 16>& mat) {
    // mat 是 row-major 4x4 齐次变换矩阵
    // rocos::Rotation 的 9 元素构造使用 row-major 顺序
    // mat[i*4 + j] 对应第 i 行、第 j 列
    rocos::Rotation R(
        mat[0], mat[1], mat[2],    // 第 0 行: (0,0), (0,1), (0,2)
        mat[4], mat[5], mat[6],    // 第 1 行: (1,0), (1,1), (1,2)
        mat[8], mat[9], mat[10]    // 第 2 行: (2,0), (2,1), (2,2)
    );
    rocos::Vector p(mat[3], mat[7], mat[11]);  // 平移: (0,3), (1,3), (2,3)
    return {R, p};
}

void MoveServo::frameToMatrix(const Frame& f, std::array<double, 16>& mat) {
    mat.fill(0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat[static_cast<size_t>(i * 4 + j)] =
                f.M(static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        }
    }
    mat[3]  = f.p.x();   // row 0, col 3
    mat[7]  = f.p.y();   // row 1, col 3
    mat[11] = f.p.z();   // row 2, col 3
    mat[15] = 1.0;       // row 3, col 3
}

// ============================================================================
// 通信质量统计
// ============================================================================

void MoveServo::recordSuccess(bool ok) {
    if (success_window_[static_cast<size_t>(success_index_)]) {
        --success_count_;
    }
    success_window_[static_cast<size_t>(success_index_)] = ok;
    if (ok) {
        ++success_count_;
    }
    success_index_ = (success_index_ + 1) % kSuccessWindowSize;
}

double MoveServo::computeSuccessRate() const {
    return static_cast<double>(success_count_) /
           static_cast<double>(kSuccessWindowSize);
}

}  // namespace rocos
