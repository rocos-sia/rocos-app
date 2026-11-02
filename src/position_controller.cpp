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

#include "position_controller.hpp"
#include "position_command_validation.hpp"
#include "log_manager.hpp"

#include <cmath>
#include <variant>

namespace rocos {

PositionController::~PositionController() {
    if (hardware_ != nullptr) {
        hardware_->WaitForSignal();
        static_cast<void>(UpdateCmd(hardware_->GetPosition()));
    }
}

Result PositionController::SetReady() {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_->WaitForSignal();
    return UpdateCmd(hardware_->GetPosition());
}

bool PositionController::Reset() {
    mode_set_ = false;
    has_last_cmd_ = false;  // 清除上一周期指令基线，下一次 UpdateCmd 重新建立
    return true;
}

Result PositionController::SetHardware(HardwareInterface* hardware) {
    if (hardware == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    hardware_ = hardware;
    return Result::NoError;
}

Result PositionController::SetModel(ModelInterface* model) {
    if (model == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }
    model_ = model;
    return Result::NoError;
}

Result PositionController::GenerateCmd(const Reference& ref_in, JntArray& q_cmd) {
    // 分支1: 关节空间参考值 — 直接透传
    if (auto* q_ref = std::get_if<JntArray>(&ref_in)) {
        if (q_ref->rows() == 0) {
            return Result::MoveInput;
        }

        for (unsigned int i = 0; i < q_ref->rows(); ++i) {
            if (!std::isfinite((*q_ref)(i))) {
                return Result::ParameterNanOrInf;
            }
        }

        // 可选校验: 维度与硬件关节数一致
        if (hardware_ != nullptr) {
            JntArray q_curr = hardware_->GetPosition();
            if (q_curr.rows() != q_ref->rows()) {
                return Result::UnmatchedJointsNumber;
            }
        }

        q_cmd = *q_ref;
        return Result::NoError;
    }

    // 分支2: 笛卡尔空间参考值 — 通过逆运动学转为关节角
    if (auto* frame_ref = std::get_if<Frame>(&ref_in)) {
        if (hardware_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }
        if (model_ == nullptr) {
            return Result::ParameterPointerEqualsNullptr;
        }

        // 读取当前关节角作为 IK 初值
        JntArray q_curr = hardware_->GetPosition();
        if (q_curr.rows() == 0) {
            return Result::JointStateError;
        }

        // 执行逆运动学求解
        JntArray q_out;
        q_out.resize(q_curr.rows());
        Result ik_res = model_->InverseKinematics(q_curr, *frame_ref, q_out);
        if (ik_res != Result::NoError) {
            return ik_res;
        }

        // 校验 IK 输出有效性
        for (unsigned int i = 0; i < q_out.rows(); ++i) {
            if (!std::isfinite(q_out(i))) {
                return Result::IkCalcFail;
            }
        }

        q_cmd = q_out;
        return Result::NoError;
    }

    return Result::MoveUnknown;
}

Result PositionController::UpdateCmd(const JntArray& q_cmd) {
    if (hardware_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const Result valid = ValidatePositionCommand(q_cmd);
    if (valid != Result::NoError) {
        return valid;
    }

    // 校验通过后记录本周期指令，作为下一周期指令速度校验的基线
    q_last_cmd_ = q_cmd;
    has_last_cmd_ = true;

    // 首次调用时切换伺服模式为 CSP (Cyclic Synchronous Position, mode=8)
    if (!mode_set_) {
        hardware_->SetMode(8);
        mode_set_ = true;
    }

    hardware_->SetPosition(q_cmd);
    return Result::NoError;
}

Result PositionController::ValidatePositionCommand(const JntArray& q_cmd) {
    if (hardware_ == nullptr || model_ == nullptr) {
        return Result::ParameterPointerEqualsNullptr;
    }

    const JntArray q_actual = hardware_->GetPosition();
    if (q_actual.rows() != static_cast<unsigned int>(model_->GetJointNum()))
        return Result::UnmatchedJointsNumber;
    for (unsigned int i = 0; i < q_actual.rows(); ++i)
        if (!std::isfinite(q_actual(i))) return Result::ParameterNanOrInf;
    const double dt = static_cast<double>(hardware_->GetDt()) / 1'000'000.0;
    const auto* previous = has_last_cmd_ && q_last_cmd_.rows() == q_cmd.rows()
        ? &q_last_cmd_ : nullptr;
    JointCommandViolation violation;
    const auto result = ValidateJointCommand(*model_, q_cmd, previous, dt, &violation);
    if (result == Result::PosLimit || result == Result::SpeedLimit) {
        emitLogEvent(result == Result::PosLimit ? LogEventType::PositionLimit : LogEventType::VelocityLimit,
                     LogModule::PositionController, static_cast<int>(result), violation.joint,
                     violation.value, violation.lower, violation.upper);
    }
    return result;
}

} // namespace rocos
