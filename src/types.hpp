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
#pragma once

#include "math_types.hpp"
#include <cstdint>

#include <variant>

namespace rocos {
  enum class JntState : uint8_t {
    DISABLED,
    ENABLED,
    ERROR
  };


  typedef std::variant<JntArray, Frame> Reference;
  typedef std::variant<JntArray, Twist> JogVec; //点动方向向量


}  // namespace rocos