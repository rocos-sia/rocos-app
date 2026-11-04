#pragma once

#include <string>

namespace rocos {
// 初始化时将多级 mimic 展开到最终非 mimic 源关节。
struct MimicJoint {
  std::string name;
  std::string source;
  double multiplier{1};
  double offset{0};
};
}  // namespace rocos
