#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot.hpp"

namespace rocos::dualarm {
using Json = nlohmann::json;
struct ApiError : std::invalid_argument {
  ApiError(int value, const std::string& message)
      : std::invalid_argument(message), code(value) {}
  int code;
};
inline double Number(const Json& value) {
  if (!value.is_number() || !std::isfinite(value.get<double>()))
    throw ApiError(1001, "Expected a finite number");
  return value.get<double>();
}
inline double Number(const Json& body, const char* key, double fallback) {
  return body.contains(key) ? Number(body.at(key)) : fallback;
}
inline double Positive(const Json& body, const char* key, double fallback) {
  const double value = Number(body, key, fallback);
  if (value <= 0) throw ApiError(1001, std::string(key) + " must be positive");
  return value;
}
inline std::vector<double> Values(const Json& body, const char* key,
                                  size_t count, int count_error = 1001) {
  if (!body.contains(key) || !body.at(key).is_array())
    throw ApiError(1001, std::string("Missing array: ") + key);
  if (body.at(key).size() != count)
    throw ApiError(count_error, std::string("Invalid vector size: ") + key);
  std::vector<double> result;
  for (const auto& value : body.at(key)) result.push_back(Number(value));
  return result;
}
inline JntArray Joints(const Json& body, int count) {
  const auto values = Values(body, "joints", count, 1003);
  JntArray result(count);
  for (int i = 0; i < count; ++i) result(i) = values[i];
  return result;
}
inline Frame ParsePose(const Json& value) {
  const auto& p = value.at("position");
  const auto& q = value.at("orientation");
  const double x = Number(q.at("x")), y = Number(q.at("y")),
               z = Number(q.at("z")), w = Number(q.at("w"));
  const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
  if (!std::isfinite(norm) || norm < 1e-12)
    throw ApiError(1001, "Invalid quaternion");
  return Frame(Rotation::Quaternion(x / norm, y / norm, z / norm, w / norm),
               Vector(Number(p.at("x")), Number(p.at("y")), Number(p.at("z"))));
}
inline Json PoseJson(const Frame& frame) {
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  return {{"position",
           {{"x", frame.p.x()}, {"y", frame.p.y()}, {"z", frame.p.z()}}},
          {"orientation", {{"x", x}, {"y", y}, {"z", z}, {"w", w}}}};
}
inline bool Succeeded(Result result) {
  return result == Result::NoError || result == Result::PlanFinished;
}
}  // namespace rocos::dualarm
