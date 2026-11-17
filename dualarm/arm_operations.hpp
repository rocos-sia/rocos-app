#pragma once
#include "api_values.hpp"
#include "httplib.h"

namespace rocos::dualarm {
// Called after arm-local authorization, under that arm's command mutex.
Result ExecuteMotion(Robot& robot, const std::string& action, const Json& body);
Json ReadFrames(Robot& robot, const std::string& action,
                const httplib::Request& req);
Json WriteFrames(Robot& robot, const std::string& action,
                 const httplib::Request& req, const Json& body);
}  // namespace rocos::dualarm
