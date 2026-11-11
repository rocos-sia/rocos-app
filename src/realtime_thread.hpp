#pragma once

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace rocos {

struct RealtimeThreadStatus {
    int requested_priority{};
    int policy{-1};
    int priority{};
    int error{};
    int query_error{};

    bool ready() const {
        return error == 0 && query_error == 0 &&
               (policy == SCHED_FIFO || policy == SCHED_RR) &&
               priority >= requested_priority;
    }

    std::string description() const {
        const char* name = policy == SCHED_RR ? "SCHED_RR" :
                           policy == SCHED_FIFO ? "SCHED_FIFO" :
                           policy == SCHED_OTHER ? "SCHED_OTHER" : "OTHER_OR_UNKNOWN";
        return "requested=SCHED_RR/" + std::to_string(requested_priority) +
               " actual=" + name + "/" + std::to_string(priority) +
               " set_error=" + std::to_string(error) + " (" + std::strerror(error) + ")" +
               " query_error=" + std::to_string(query_error);
    }
};

// 仅设置调用线程；保留调用方已经配置的更高实时优先级。
inline RealtimeThreadStatus ConfigureRealtimeThread(int priority) {
    RealtimeThreadStatus status;
    status.requested_priority = priority;
    sched_param current{};
    status.query_error = pthread_getschedparam(pthread_self(), &status.policy, &current);
    status.priority = current.sched_priority;
    if (status.query_error != 0) return status;
    if (priority < sched_get_priority_min(SCHED_RR) ||
        priority > sched_get_priority_max(SCHED_RR)) {
        status.error = EINVAL;
        return status;
    }
    if (status.ready()) return status;
    sched_param requested{};
    requested.sched_priority = priority;
    status.error = pthread_setschedparam(pthread_self(), SCHED_RR, &requested);
    status.query_error = pthread_getschedparam(pthread_self(), &status.policy, &current);
    status.priority = current.sched_priority;
    return status;
}

}  // namespace rocos
