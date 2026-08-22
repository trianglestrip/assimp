#pragma once
#include <taskflow/taskflow.hpp>
#include <thread>
namespace ap {
inline tf::Executor& globalExecutor() {
    static tf::Executor exec(std::max(1u, std::thread::hardware_concurrency()));
    return exec;
}
}
