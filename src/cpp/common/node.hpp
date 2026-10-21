#pragma once
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>

/**
 * @namespace node
 * @brief Small helpers shared by the C++ node executables.
 */
namespace node {

using Clock = std::chrono::steady_clock;

inline std::atomic<bool> &running() {
    static std::atomic<bool> flag{true};
    return flag;
}

/// Clear running() on SIGINT/SIGTERM, so scripts/launch.py can stop nodes cleanly.
inline void install_signal_handlers() {
    auto stop = [](int) { running().store(false); };
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
}

/// Wall-clock timestamp in microseconds, for the *_us message fields.
inline uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

inline double seconds_since(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now() - t).count();
}

}
