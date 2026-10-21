#pragma once
#include <chrono>

#include "common/node.hpp"
#include "messaging/endpoints.hpp"
#include "messaging/subscriber.hpp"
#include "telemetry.pb.h"

/**
 * @class DepthControlSubscriber
 * @brief Latest depthControlCommand from the surface (enable, target depth,
 * and tuning weights while testing).
 *
 * If commands stop arriving for COMMAND_TIMEOUT_S, control reads as disabled,
 * so losing the tether doesn't leave the ROV holding a stale depth.
 */
class DepthControlSubscriber {
public:
  static constexpr double COMMAND_TIMEOUT_S = 1.0;

  DepthControlSubscriber()
      : sub(endpoints::DEPTH_CONTROL_COMMAND.connect(), endpoints::DEPTH_CONTROL_COMMAND.topic,
            [this](const rov::telemetry::depthControlCommand &msg) {
              last = msg;
              received = true;
              last_time = node::Clock::now();
            }) {}

  /// Drain every queued command, waiting up to `timeout` for the first.
  void spinOnce(std::chrono::milliseconds timeout = std::chrono::milliseconds{0}) {
    if (sub.spinOnce(timeout)) {
      while (sub.spinOnce(std::chrono::milliseconds{0})) {}
    }
  }

  bool getEnableControl() const {
    return received && node::seconds_since(last_time) < COMMAND_TIMEOUT_S && last.enable_control();
  }
  bool getEnableTesting() const { return getEnableControl() && last.enable_testing(); }
  double getTargetDepth() const { return last.target_depth(); }
  double getKp() const { return last.k_p(); }
  double getKi() const { return last.k_i(); }
  double getKd() const { return last.k_d(); }

private:
  rov::telemetry::depthControlCommand last;
  bool received = false;
  node::Clock::time_point last_time;
  CppMsg::Subscriber<rov::telemetry::depthControlCommand> sub;
};
