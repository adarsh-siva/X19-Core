/**
 * straight_line node: drive the ROV in a straight line autonomously.
 *
 *   settle  enable depth control at the target depth, lock the heading
 *   cruise  surge forward while holding depth and heading
 *   stop    surge off, keep holding depth and heading
 *
 * It drives the rest of Core exactly like the surface would: it sends
 * depthControlCommand to depth_control and ControllerCommand to
 * thruster_control. Heading is held with the same Controller class as depth.
 *   in:  ImuData (heading), SensorData (depth, to know when settled)
 *   out: depthControlCommand, ControllerCommand, AutonomyStatus
 *
 * Only on-board sensors are used, so "straight" means constant heading at
 * constant depth; a cross current still pushes the track sideways.
 *
 * Usage: straight_line [--depth M] [--surge F] [--cruise-time S]
 *                      [--heading-deg D] [--heading-gains KP KI KD] [--exit-after S]
 */
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "common/node.hpp"
#include "depth_control/controller.hpp"
#include "messaging/endpoints.hpp"
#include "messaging/publisher.hpp"
#include "messaging/subscriber.hpp"
#include "telemetry.pb.h"

// heading PID, output in [-1, 1] of full yaw authority per radian of error.
// Tuned in the MuJoCo sim through the full node chain; retune in the pool
// with --heading-gains. kp 1.0 put the loop at ~2.5 Hz, where the pub/sub hops
// and thruster spin-up add enough lag for a +-1.2 deg limit cycle.
static constexpr Weights HEADING_WEIGHTS = {.k_p = 0.3, .k_i = 0.03, .k_d = 0.05};

static constexpr double LOOP_HZ = 50.0;
static constexpr double SETTLE_TOLERANCE = 0.05; // m
static constexpr double SETTLE_HOLD = 2.0;       // s within tolerance before cruising
static constexpr double SETTLE_TIMEOUT = 20.0;   // s, cruise anyway after this

struct Options {
  double depth = 1.5;
  double surge = 0.5;
  double cruise_time = 8.0;
  bool heading_given = false;
  double heading = 0.0;   // rad
  double exit_after = -1; // s after stopping; < 0 runs until killed
  Weights heading_weights = HEADING_WEIGHTS;
};

static Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    auto next = [&]() -> double {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << argv[i] << std::endl;
        std::exit(2);
      }
      return std::atof(argv[++i]);
    };
    if (!std::strcmp(argv[i], "--depth")) o.depth = next();
    else if (!std::strcmp(argv[i], "--surge")) o.surge = next();
    else if (!std::strcmp(argv[i], "--cruise-time")) o.cruise_time = next();
    else if (!std::strcmp(argv[i], "--heading-deg")) { o.heading = next() * M_PI / 180.0; o.heading_given = true; }
    else if (!std::strcmp(argv[i], "--exit-after")) o.exit_after = next();
    else if (!std::strcmp(argv[i], "--heading-gains")) {
      o.heading_weights.k_p = next();
      o.heading_weights.k_i = next();
      o.heading_weights.k_d = next();
    }
    else {
      std::cerr << "usage: straight_line [--depth M] [--surge F] [--cruise-time S] "
                   "[--heading-deg D] [--heading-gains KP KI KD] [--exit-after S]" << std::endl;
      std::exit(2);
    }
  }
  return o;
}

static double wrap(double a) {
  a = std::fmod(a + M_PI, 2 * M_PI);
  return (a < 0 ? a + 2 * M_PI : a) - M_PI;
}

int main(int argc, char **argv) {
  node::install_signal_handlers();
  Options opt = parse(argc, argv);

  double yaw = 0, depth = 0;
  bool have_imu = false, have_depth = false;

  CppMsg::Subscriber<rov::telemetry::ImuData> imu_sub(
      endpoints::IMU.connect(), endpoints::IMU.topic,
      [&](const rov::telemetry::ImuData &msg) {
        const auto &q = msg.orientation();
        yaw = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        have_imu = true;
      });
  CppMsg::Subscriber<rov::telemetry::SensorData> telemetry_sub(
      endpoints::TELEMETRY.connect(), endpoints::TELEMETRY.topic,
      [&](const rov::telemetry::SensorData &msg) {
        depth = msg.depth();
        have_depth = true;
      });
  CppMsg::Publisher depth_pub(endpoints::DEPTH_CONTROL_COMMAND.bind(), endpoints::DEPTH_CONTROL_COMMAND.topic);
  CppMsg::Publisher command_pub(endpoints::CONTROLLER_COMMAND.bind(), endpoints::CONTROLLER_COMMAND.topic);
  CppMsg::Publisher status_pub(endpoints::AUTONOMY_STATUS.bind(), endpoints::AUTONOMY_STATUS.topic);

  Controller heading(opt.heading_weights);

  std::string phase = "wait"; // until the first IMU and depth samples
  auto phase_start = node::Clock::now();
  double settled_for = 0;
  auto enter = [&](const char *p) {
    phase = p;
    phase_start = node::Clock::now();
    std::cout << "straight_line: " << phase << std::endl;
  };

  std::cout << "straight_line: depth " << opt.depth << " m, surge " << opt.surge
            << ", cruise " << opt.cruise_time << " s" << std::endl;

  const double dt = 1.0 / LOOP_HZ;
  const auto period = std::chrono::duration_cast<node::Clock::duration>(std::chrono::duration<double>(dt));
  auto next = node::Clock::now();

  while (node::running()) {
    next += period;
    while (imu_sub.spinOnce(std::chrono::milliseconds{0})) {}
    while (telemetry_sub.spinOnce(std::chrono::milliseconds{0})) {}

    if (phase == "wait") {
      if (have_imu && have_depth) {
        if (!opt.heading_given) opt.heading = yaw; // lock the current heading
        heading.target = opt.heading;
        std::cout << "straight_line: heading locked at " << opt.heading * 180.0 / M_PI << " deg" << std::endl;
        enter("settle");
      }
      std::this_thread::sleep_until(next);
      continue;
    }

    // heading: hand the controller a measurement unwrapped around its target,
    // so crossing +-180 deg is a small error rather than a 360 deg one
    heading.record_measurement(heading.target - wrap(heading.target - yaw));
    double yaw_cmd = heading.compute_filtered(dt);

    double in_phase = node::seconds_since(phase_start);
    if (phase == "settle") {
      settled_for = std::abs(depth - opt.depth) < SETTLE_TOLERANCE ? settled_for + dt : 0.0;
      if (settled_for >= SETTLE_HOLD || in_phase >= SETTLE_TIMEOUT) enter("cruise");
    } else if (phase == "cruise" && in_phase >= opt.cruise_time) {
      enter("stop");
    } else if (phase == "stop" && opt.exit_after >= 0 && in_phase >= opt.exit_after) {
      break;
    }

    rov::telemetry::depthControlCommand dc;
    dc.set_enable_control(true);
    dc.set_enable_testing(false);
    dc.set_target_depth(opt.depth);
    depth_pub.publish(dc);

    rov::telemetry::ControllerCommand cmd;
    cmd.set_x(phase == "cruise" ? opt.surge : 0.0);
    cmd.set_yaw(yaw_cmd);
    cmd.set_enable_depth_control(true);
    cmd.set_target_depth(opt.depth);
    command_pub.publish(cmd);

    rov::telemetry::AutonomyStatus status;
    status.set_timestamp_us(node::now_us());
    status.set_behavior("straight_line");
    status.set_phase(phase);
    status.set_target_heading(opt.heading);
    status.set_target_depth(opt.depth);
    status_pub.publish(status);

    std::this_thread::sleep_until(next);
  }

  // hand back: depth control off, thrusters idle
  rov::telemetry::depthControlCommand off;
  depth_pub.publish(off);
  command_pub.publish(rov::telemetry::ControllerCommand());
  return 0;
}
