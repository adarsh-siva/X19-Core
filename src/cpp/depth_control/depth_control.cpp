/**
 * depth_control node
 *
 * Holds the ROV at the surface's target depth with the Controller PID.
 *   in:  depthControlCommand  (surface: enable, target depth, test weights)
 *        SensorData           (depth from the pressure sensor)
 *   out: depthControlOutput   (to thruster_control, every tick)
 *        depthControlData     (default weights, for the surface, 1 Hz)
 */
#include <cmath>
#include <iostream>
#include <thread>

#include "common/node.hpp"
#include "controller.hpp"
#include "depth_control_subscriber.hpp"
#include "messaging/endpoints.hpp"
#include "messaging/publisher.hpp"
#include "messaging/subscriber.hpp"
#include "telemetry.pb.h"

// Default weights, output in [-1, 1] of full heave authority per metre of
// error. Tuned in the MuJoCo sim (ROV_Mujoco_simulation); retune in the pool.
static constexpr Weights DEFAULT_WEIGHTS = {.k_p = 1.2, .k_i = 0.2, .k_d = 0.5};

// Only integrate within this band (m) of the target. The Controller's
// anti-windup only stops integrating while saturated, and a dive at ~40%
// output never saturates, so the integral piled up on the way down and held
// the ROV ~4 cm too deep for tens of seconds after arriving.
static constexpr double INTEGRAL_ZONE = 0.15;

static constexpr double LOOP_HZ = 50.0;
static constexpr double DEPTH_TIMEOUT_S = 0.5;

static double latest_depth = 0;
static bool have_depth = false;
static node::Clock::time_point depth_time;

double read_depth() {
  return latest_depth;
}

static CppMsg::Publisher *output_pub = nullptr;

void send_thruster_power(double value, const Controller &controller, bool enabled) {
  rov::telemetry::depthControlOutput msg;
  msg.set_timestamp_us(node::now_us());
  msg.set_enabled(enabled);
  msg.set_command(enabled ? value : 0.0);
  msg.set_measured_depth(controller.moving_avg);
  msg.set_target_depth(controller.target);
  output_pub->publish(msg);
}

int main() {
  node::install_signal_handlers();

  Controller *controller = new Controller(DEFAULT_WEIGHTS);
  DepthControlSubscriber sub;

  // recieve sensor data
  CppMsg::Subscriber<rov::telemetry::SensorData> telemetry(
      endpoints::TELEMETRY.connect(), endpoints::TELEMETRY.topic,
      [&](const rov::telemetry::SensorData &data) {
        latest_depth = data.depth();
        have_depth = true;
        depth_time = node::Clock::now();
        controller->record_measurement(read_depth());
      });

  CppMsg::Publisher output(endpoints::DEPTH_CONTROL_OUTPUT.bind(), endpoints::DEPTH_CONTROL_OUTPUT.topic);
  CppMsg::Publisher data_pub(endpoints::DEPTH_CONTROL_DATA.bind(), endpoints::DEPTH_CONTROL_DATA.topic);
  output_pub = &output;

  rov::telemetry::depthControlData defaults;
  defaults.set_k_p(DEFAULT_WEIGHTS.k_p);
  defaults.set_k_i(DEFAULT_WEIGHTS.k_i);
  defaults.set_k_d(DEFAULT_WEIGHTS.k_d);

  std::cout << "depth_control: kp=" << DEFAULT_WEIGHTS.k_p << " ki=" << DEFAULT_WEIGHTS.k_i
            << " kd=" << DEFAULT_WEIGHTS.k_d << std::endl;

  const auto period = std::chrono::duration_cast<node::Clock::duration>(
      std::chrono::duration<double>(1.0 / LOOP_HZ));
  auto next = node::Clock::now();
  auto last_tick = node::Clock::now();
  auto last_data = node::Clock::now() - std::chrono::seconds(2);
  bool was_enabled = false;

  while (node::running()) {
    next += period;

    // recieve protobuf messages
    sub.spinOnce();
    while (telemetry.spinOnce(std::chrono::milliseconds{0})) {}

    if (node::seconds_since(last_data) >= 1.0) {
      data_pub.publish(defaults);
      last_data = node::Clock::now();
    }

    // get change in time
    double dt = node::seconds_since(last_tick);
    last_tick = node::Clock::now();

    bool depth_ok = have_depth && node::seconds_since(depth_time) < DEPTH_TIMEOUT_S;
    bool enabled = sub.getEnableControl() && depth_ok;

    if (!enabled) {
      if (was_enabled) {
        std::cout << "depth_control: disabled" << (depth_ok ? "" : " (no depth data)") << std::endl;
      }
      was_enabled = false;
      send_thruster_power(0.0, *controller, false);
      std::this_thread::sleep_until(next);
      continue;
    }

    if (!was_enabled) {
      // start clean, keeping only the filter's view of the current depth
      double depth = controller->get_measurement();
      controller->reset();
      controller->record_measurement(depth);
      dt = 1.0 / LOOP_HZ;
      std::cout << "depth_control: enabled, target " << sub.getTargetDepth() << " m" << std::endl;
    }
    was_enabled = true;

    controller->target = sub.getTargetDepth();

    if (sub.getEnableTesting()) {
      controller->weights = (Weights) {
        .k_p = sub.getKp(),
        .k_i = sub.getKi(),
        .k_d = sub.getKd()
      };
    } else {
      controller->weights = DEFAULT_WEIGHTS;
    }

    if (std::abs(controller->target - controller->get_measurement()) > INTEGRAL_ZONE) {
      controller->integral = 0;
    }

    double output = controller->compute_filtered(dt);
    send_thruster_power(output, *controller, true);

    std::this_thread::sleep_until(next);
  }

  delete controller;
  return 0;
}
