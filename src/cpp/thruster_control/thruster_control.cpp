/**
 * thruster_control node
 *
 * Merges the pilot/autonomy command with depth control through
 * genThrusterCommand and sends one body-frame effort command on to the
 * thruster mapping (the simulator stands in for the mapping + thrusters).
 *   in:  ControllerCommand   (surface joystick or an autonomy node)
 *        depthControlOutput  (depth_control)
 *        ImuData             (imu_node: measured attitude)
 *   out: ThrusterCommand     (body frame, each axis in [-1, 1])
 *
 * Frames: body x forward, y left, z up; world z up. The depth controller's
 * output is positive for "go deeper", i.e. world -z.
 */
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "common/node.hpp"
#include "messaging/endpoints.hpp"
#include "messaging/publisher.hpp"
#include "messaging/subscriber.hpp"
#include "telemetry.pb.h"
#include "thruster_control.hpp"

static constexpr double LOOP_HZ = 100.0;
// stop the thrusters if the pilot/autonomy goes quiet for this long
static constexpr double COMMAND_TIMEOUT_S = 0.5;
static constexpr double DEPTH_TIMEOUT_S = 0.5;
static constexpr double IMU_TIMEOUT_S = 0.5;

/// roll/pitch/yaw effort packed as a rotation, the way genThrusterCommand
/// carries the rotational part of the command (ZYX order)
static Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// Inverse of rpyToRotation. Not Eigen's eulerAngles(), which returns the
/// first angle in [0, pi] and so flips the sign of a negative yaw effort.
static Eigen::Vector3d rotationToRpy(const Eigen::Matrix3d &R) {
  double roll = std::atan2(R(2, 1), R(2, 2));
  double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  double yaw = std::atan2(R(1, 0), R(0, 0));
  return {roll, pitch, yaw};
}

static float clamp1(double v) { return static_cast<float>(std::clamp(v, -1.0, 1.0)); }

int main() {
  node::install_signal_handlers();

  rov::telemetry::ControllerCommand pilot;
  rov::telemetry::depthControlOutput depth;
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  auto never = node::Clock::now() - std::chrono::hours(1);
  auto pilot_time = never, depth_time = never, imu_time = never;

  CppMsg::Subscriber<rov::telemetry::ControllerCommand> pilot_sub(
      endpoints::CONTROLLER_COMMAND.connect(), endpoints::CONTROLLER_COMMAND.topic,
      [&](const rov::telemetry::ControllerCommand &msg) {
        pilot = msg;
        pilot_time = node::Clock::now();
      });
  CppMsg::Subscriber<rov::telemetry::depthControlOutput> depth_sub(
      endpoints::DEPTH_CONTROL_OUTPUT.connect(), endpoints::DEPTH_CONTROL_OUTPUT.topic,
      [&](const rov::telemetry::depthControlOutput &msg) {
        depth = msg;
        depth_time = node::Clock::now();
      });
  CppMsg::Subscriber<rov::telemetry::ImuData> imu_sub(
      endpoints::IMU.connect(), endpoints::IMU.topic,
      [&](const rov::telemetry::ImuData &msg) {
        const auto &q = msg.orientation();
        attitude = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).normalized();
        imu_time = node::Clock::now();
      });
  CppMsg::Publisher out(endpoints::THRUSTER_COMMAND.bind(), endpoints::THRUSTER_COMMAND.topic);

  std::cout << "thruster_control: running at " << LOOP_HZ << " Hz" << std::endl;

  const auto period = std::chrono::duration_cast<node::Clock::duration>(
      std::chrono::duration<double>(1.0 / LOOP_HZ));
  auto next = node::Clock::now();

  while (node::running()) {
    next += period;
    while (pilot_sub.spinOnce(std::chrono::milliseconds{0})) {}
    while (depth_sub.spinOnce(std::chrono::milliseconds{0})) {}
    while (imu_sub.spinOnce(std::chrono::milliseconds{0})) {}

    rov::telemetry::ThrusterCommand cmd;
    cmd.set_timestamp_us(node::now_us());

    bool pilot_ok = node::seconds_since(pilot_time) < COMMAND_TIMEOUT_S;
    bool depth_ok = node::seconds_since(depth_time) < DEPTH_TIMEOUT_S && depth.enabled() &&
                    pilot_ok && pilot.enable_depth_control();
    bool imu_ok = node::seconds_since(imu_time) < IMU_TIMEOUT_S;

    if (pilot_ok) {
      Eigen::Isometry3d joystick = Eigen::Isometry3d::Identity();
      joystick.translation() = Eigen::Vector3d(pilot.x(), pilot.y(), pilot.z());
      joystick.linear() = rpyToRotation(pilot.roll(), pilot.pitch(), pilot.yaw());

      Eigen::Isometry3d result = joystick;
      if (depth_ok) {
        // without a fresh attitude, assume level rather than skip depth hold
        Eigen::AngleAxisd pose(imu_ok ? attitude : Eigen::Quaterniond::Identity());
        result = genThrusterCommand(pose, joystick, static_cast<float>(-depth.command()));
      }

      Eigen::Vector3d rpy = rotationToRpy(result.linear());
      cmd.set_x(clamp1(result.translation().x()));
      cmd.set_y(clamp1(result.translation().y()));
      cmd.set_z(clamp1(result.translation().z()));
      cmd.set_roll(clamp1(rpy.x()));
      cmd.set_pitch(clamp1(rpy.y()));
      cmd.set_yaw(clamp1(rpy.z()));
    }
    // else: all zero, thrusters stop

    out.publish(cmd);
    std::this_thread::sleep_until(next);
  }
  return 0;
}
