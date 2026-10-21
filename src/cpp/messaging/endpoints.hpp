#pragma once
#include <string>

/**
 * @namespace endpoints
 * @brief ZeroMQ ports and topics shared by the Core nodes.
 *
 * Each topic has exactly one publisher, which binds; subscribers connect.
 * Keep in sync with src/python/messaging/endpoints.py.
 */
namespace endpoints {

struct Endpoint {
    int port;
    const char *topic;

    /// Address for the publisher to bind (all interfaces).
    std::string bind() const { return "tcp://*:" + std::to_string(port); }

    /// Address for a subscriber on the same machine to connect to.
    std::string connect(const std::string &host = "127.0.0.1") const {
        return "tcp://" + host + ":" + std::to_string(port);
    }
};

// sensors (telemetry node / imu_node, or the simulator standing in for them)
inline constexpr Endpoint TELEMETRY{5555, "telemetry"};         // SensorData
inline constexpr Endpoint IMU{5557, "imu"};                     // ImuData

// depth control
inline constexpr Endpoint DEPTH_CONTROL_COMMAND{5560, "depth_control_command"}; // depthControlCommand
inline constexpr Endpoint DEPTH_CONTROL_OUTPUT{5561, "depth_control_output"};   // depthControlOutput
inline constexpr Endpoint DEPTH_CONTROL_DATA{5564, "depth_control_data"};       // depthControlData

// pilot / autonomy -> thruster control -> thruster mapping
inline constexpr Endpoint CONTROLLER_COMMAND{5562, "controller_command"};       // ControllerCommand
inline constexpr Endpoint THRUSTER_COMMAND{5563, "thruster_command"};           // ThrusterCommand

// autonomy behaviors
inline constexpr Endpoint AUTONOMY_STATUS{5565, "autonomy_status"};             // AutonomyStatus

}
