"""ZeroMQ ports and topics shared by the Core nodes.

Each topic has exactly one publisher, which binds; subscribers connect.
Keep in sync with src/cpp/messaging/endpoints.hpp.
"""
from typing import NamedTuple


class Endpoint(NamedTuple):
    port: int
    topic: str

    def bind(self) -> str:
        """Address for the publisher to bind (all interfaces)."""
        return f"tcp://*:{self.port}"

    def connect(self, host: str = "127.0.0.1") -> str:
        """Address for a subscriber to connect to."""
        return f"tcp://{host}:{self.port}"


# sensors (telemetry node / imu_node, or the simulator standing in for them)
TELEMETRY = Endpoint(5555, "telemetry")                           # SensorData
IMU = Endpoint(5557, "imu")                                       # ImuData

# depth control
DEPTH_CONTROL_COMMAND = Endpoint(5560, "depth_control_command")   # depthControlCommand
DEPTH_CONTROL_OUTPUT = Endpoint(5561, "depth_control_output")     # depthControlOutput
DEPTH_CONTROL_DATA = Endpoint(5564, "depth_control_data")         # depthControlData

# pilot / autonomy -> thruster control -> thruster mapping
CONTROLLER_COMMAND = Endpoint(5562, "controller_command")         # ControllerCommand
THRUSTER_COMMAND = Endpoint(5563, "thruster_command")             # ThrusterCommand

# autonomy behaviors
AUTONOMY_STATUS = Endpoint(5565, "autonomy_status")               # AutonomyStatus
