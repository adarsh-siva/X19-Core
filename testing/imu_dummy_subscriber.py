#!/usr/bin/env python3
"""
Proof-of-concept consumer for imu_node.py's published IMU data.
Stands in for pid_node.py until the vertical-stabilization control loop exists -
just proves IMU data is reaching a subscriber, printed to terminal.
"""

import argparse

from src.python.messaging import Subscriber
from src.protocols.python import telemetry_pb2


def imu_callback(data: "telemetry_pb2.ImuData") -> None:
    a = data.acceleration
    g = data.angular_velocity
    q = data.orientation
    print(
        f"[t={data.timestamp_us}] "
        f"accel=({a.x:+.2f}, {a.y:+.2f}, {a.z:+.2f}) m/s^2  "
        f"gyro=({g.x:+.2f}, {g.y:+.2f}, {g.z:+.2f}) rad/s  "
        f"quat=({q.x:+.2f}, {q.y:+.2f}, {q.z:+.2f}, {q.w:+.2f}) acc={q.accuracy_rad:.2f}rad",
        flush=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Dummy IMU subscriber (PoC)")
    parser.add_argument("--address", default="tcp://127.0.0.1:5557",
                         help="ZMQ address to connect to (imu_node's publisher)")
    parser.add_argument("--topic", default="imu", help="ZMQ topic to subscribe to")
    args = parser.parse_args()

    subscriber = Subscriber(
        address=args.address,
        topic=args.topic,
        message_type=telemetry_pb2.ImuData,
        callback=imu_callback,
    )

    print(f"[IMU Dummy Subscriber] Listening on {args.address} (topic='{args.topic}'). "
          f"Press Ctrl+C to stop.", flush=True)

    try:
        while True:
            subscriber.spin_once(timeout_ms=100)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()


if __name__ == "__main__":
    main()
