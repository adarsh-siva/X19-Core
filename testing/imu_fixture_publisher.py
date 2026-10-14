#!/usr/bin/env python3
"""
Synthetic IMU publisher for testing the ZMQ/proto wiring without the OAK-D
camera attached - publishes a slow sine-wave tilt on the same topic/schema
imu_node.py uses, so imu_dummy_subscriber.py (or later pid_node.py) can be
exercised on a dev machine.
"""

import argparse
import math
import time

from src.python.messaging import Publisher
from src.protocols.python import telemetry_pb2


def main():
    parser = argparse.ArgumentParser(description="Synthetic IMU fixture publisher")
    parser.add_argument("--address", default="tcp://127.0.0.1:5557")
    parser.add_argument("--topic", default="imu")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    args = parser.parse_args()

    publisher = Publisher(address=args.address, topic=args.topic)
    period = 1.0 / args.rate_hz
    start = time.time()

    print(f"[IMU Fixture Publisher] Publishing synthetic IMU data on {args.address} "
          f"(topic='{args.topic}') at {args.rate_hz} Hz. Press Ctrl+C to stop.", flush=True)

    try:
        while True:
            t = time.time() - start
            pitch_rad = 0.2 * math.sin(t)  # slow +/-0.2 rad tilt

            msg = telemetry_pb2.ImuData()
            msg.timestamp_us = int(time.time() * 1e6)
            msg.acceleration.x = 9.81 * math.sin(pitch_rad)
            msg.acceleration.z = 9.81 * math.cos(pitch_rad)
            msg.angular_velocity.y = 0.2 * math.cos(t)
            msg.orientation.y = math.sin(pitch_rad / 2)
            msg.orientation.w = math.cos(pitch_rad / 2)
            msg.orientation.accuracy_rad = 0.05

            publisher.publish(msg)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
