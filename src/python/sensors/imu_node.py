#!/usr/bin/env python3
"""
IMU publisher node for the OAK-D W Pro's onboard BNO086 IMU.

Reads calibrated accelerometer, calibrated gyroscope, and the fused rotation
vector (quaternion) off the camera's RVC2 processor via DepthAI, and
publishes each sample as an ImuData protobuf message over ZMQ.

Run (from the X19-Core repo root, with the venv's deps installed):
    export PYTHONPATH=$(pwd)
    .venv/bin/python src/python/sensors/imu_node.py [--address ADDR] [--topic TOPIC] [--rate-hz N]

Dependencies:
    - DepthAI v2.x (NOT v3 - v3 removed the dai.node.XLinkOut / getOutputQueue
      API this file uses in favor of createOutputQueue(); this repo's other
      DepthAI node, cv_camera_connect.py, is also written against v2):
          pip install "depthai<3"
      Docs: https://docs.luxonis.com/software/depthai-components/nodes/imu/
      IMU example source: https://github.com/luxonis/depthai-python/tree/main/examples/IMU
    - A udev rule so DepthAI can access the camera over USB without root
      (one-time, per machine - see README's IMU Node section for the exact
      rule and why a physical unplug/replug is required afterward).

See the top-level README ("IMU Node" section) for full setup instructions,
OAK-D configuration rationale, and port/topic reference.
"""

import sys
import time
import signal
import argparse
import threading

try:
    import depthai as dai
except ImportError:
    dai = None

from src.python.messaging import Publisher
from src.protocols.python import telemetry_pb2

quitEvent = threading.Event()

signal.signal(signal.SIGTERM, lambda *_args: quitEvent.set())
signal.signal(signal.SIGINT, lambda *_args: quitEvent.set())


def build_pipeline(report_rate_hz: int) -> "dai.Pipeline":
    """
    Build the on-camera DepthAI graph: one IMU node emitting three report
    types, linked to an XLinkOut so the host can read them over USB.

    Sensor choices (BNO086, all calibrated/fused):
      - ACCELEROMETER: calibrated linear acceleration, m/s^2.
      - GYROSCOPE_CALIBRATED: calibrated angular velocity, rad/s.
      - ROTATION_VECTOR: on-chip sensor-fused orientation quaternion, so
        vertical stabilization gets a drift-corrected orientation instead of
        one integrated from raw gyro data on the host.

    Batching (setBatchReportThreshold/setMaxBatchReports) controls how many
    samples get bundled into one XLink transfer before crossing USB:
      - threshold=1: send as soon as a single report is ready (lowest
        latency, more USB overhead) - the right tradeoff for a control-loop
        input, favoring responsiveness over throughput.
      - max=10: cap batch size in case the host falls behind reading.

      
    """
    pipeline = dai.Pipeline()

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, report_rate_hz)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, report_rate_hz)
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, report_rate_hz)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("imu")
    imu.out.link(xout.input)

    return pipeline


def packet_to_proto(packet) -> "telemetry_pb2.ImuData":
    """
    Map one DepthAI IMUPacket to an ImuData protobuf message.

    packet.acceleroMeter / .gyroscope / .rotationVector are DepthAI's own
    report objects (not protobuf) - field names/spelling (e.g.
    "acceleroMeter", quaternion components i/j/k/real) come straight from
    the v2 API, confirmed against Luxonis's own examples:
    https://github.com/luxonis/depthai-python/tree/main/examples/IMU

    Uses the accelerometer report's *device* timestamp (getTimestampDevice),
    not host arrival time, when the BNO086 actually captured the
    sample, which matters for anything measuring rate or lag downstream.
    """
    msg = telemetry_pb2.ImuData()

    accel = packet.acceleroMeter
    msg.timestamp_us = int(accel.getTimestampDevice().total_seconds() * 1e6)
    msg.acceleration.x = accel.x
    msg.acceleration.y = accel.y
    msg.acceleration.z = accel.z

    gyro = packet.gyroscope
    msg.angular_velocity.x = gyro.x
    msg.angular_velocity.y = gyro.y
    msg.angular_velocity.z = gyro.z

    rv = packet.rotationVector
    msg.orientation.x = rv.i
    msg.orientation.y = rv.j
    msg.orientation.z = rv.k
    msg.orientation.w = rv.real
    msg.orientation.accuracy_rad = rv.rotationVectorAccuracy

    return msg


def main():
    """
    Entry point: build the pipeline, open the device, and publish every
    IMU packet as it arrives until interrupted (SIGINT/SIGTERM).

    Only one process can hold the DepthAI device open at a time - make sure
    nothing else (e.g. cv_camera_connect.py) is running against the same
    camera before starting this node, or dai.Device(pipeline) will fail.
    
    """
    parser = argparse.ArgumentParser(description="OAK-D W Pro IMU Publisher Node")
    parser.add_argument("--address", default="tcp://127.0.0.1:5557",
                         help="ZMQ address to bind the IMU publisher to")
    parser.add_argument("--topic", default="imu", help="ZMQ topic to publish under")
    parser.add_argument("--rate-hz", type=int, default=100,
                         help="Requested IMU report rate in Hz (BNO086 rounds up)")
    args = parser.parse_args()

    if dai is None:
        print("[IMU Node] DepthAI library not installed.", flush=True)
        sys.exit(1)

    pipeline = build_pipeline(args.rate_hz)
    publisher = Publisher(address=args.address, topic=args.topic)

    print(f"[IMU Node] Publishing on {args.address} (topic='{args.topic}'). "
          f"Press Ctrl+C to stop.", flush=True)

    try:
        with dai.Device(pipeline) as device:
            imu_queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            while not quitEvent.is_set():
                imu_data = imu_queue.tryGet()
                if imu_data is None:
                    time.sleep(0.001)
                    continue
                for packet in imu_data.packets:
                    publisher.publish(packet_to_proto(packet))
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
