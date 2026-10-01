#!/usr/bin/env python3
"""
Live IMU + stereo depth over USB 2.0 on the OAK-D Pro W (BNO086 + OV9782 + 2x OV9282).

Forces the MyriadX to boot USB 2.0 firmware via dai.UsbSpeed.HIGH so this exercises
the same 480 Mbps link budget the ROV Pi will have. Measured ceilings on this rig (USB 2.0 forced, depth RAW16 + IMU):
  - 400p@30 -> 29.5 fps, 3.60 MB/s, depth 320x200
  - 400p@60 -> 43.7 fps, 5.34 MB/s, depth 320x200
  - 720p@30 -> 20.9 fps, 9.17 MB/s, depth 640x360
  - 800p@30 -> 19.0 fps, 9.27 MB/s, depth 640x400  (VPU-bound, not USB-bound)
  - IMU is ~5 KiB/s and costs nothing even at full video rate.
  - Raw uncompressed 800p sensor pairs halve to ~15 fps over USB 2.0. Do NOT stream
    raw mono on the vehicle -- stream the StereoDepth node's output instead.

Levers that did NOT help here (measured, do not bother):
  setXLinkChunkSize(0), subpixel on/off, median OFF, camera low-bandwidth mode.
Levers that DID help:
  setPostProcessingHardwareResources(4, 4) -> +5% at 800p.

Usage:
    python live_imu_stereo.py                      # 400p@30 depth + 100 Hz IMU
    python live_imu_stereo.py --resolution 800p --fps 15
    python live_imu_stereo.py --superspeed         # compare against USB 3.0

Prints a 2 s rolling summary: depth fps, MB/s, IMU Hz, orientation accuracy.
"""

import argparse
import sys
import time

try:
    import depthai as dai
except ImportError:
    print("DepthAI not installed. Use: pip install \"depthai<3\"", file=sys.stderr)
    sys.exit(1)

RESOLUTIONS = {
    "400p": dai.MonoCameraProperties.SensorResolution.THE_400_P,
    "720p": dai.MonoCameraProperties.SensorResolution.THE_720_P,
    "800p": dai.MonoCameraProperties.SensorResolution.THE_800_P,
}


def build_pipeline(resolution, fps, imu_rate):
    """One pipeline, one dai.Device: IMU and stereo share the same device handle."""
    pipeline = dai.Pipeline()

    left = pipeline.create(dai.node.MonoCamera)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.MonoCamera)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    for cam in (left, right):
        cam.setResolution(resolution)
        cam.setFps(fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    # v2 has no FAST_ACCURACY; ROBOTICS is the fast preset for a moving platform.
    stereo.initialConfig.setConfidenceThreshold(200)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    # Measured on this rig: +~5% depth fps (18.1 -> 19.0) at 800p. The default is
    # 3 shaves / 3 memory slices; 4/4 is the most that fits on RVC2 stereo
    # post-processing. 2/2 or 8/8 are both worse (8/8 fails to open).
    stereo.setPostProcessingHardwareResources(4, 4)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    # NOTE: dai.node.MessageEncoder does not exist in DepthAI v2 (it is a v3 node), so
    # there is no H264/H265 depth-encoding option here. On v2 the way to cut depth
    # bandwidth is to lower the sensor resolution/fps or use H265 in an RTSP/ffmpeg
    # pipeline outside DepthAI.
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, imu_rate)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, imu_rate)
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, imu_rate)
    # Latency-first: one sample per transfer. Safe, because IMU is ~5 KiB/s.
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    xout_imu = pipeline.create(dai.node.XLinkOut)
    xout_imu.setStreamName("imu")
    imu.out.link(xout_imu.input)

    return pipeline


def main():
    parser = argparse.ArgumentParser(description="Live IMU + stereo depth over USB 2.0")
    parser.add_argument("--resolution", default="400p", choices=sorted(RESOLUTIONS),
                        help="mono sensor resolution (default: 400p)")
    parser.add_argument("--fps", type=int, default=30, help="sensor fps (default: 30)")
    parser.add_argument("--imu-rate", type=int, default=100, help="IMU report rate Hz (default: 100)")
    parser.add_argument("--superspeed", action="store_true",
                        help="use SuperSpeed instead of forcing USB 2.0")
    parser.add_argument("--seconds", type=float, default=0, help="exit after N seconds (0 = run forever)")
    args = parser.parse_args()

    pipeline = build_pipeline(RESOLUTIONS[args.resolution], args.fps, args.imu_rate)

    mode = "SuperSpeed" if args.superspeed else "USB 2.0 (forced)"
    print(f"[live] {mode} | mono {args.resolution} @ {args.fps}fps | "
          f"IMU {args.imu_rate} Hz | depth raw RAW16")

    try:
        # usb2Mode=True is the deprecated spelling; UsbSpeed.HIGH is the current API.
        device = (
            dai.Device(pipeline) if args.superspeed
            else dai.Device(pipeline, dai.UsbSpeed.HIGH)
        )
    except Exception as e:
        print(f"[live] FAILED to open device: {type(e).__name__}: {e}", file=sys.stderr)
        print("[live] If this is INSUFFICIENT_PERMISSIONS, another process holds the "
              "OAK-D. Only one dai.Device can be open at a time.", file=sys.stderr)
        return 1

    depth_q = device.getOutputQueue(name="depth", maxSize=30, blocking=False)
    imu_q = device.getOutputQueue(name="imu", maxSize=200, blocking=False)

    print("[live] streaming. NOTE: on this unit rotationVectorAccuracy is pinned at "
          "pi (3.1416) and carries no information -- ignore that field and use the "
          "quaternion, which does track motion.\n")

    depth_frames = imu_packets = depth_bytes = 0
    window_start = time.time()
    started = time.time()
    last_acc = None

    try:
        while True:
            frame = depth_q.tryGet()
            if frame is not None:
                depth_frames += 1
                try:
                    depth_bytes += frame.getDataSize() if hasattr(frame, "getDataSize") \
                        else len(frame.getData())
                except Exception:
                    pass

            msg = imu_q.tryGet()
            if msg is not None:
                for packet in getattr(msg, "packets", []):
                    imu_packets += 1
                    last_acc = packet.rotationVector.rotationVectorAccuracy

            now = time.time()
            if now - window_start >= 2.0:
                elapsed = now - window_start
                acc_txt = f"{last_acc:.2f} rad" if last_acc is not None else "n/a"
                print(f"[live] depth {depth_frames / elapsed:5.1f} fps | "
                      f"{depth_bytes / elapsed / 1048576:5.2f} MB/s | "
                      f"IMU {imu_packets / elapsed:6.1f} Hz | "
                      f"orient-acc {acc_txt}")
                depth_frames = imu_packets = depth_bytes = 0
                window_start = now

            if args.seconds and (now - started) >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\n[live] interrupted")
    finally:
        device.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())