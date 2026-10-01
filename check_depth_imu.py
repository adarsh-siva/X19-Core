#!/usr/bin/env python3
"""
Health-check the OAK-D stereo pair + IMU, then dump depth and IMU samples.

Run this before trusting any depth or orientation numbers. The OV9282 mono sensors
saturate (all pixels 255) if the lens caps are on or the scene is too bright, and a
saturated image still produces full-rate frames -- it just produces garbage depth.

    python check_depth_imu.py                 # check + 10s capture
    python check_depth_imu.py --seconds 30    # longer run
    python check_depth_imu.py --save          # write depth/mono PNGs + IMU CSV

Exit codes: 0 = healthy, 1 = camera saturated/blocked, 2 = device would not open.
"""

import argparse
import csv
import sys
import time

try:
    import depthai as dai
    import numpy as np
except ImportError:
    print("Need depthai and numpy. In the venv: pip install \"depthai<3\" numpy", file=sys.stderr)
    sys.exit(2)

R = dai.MonoCameraProperties.SensorResolution


def mono_stats(frame):
    """Saturation/texture stats for a mono ImgFrame."""
    a = np.frombuffer(frame.getData(), dtype=np.uint8)
    return {
        "mean": float(a.mean()),
        "min": int(a.min()),
        "max": int(a.max()),
        # >30% of pixels at 255 => blown out, stereo matching is meaningless
        "saturated_pct": float((a >= 250).mean() * 100.0),
        # low stdev => flat/untextured surface, also bad for stereo
        "stdev": float(a.std()),
    }


def health_check(res, fps):
    """
    Separate short run that pulls the full-res mono frame + confidence map.

    This MUST be its own pipeline: streaming 1280x800 RAW8 over USB 2.0 costs about
    30 MB/s at 30fps, which saturates the link and would otherwise understate the
    real depth-only throughput by roughly 10x.
    """
    p = dai.Pipeline()
    left = p.create(dai.node.MonoCamera); left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right = p.create(dai.node.MonoCamera); right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    for cam in (left, right):
        cam.setResolution(res); cam.setFps(fps)
    st = p.create(dai.node.StereoDepth)
    st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    left.out.link(st.left); right.out.link(st.right)
    xd = p.create(dai.node.XLinkOut); xd.setStreamName("d"); st.depth.link(xd.input)
    xm = p.create(dai.node.XLinkOut); xm.setStreamName("m"); left.out.link(xm.input)
    d = dai.Device(p, dai.UsbSpeed.HIGH)
    q = d.getOutputQueue(name="m", maxSize=5, blocking=False)
    stats = None
    t0 = time.time()
    while stats is None and time.time() - t0 < 20:
        m = q.tryGet()
        if m is None:
            time.sleep(0.05); continue
        stats = mono_stats(m)
    d.close()
    time.sleep(2.0)
    return stats


def capture(res, fps, seconds, save, outdir):
    pipeline = dai.Pipeline()

    left = pipeline.create(dai.node.MonoCamera)
    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.MonoCamera)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    for cam in (left, right):
        cam.setResolution(res)
        cam.setFps(fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.initialConfig.AlgorithmControl.enableExtended = False
    stereo.setPostProcessingHardwareResources(4, 4)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    x_depth = pipeline.create(dai.node.XLinkOut); x_depth.setStreamName("depth")
    stereo.depth.link(x_depth.input)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, 100)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, 100)
    imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 100)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    x_imu = pipeline.create(dai.node.XLinkOut); x_imu.setStreamName("imu")
    imu.out.link(x_imu.input)

    try:
        device = dai.Device(pipeline, dai.UsbSpeed.HIGH)
    except Exception as e:
        print(f"Could not open OAK-D: {type(e).__name__}: {e}", file=sys.stderr)
        if "INSUFFICIENT_PERMISSIONS" in str(e):
            print("  -> another process holds the device; only one dai.Device at a time.",
                  file=sys.stderr)
        return 2

    q_depth = device.getOutputQueue(name="depth", maxSize=30, blocking=False)
    q_imu = device.getOutputQueue(name="imu", maxSize=200, blocking=False)

    print(f"Running {seconds}s at {fps}fps requested. Warming up 3s...")
    time.sleep(3.0)

    frames = 0
    imu_rows = []
    depth_valid = []
    first_depth = None
    t0 = time.time()

    while time.time() - t0 < seconds:
        d = q_depth.tryGet()
        if d is not None:
            frames += 1
            if first_depth is None:
                first_depth = d
            dep = np.frombuffer(d.getData(), dtype=np.uint16)
            valid = dep[dep != 0]
            depth_valid.append((len(valid) / dep.size * 100.0, valid, dep.shape))

        mi = q_imu.tryGet()
        if mi is not None:
            for pk in mi.packets:
                a = pk.acceleroMeter
                g = pk.gyroscope
                rv = pk.rotationVector
                imu_rows.append([
                    pk.acceleroMeter.getTimestampDevice().total_seconds() * 1e6,
                    a.x, a.y, a.z, g.x, g.y, g.z,
                    rv.i, rv.j, rv.k, rv.real, rv.rotationVectorAccuracy,
                ])
        time.sleep(0.001)

    el = time.time() - t0
    device.close()

    # ---------- report ----------
    print("\n================ CAMERA HEALTH ================")
    print("  see phase 1 (mono probe runs separately to keep this phase depth-only)")

    print("\n================ DEPTH ================")
    print(f"  {frames/el:.1f} fps delivered")
    if depth_valid:
        vps = [v[0] for v in depth_valid]
        print(f"  output {first_depth.getWidth()}x{first_depth.getHeight()} RAW16 uint16 (mm)")
        print(f"  valid-pixel ratio  min {min(vps):.1f}%  mean {sum(vps)/len(vps):.1f}%  max {max(vps):.1f}%")
        allv = np.concatenate([v[1] for v in depth_valid if v[1].size])
        if allv.size:
            print(f"  depth mm  min {allv.min()}  p50 {np.percentile(allv,50):.0f}  max {allv.max()}")
            print(f"  depth m   min {allv.min()/1000:.2f}  p50 {np.percentile(allv,50)/1000:.2f}  max {allv.max()/1000:.2f}")
            if allv.max() <= 300:
                print("  (values look like noise, not a real scene)")

    print("\n================ IMU ================")
    if imu_rows:
        arr = np.array(imu_rows)
        rate = (len(arr)-1) / ((arr[-1,0]-arr[0,0])/1e6) if len(arr) > 2 else 0
        print(f"  {rate:.1f} Hz  ({len(arr)} samples)")
        mag = np.sqrt(arr[:,1]**2 + arr[:,2]**2 + arr[:,3]**2)
        print(f"  |accel| {mag.min():.2f}..{mag.max():.2f} m/s^2 (expect ~9.81 if level)")
        gm = np.sqrt(arr[:,4]**2 + arr[:,5]**2 + arr[:,6]**2)
        print(f"  |gyro|  {gm.min():.3f}..{gm.max():.3f} rad/s")
        q = arr[:,7:11]
        qn = np.sqrt((q**2).sum(axis=1))
        print(f"  quat norm {qn.min():.5f}..{qn.max():.5f}")
        uniq = len(set(map(tuple, np.round(q, 5))))
        print(f"  distinct quat values {uniq}/{len(q)}  (low => orientation frozen)")
        acc = arr[:,11]
        print(f"  accuracy_rad first {acc[0]:.3f}  last {acc[-1]:.3f}  min {acc.min():.3f}  max {acc.max():.3f}")
        if uniq < len(q) * 0.25:
            print("  !! orientation nearly static -- fine if board is still;")
            print("     move the board and re-run to confirm fusion updates.")
        if acc.max() - acc.min() < 0.01 and acc[0] > 3.0:
            print("  !! accuracy_rad pinned near pi: fused orientation not converging.")

    if save and imu_rows:
        try:
            import cv2, os
            os.makedirs(outdir, exist_ok=True)
            if first_depth is not None:
                dep = np.frombuffer(first_depth.getData(), dtype=np.uint16).reshape(
                    first_depth.getHeight(), first_depth.getWidth())
                valid = dep[dep != 0]
                if valid.size:
                    lo, hi = np.percentile(valid, 2), np.percentile(valid, 98)
                    scaled = np.clip((dep.astype(np.float32)-lo)/(hi-lo+1e-6), 0, 1)
                    scaled[dep == 0] = 0
                    col = cv2.applyColorMap((scaled*255).astype(np.uint8), cv2.COLORMAP_JET)
                    col[dep == 0] = 0
                    cv2.imwrite(f"{outdir}/depth.png", col)
                    print(f"\n  wrote {outdir}/depth.png")
            with open(f"{outdir}/imu.csv", "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["ts_us","ax","ay","az","gx","gy","gz","qi","qj","qk","qw","accuracy_rad"])
                w.writerows(imu_rows)
            print(f"  wrote {outdir}/imu.csv")
        except Exception as e:
            print(f"  save failed: {e}")

    return 0


def main():
    ap = argparse.ArgumentParser(description="OAK-D stereo depth + IMU health check")
    ap.add_argument("--resolution", default="800p", choices=["400p","720p","800p"])
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--save", action="store_true", help="write depth.png and imu.csv")
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--skip-health", action="store_true",
                    help="skip the separate mono-saturation probe")
    args = ap.parse_args()
    res = {"400p": R.THE_400_P, "720p": R.THE_720_P, "800p": R.THE_800_P}[args.resolution]

    if args.skip_health:
        print("================ CAMERA HEALTH ================")
        print("  skipped (--skip-health)")
        return capture(res, args.fps, args.seconds, args.save, args.outdir)

    print("Phase 1/2: camera health (streams full-res mono; separate device run)")
    try:
        stats = health_check(res, args.fps)
    except Exception as e:
        print(f"  health check failed: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    if stats is None:
        print("  !! no mono frame captured -- camera may be disconnected")
        return 2
    print(f"  left mono mean {stats['mean']:.1f}  min {stats['min']}  "
          f"max {stats['max']}  stdev {stats['stdev']:.1f}")
    print(f"  saturated (>=250) {stats['saturated_pct']:.1f}%")
    healthy = stats["saturated_pct"] < 30.0 and stats["stdev"] > 12.0
    if stats["saturated_pct"] >= 30.0:
        print("  !! CAMERA SATURATED -- depth will be garbage. Check lens caps,")
        print("     IR filter, and scene brightness; auto-exposure can latch high.")
    elif stats["stdev"] <= 12.0:
        print("  !! LOW TEXTURE -- blank wall/desk. Stereo match will be sparse.")

    print("\nPhase 2/2: depth + IMU capture (depth only over XLink)")
    rc = capture(res, args.fps, args.seconds, args.save, args.outdir)
    return rc if rc else (0 if healthy else 1)


if __name__ == "__main__":
    sys.exit(main())