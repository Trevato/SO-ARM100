"""Where is the camera relative to the arm? Solved by looking at a marker taped to the gripper from several arm poses.

Save 6-10 poses by hand first (`so101 pose save he1` ... with the gripper marker facing the camera, spread around the
workspace), then:

    python Software/vision/handeye.py --camera 1 --poses he1,he2,he3,he4,he5,he6 --out outputs/handeye.json

The arm visits each pose by itself. Nothing else to do but keep hands clear.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from common import (
    GRIPPER_ID,
    JOINTS,
    POSES,
    URDF,
    Intrinsics,
    detect,
    detector,
    grab,
    invert,
    last_line,
    load_joint_ranges,
    make_T,
    median_T,
    open_camera,
    read_joint_degrees,
    servo_ranges_match,
    arm,
    camera_source,
)

GRIPPER_MM = 25.0


def solve_hand_eye(T_cam_marker: list[np.ndarray], T_base_gripper: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Fixed camera, marker on the gripper. Returns (T_base_cam, T_gripper_marker).

    OpenCV's calibrateRobotWorldHandEye solves T_cam'_world' . T_world'_base' = T_cam'_gripper' . T_gripper'_base'.
    Our relation T_marker_cam . T_cam_base = T_marker_gripper . T_gripper_base has that shape with
    cam' = marker, world' = cam, base' = base, gripper' = gripper.
    """
    R_w2c, t_w2c, R_b2g, t_b2g = [], [], [], []
    for A, B in zip(T_cam_marker, T_base_gripper):
        Ai, Bi = invert(A), invert(B)  # T_marker_cam, T_gripper_base
        R_w2c.append(Ai[:3, :3]); t_w2c.append(Ai[:3, 3])
        R_b2g.append(Bi[:3, :3]); t_b2g.append(Bi[:3, 3])
    R_b2w, t_b2w, R_g2c, t_g2c = cv2.calibrateRobotWorldHandEye(R_w2c, t_w2c, R_b2g, t_b2g)
    T_cam_base = make_T(R_b2w, t_b2w)      # world'_base' = cam_base
    T_marker_gripper = make_T(R_g2c, t_g2c)  # cam'_gripper' = marker_gripper
    return invert(T_cam_base), invert(T_marker_gripper)


def residuals(T_base_cam, T_gripper_marker, T_cam_marker, T_base_gripper) -> list[float]:
    """Per pose: distance in mm between the marker as the camera saw it and where the arm says it was."""
    out = []
    for A, B in zip(T_cam_marker, T_base_gripper):
        seen = (T_base_cam @ A)[:3, 3]
        told = (B @ T_gripper_marker)[:3, 3]
        out.append(float(np.linalg.norm(seen - told) * 1000))
    return out


def orientation_spread_deg(Ts: list[np.ndarray]) -> float:
    """Largest rotation angle between any two gripper orientations; the solver needs variety here, not just positions."""
    worst = 0.0
    for i in range(len(Ts)):
        for j in range(i + 1, len(Ts)):
            R = Ts[i][:3, :3].T @ Ts[j][:3, :3]
            worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))))
    return worst


def park(arm) -> None:
    """Leave the arm somewhere safe before letting go of it."""
    if "home" in arm.poses():
        ok, msg = arm.go_pose("home", speed=200, release=True)
        print(f"home: {last_line(msg)}")
    else:
        print("no `home` pose saved; leaving torque on so the arm does not drop (so101 torque off when it is supported)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=camera_source, default=1, help="an OpenCV index, or the ustreamer URL on the Pi")
    ap.add_argument("--intrinsics", type=Path, default=Path("outputs/camera.json"))
    ap.add_argument("--poses", required=True, help="comma-separated pose names saved with `so101 pose save`")
    ap.add_argument("--frames", type=int, default=15, help="frames averaged per pose")
    ap.add_argument("--poses-file", type=Path, default=POSES, help="where the he* poses were saved (a remote arm keeps its own)")
    ap.add_argument("--out", type=Path, default=Path("outputs/handeye.json"))
    ap.add_argument("--publish-as", default="handeye", help="name on the arm's server: handeye for camera 1, handeye2 for camera 2")
    args = ap.parse_args()

    from lerobot.model.kinematics import RobotKinematics

    intr = Intrinsics.load(args.intrinsics)
    ranges = load_joint_ranges()
    the_arm = arm(args.poses_file)
    mismatch = servo_ranges_match(ranges, the_arm)
    if mismatch:
        raise SystemExit("the servos' limits differ from the calibration file; re-run `just calibrate`:\n  " + "\n  ".join(mismatch))
    kin = RobotKinematics(str(URDF), "gripper_frame_link")
    det = detector()
    cap = open_camera(args.camera, intr.width, intr.height)
    print(f"using {the_arm}")

    T_cam_marker, T_base_gripper, used = [], [], []
    for name in args.poses.split(","):
        print(f"going to {name}…", end=" ", flush=True)
        ok, msg = the_arm.go_pose(name, speed=200)
        if not ok:
            print(f"\n  could not reach {name}: {last_line(msg)}")
            continue
        time.sleep(0.8)
        seen = []
        for _ in range(args.frames):
            for d in detect(grab(cap), det, intr, {GRIPPER_ID: GRIPPER_MM / 1000}):
                if d.id == GRIPPER_ID and d.T_cam_marker is not None and d.ambiguity > 1.5:
                    seen.append(d.T_cam_marker)
        if len(seen) < args.frames // 2:
            print(f"gripper marker seen clearly in only {len(seen)}/{args.frames} frames, skipping this pose")
            continue
        q = read_joint_degrees(ranges, the_arm)
        T_cam_marker.append(median_T(seen))
        T_base_gripper.append(kin.forward_kinematics(q))
        used.append(name)
        print(f"marker at {np.round(T_cam_marker[-1][:3, 3] * 1000).astype(int)} mm in camera, gripper at {np.round(T_base_gripper[-1][:3, 3] * 1000).astype(int)} mm in base")
    cap.release()
    park(the_arm)

    if len(used) < 4:
        raise SystemExit(f"only {len(used)} usable poses; need at least 4, ideally 6-10 with varied orientations")
    spread = orientation_spread_deg(T_base_gripper)
    print(f"orientation spread across poses: {spread:.0f}°")
    if spread < 30:
        raise SystemExit("the gripper pointed almost the same way in every pose; the solver cannot separate camera position from marker offset. Save poses that tilt and roll the gripper differently (aim for 60° or more of spread) and run again")
    T_base_cam, T_gripper_marker = solve_hand_eye(T_cam_marker, T_base_gripper)
    res = residuals(T_base_cam, T_gripper_marker, T_cam_marker, T_base_gripper)
    cam = T_base_cam[:3, 3] * 1000
    print(f"camera sits at x={cam[0]:.0f} y={cam[1]:.0f} z={cam[2]:.0f} mm from the arm base")
    print(f"residuals per pose: {', '.join(f'{r:.1f}' for r in res)} mm (mean {np.mean(res):.1f}, worst {max(res):.1f})")
    if np.mean(res) > 12 or max(res) > 25:
        raise SystemExit("not saved: residuals too large. Under ~8 mm mean is what a correct model gives; this size means a joint direction disagrees between servo and model, or the camera moved. Compare `just vision-view` base coordinates of the gripper marker with a ruler at one held pose")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    result = {"T_base_cam": T_base_cam.tolist(), "T_gripper_marker": T_gripper_marker.tolist(), "poses": used, "residuals_mm": res, "orientation_spread_deg": spread}
    args.out.write_text(json.dumps(result, indent=1))
    print(f"saved {args.out}")
    the_arm.publish_calibration(args.publish_as, result)


if __name__ == "__main__":
    main()
