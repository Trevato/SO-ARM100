"""Find the marked cube, pick it up, put it at the saved `drop` pose.

    python Software/vision/pick.py --camera 1 [--dry-run] [--cube-mm 40]

Needs outputs/camera.json, outputs/handeye.json, and two poses saved from the repo root with the arm only
(`so101 pose save drop --ids 1-5` over the bin, `so101 pose save home --ids 1-5` at rest), plus a cube wearing
the 40 mm markers in the camera's view. Every move is a supervised `so101 pose go`; a jam stops everything.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from common import (
    CUBE_IDS,
    JOINTS,
    POSES,
    URDF,
    ArmMove,
    Intrinsics,
    degrees_to_ticks,
    detect,
    detector,
    grab,
    last_line,
    load_joint_ranges,
    make_T,
    median_T,
    open_camera,
    read_joint_degrees,
    read_joint_ticks,
    rotation_about_z,
    servo_ranges_match,
    arm,
    camera_source,
    ticks_to_degrees,
)

MARKER_MM = 40.0
APPROACH_HEIGHTS_M = (0.03, 0.025, 0.02)  # tried in order: a vertical gripper cannot reach much higher than 30 mm over the table
GRASP_DEPTH_M = 0.005   # how far below the cube's top face the jaws aim
IK_ITERATIONS = 40
IK_TOLERANCE_MM = 8
SQUEEZE_TORQUE = 30     # percent, for holding the cube after the jaws meet it

# The arm's natural "pointing down" orientation at pan 0: frame x = -X, y = +Y, z = -Z of the base.
# Any other roll costs wrist_roll range it does not have.
DOWN = np.diag([-1.0, 1.0, -1.0])

# A mid-workspace grasp solution; a good second guess when the current pose seeds the solver into a corner.
READY = np.array([0.0, -30.0, 47.0, 73.0, 0.0, 0.0])


def find_cube(cap, det, intr, marker_m: float, cube_m: float) -> tuple[np.ndarray, int] | None:
    """Cube center pose in the camera frame from the face that looks most squarely at the camera."""
    seen: dict[int, list[np.ndarray]] = {}
    facing: dict[int, list[float]] = {}
    for _ in range(12):
        for d in detect(grab(cap), det, intr, {i: marker_m for i in CUBE_IDS}):
            if d.id in CUBE_IDS and d.T_cam_marker is not None and d.ambiguity > 1.5:
                seen.setdefault(d.id, []).append(d.T_cam_marker)
                facing.setdefault(d.id, []).append(-d.T_cam_marker[2, 2])  # marker z against camera z: 1 = square-on
    usable = {i: p for i, p in seen.items() if len(p) >= 5}
    if not usable:
        return None
    face_id = max(usable, key=lambda i: float(np.mean(facing[i])))
    T_cam_face = median_T(usable[face_id])
    # The face marker's z points out of the cube; the center is half a cube behind it.
    return T_cam_face @ make_T(np.eye(3), [0, 0, -cube_m / 2]), face_id


def within_ranges(q: np.ndarray, ranges) -> list[str]:
    """Joints whose IK angle falls outside the calibrated servo range (the URDF's limits are wider than the servo's)."""
    bad = []
    for i, joint in enumerate(JOINTS[:5]):
        t = degrees_to_ticks(float(q[i]), ranges[joint])
        if not (ranges[joint].min + 30 <= t <= ranges[joint].max - 30):
            bad.append(f"{joint}={q[i]:.0f}°")
    return bad


def solve(kin, q_start: np.ndarray, T_target: np.ndarray, ranges=None) -> np.ndarray:
    """Iterate LeRobot's one-step IK to convergence from a few seeds; raises if the target is out of reach."""
    best, best_err = None, float("inf")
    for seed in (q_start, READY, np.zeros(6)):
        q = np.array(seed, dtype=float)
        for _ in range(IK_ITERATIONS):
            q = kin.inverse_kinematics(q, T_target, position_weight=1.0, orientation_weight=0.05)
        err = np.linalg.norm(kin.forward_kinematics(q)[:3, 3] - T_target[:3, 3]) * 1000
        if err <= IK_TOLERANCE_MM and (ranges is None or not within_ranges(q, ranges)):
            return q
        if err < best_err:
            best, best_err = q, err
    if best_err > IK_TOLERANCE_MM:
        raise ValueError(f"off by {best_err:.0f} mm")
    return best


def fold_yaw(yaw: float) -> float:
    """A cube looks the same every 90°; pick the equivalent yaw closest to zero so the wrist barely rolls."""
    return (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=camera_source, default=1, help="an OpenCV index, or the ustreamer URL on the Pi")
    ap.add_argument("--intrinsics", type=Path, default=Path("outputs/camera.json"))
    ap.add_argument("--handeye", type=Path, default=Path("outputs/handeye.json"))
    ap.add_argument("--poses-file", type=Path, default=POSES, help="where `drop` and `home` were saved")
    ap.add_argument("--cube-mm", type=float, default=40.0, help="width of the object the markers are on")
    ap.add_argument("--dry-run", action="store_true", help="see the cube and plan every move, move nothing")
    args = ap.parse_args()

    from lerobot.model.kinematics import RobotKinematics

    the_arm = arm(args.poses_file)
    poses = the_arm.poses()
    for needed in ("drop", "home"):
        if needed not in poses:
            raise SystemExit(f"no `{needed}` pose on {the_arm}: save it from the arm's web page without the gripper, or `so101 pose save {needed} --ids 1-5`")
        if 6 in poses[needed]["ids"]:
            raise SystemExit(f"`{needed}` was saved with the gripper; save it again without the gripper (web page) or `so101 pose save {needed} --ids 1-5`")
    intr = Intrinsics.load(args.intrinsics)
    handeye = json.loads(args.handeye.read_text())
    T_base_cam = np.array(handeye["T_base_cam"])
    ranges = load_joint_ranges()
    mismatch = servo_ranges_match(ranges, the_arm)
    if mismatch:
        raise SystemExit("the servos' limits differ from the calibration file; re-run `just calibrate`:\n  " + "\n  ".join(mismatch))
    kin = RobotKinematics(str(URDF), "gripper_frame_link")
    det = detector()
    cube_m = args.cube_mm / 1000
    jaw_offset_m = cube_m / 2  # the gripper frame sits on the fixed jaw's inner face; the cube is centered half a cube toward -x

    cap = open_camera(args.camera, intr.width, intr.height)
    found = find_cube(cap, det, intr, MARKER_MM / 1000, cube_m)
    cap.release()
    if found is None:
        raise SystemExit("no cube marker seen clearly; check `just vision-view`")
    T_cam_cube, face = found
    T_base_cube = T_base_cam @ T_cam_cube
    c = T_base_cube[:3, 3]
    yaw = fold_yaw(math.atan2(T_base_cube[1, 0], T_base_cube[0, 0]))
    print(f"cube (face {face}) at x={c[0]*1000:.0f} y={c[1]*1000:.0f} z={c[2]*1000:.0f} mm from the base; jaws will roll {math.degrees(yaw):.0f}°")
    if not (-0.02 < c[2] < 0.12):
        raise SystemExit(f"a cube on the table cannot be {c[2]*1000:.0f} mm above the base; the hand-eye calibration is off")

    R = rotation_about_z(yaw) @ DOWN
    frame_at_cube = c + R[:, 0] * jaw_offset_m
    T_grasp = make_T(R, frame_at_cube + [0, 0, cube_m / 2 - GRASP_DEPTH_M])
    q_now = read_joint_degrees(ranges)
    try:
        q_grasp = solve(kin, q_now, T_grasp, ranges)
    except ValueError as e:
        raise SystemExit(f"the grasp pose is out of reach ({e}); move the cube into the zone in QUEST.md") from None
    if bad := within_ranges(q_grasp, ranges):
        raise SystemExit(f"grasp needs {', '.join(bad)}, outside the calibrated range; move the cube closer")
    q_pre, approach = None, None
    for h in APPROACH_HEIGHTS_M:
        try:
            q = solve(kin, q_grasp, make_T(R, T_grasp[:3, 3] + [0, 0, h]), ranges)
        except ValueError:
            continue
        if not within_ranges(q, ranges):
            q_pre, approach = q, h
            break
    if q_pre is None:
        raise SystemExit("no reachable approach pose above the cube; move the cube closer to the arm")

    def arm_joints(q: np.ndarray) -> dict[str, float]:
        return {j: float(q[i]) for i, j in enumerate(JOINTS[:5])}

    def saved_joints(name: str) -> dict[str, float]:
        p = poses[name]
        return {JOINTS[sid - 1]: ticks_to_degrees(t, ranges[JOINTS[sid - 1]]) for sid, t in zip(p["ids"], p["positions"])}

    g = ranges["gripper"]
    gripper_open, gripper_closed = g.max - 60, g.min + 40
    plan = [
        ("open", {"gripper": ticks_to_degrees(gripper_open, g)}),
        ("pre-grasp", arm_joints(q_pre)),
        ("grasp", arm_joints(q_grasp)),
        ("close", {"gripper": ticks_to_degrees(gripper_closed, g)}),
        ("lift", arm_joints(q_pre)),
        ("drop", saved_joints("drop")),
        ("release", {"gripper": ticks_to_degrees(gripper_open, g)}),
        ("home", saved_joints("home")),
    ]
    print(f"plan (approach from {approach*1000:.0f} mm above the grasp):")
    for label, joints in plan:
        print(f"  {label:9s} " + " ".join(f"{j}={v:.0f}°" for j, v in joints.items()))
    mover = ArmMove(ranges, the_arm, speed=200, torque=70)
    for label, joints in plan:
        mover.validate(joints)  # every step within the calibrated ranges, before anything moves
    if args.dry_run:
        print("dry run: nothing moved")
        return

    def step(label: str, joints: dict[str, float]) -> None:
        ok, msg = mover.go(label, joints)
        print(f"{label}: {last_line(msg)}")
        if not ok:
            raise SystemExit(f"stopped at {label}; the arm is holding where it is (so101 torque off to release)")

    step("open", plan[0][1])
    step("pre-grasp", plan[1][1])
    step("grasp", plan[2][1])

    ok, msg = mover.go("close", plan[3][1])
    print(f"close: {last_line(msg)}")
    ticks = read_joint_ticks(the_arm)["gripper"]
    grasped = (not ok) and "jammed" in msg and gripper_closed + 60 < ticks < gripper_open - 60
    if ok or not grasped:
        why = "the jaws closed on nothing" if ok else f"the close stopped for another reason at {ticks} ticks"
        raise SystemExit(f"no grasp: {why}; holding at the grasp pose (so101 torque off to release)")
    print(f"grasped: jaws met the cube at {ticks} ticks")
    # A jam parks the goal at the present position, which holds the cube with almost no force. Squeeze gently
    # toward closed at a low torque cap; the servo's own overload protection then holds at a steady grip.
    the_arm.grip(gripper_closed, torque=SQUEEZE_TORQUE)

    step("lift", plan[4][1])
    step("drop", plan[5][1])
    step("release", plan[6][1])
    step("home", plan[7][1])
    the_arm.torque(False)
    print("done")


if __name__ == "__main__":
    main()
