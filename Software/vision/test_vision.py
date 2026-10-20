"""Offline checks for the vision math; none of these touch a camera or the arm.  Run: pytest Software/vision"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from common import (
    JointRange,
    Intrinsics,
    average_T,
    degrees_to_ticks,
    detect,
    detector,
    invert,
    make_T,
    marker_object_points,
    rodrigues_T,
    rotation_about_z,
    ticks_to_degrees,
)
from common import ArmMove, CliArm, HttpArm, JOINTS, arm, camera_source, grab, open_camera, read_joint_ticks
from handeye import residuals, solve_hand_eye
from markers import charuco_board, marker_tile

rng = np.random.default_rng(7)


def random_T(scale: float = 0.3) -> np.ndarray:
    R, _ = cv2.Rodrigues(rng.normal(size=3))
    return make_T(R, rng.normal(size=3) * scale)


def test_transform_algebra():
    T = random_T()
    assert np.allclose(T @ invert(T), np.eye(4), atol=1e-12)
    assert np.allclose(rotation_about_z(np.pi / 2) @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-12)
    Ts = [T @ make_T(cv2.Rodrigues(rng.normal(size=3) * 0.01)[0], rng.normal(size=3) * 0.001) for _ in range(20)]
    assert np.linalg.norm(average_T(Ts)[:3, 3] - T[:3, 3]) < 0.002


def test_servo_units_follow_lerobot():
    r = JointRange(813, 3202)  # the follower's shoulder lift
    assert ticks_to_degrees(r.mid, r) == 0.0
    assert degrees_to_ticks(0.0, r) == round(r.mid)
    assert abs(ticks_to_degrees(r.mid + 4095 / 4, r) - 90.0) < 1e-9
    assert degrees_to_ticks(ticks_to_degrees(1234, r), r) == 1234


def test_marker_round_trip_through_a_synthetic_camera():
    """Generate a marker, detect it, and recover a known pose with solvePnP through made-up intrinsics."""
    K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
    intr = Intrinsics(1280, 720, K, np.zeros(5))
    size = 0.04
    # A marker the camera can read faces it: its z axis points back at the lens, so start from a half-turn about x.
    facing = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0]
    T_true = make_T(facing @ cv2.Rodrigues(np.array([0.2, -0.3, 0.1]))[0], [0.05, -0.02, 0.5])
    # Render: warp the marker image onto the projected corners.
    tile = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 3, 200)
    tile = cv2.copyMakeBorder(tile, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
    obj = marker_object_points(size)
    quiet = obj * 1.5  # the white border's corners, so the warp includes the quiet zone
    img_pts, _ = cv2.projectPoints(quiet, cv2.Rodrigues(T_true[:3, :3])[0], T_true[:3, 3], K, None)
    src = np.float32([[0, 0], [300, 0], [300, 300], [0, 300]])
    H = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    frame = cv2.warpPerspective(tile, H, (1280, 720), borderValue=255)
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    found = detect(frame, detector(), intr, size)
    assert [d.id for d in found] == [3]
    T = found[0].T_cam_marker
    assert np.linalg.norm(T[:3, 3] - T_true[:3, 3]) < 0.003, "position within 3 mm at half a meter"
    assert np.allclose(T[:3, :3], T_true[:3, :3], atol=0.03)


def test_hand_eye_convention_on_synthetic_data():
    """The one that matters: with a known camera placement and marker offset, the solver must give them back."""
    T_base_cam_true = make_T(cv2.Rodrigues(np.array([2.0, 0.3, -0.4]))[0], [0.35, -0.25, 0.45])
    T_gripper_marker_true = make_T(cv2.Rodrigues(np.array([0.1, 1.5, 0.0]))[0], [0.02, 0.0, 0.03])
    T_cam_marker, T_base_gripper = [], []
    for _ in range(8):
        B = make_T(cv2.Rodrigues(rng.normal(size=3) * 0.8)[0], [0.25 + rng.normal() * 0.05, rng.normal() * 0.1, 0.15 + rng.normal() * 0.05])
        A = invert(T_base_cam_true) @ B @ T_gripper_marker_true
        # a little measurement noise
        A = A @ make_T(cv2.Rodrigues(rng.normal(size=3) * 0.003)[0], rng.normal(size=3) * 0.0005)
        T_cam_marker.append(A)
        T_base_gripper.append(B)
    T_base_cam, T_gripper_marker = solve_hand_eye(T_cam_marker, T_base_gripper)
    assert np.linalg.norm(T_base_cam[:3, 3] - T_base_cam_true[:3, 3]) < 0.005
    assert np.allclose(T_base_cam[:3, :3], T_base_cam_true[:3, :3], atol=0.02)
    assert np.linalg.norm(T_gripper_marker[:3, 3] - T_gripper_marker_true[:3, 3]) < 0.005
    assert max(residuals(T_base_cam, T_gripper_marker, T_cam_marker, T_base_gripper)) < 5.0


def test_charuco_board_renders_and_is_detected():
    board = charuco_board()
    img = board.generateImage((1400, 1000), marginSize=60)
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(img)
    assert ids is not None and len(ids) >= 20


def test_marker_tiles_detect_with_their_ids():
    for mid in (0, 5, 10):
        tile = marker_tile(mid, 40.0, "x")
        corners, ids, _ = detector().detectMarkers(tile)
        assert ids is not None and ids.ravel().tolist() == [mid]


def test_kinematics_round_trip_on_the_repo_urdf():
    pytest.importorskip("placo")
    from lerobot.model.kinematics import RobotKinematics
    from common import URDF
    from pick import solve

    kin = RobotKinematics(str(URDF), "gripper_frame_link")
    q_true = np.array([20.0, -35.0, 55.0, 25.0, 10.0, 0.0])
    T = kin.forward_kinematics(q_true)
    q = solve(kin, np.zeros(6), T)
    assert np.linalg.norm(kin.forward_kinematics(q)[:3, 3] - T[:3, 3]) < 0.002


def test_pick_helpers():
    from pick import fold_yaw, within_ranges, DOWN
    import math
    assert abs(fold_yaw(math.radians(100)) - math.radians(10)) < 1e-9
    assert abs(fold_yaw(math.radians(-50)) - math.radians(40)) < 1e-9
    assert abs(fold_yaw(math.radians(44)) - math.radians(44)) < 1e-9
    # DOWN is a proper rotation whose z points down
    assert np.allclose(DOWN @ DOWN.T, np.eye(3)) and np.isclose(np.linalg.det(DOWN), 1.0) and DOWN[2, 2] == -1
    ranges = {j: JointRange(800, 3300) for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
    assert within_ranges(np.zeros(6), ranges) == []
    assert within_ranges(np.array([0, 0, 0, 120.0, 0, 0]), ranges) == ["wrist_flex=120°"]


def test_hand_eye_orientation_spread_and_tilt_skew():
    from handeye import orientation_spread_deg
    from calibrate_camera import skew_deg
    I = np.eye(4)
    R45 = make_T(cv2.Rodrigues(np.array([0, np.radians(45), 0]))[0], [0, 0, 0])
    assert orientation_spread_deg([I, I]) < 1e-6
    assert abs(orientation_spread_deg([I, R45, I]) - 45) < 1e-6
    square = np.array([[100, 100], [500, 100], [500, 400], [100, 400]], dtype=float)
    assert skew_deg(square) < 1e-6
    trapezoid = np.array([[150, 100], [450, 100], [500, 400], [100, 400]], dtype=float)
    assert skew_deg(trapezoid) > 10


# ---------------------------------------------------------------- the remote arm and camera
def _stub_pi():
    """Enough of `so101 serve` and ustreamer to exercise the bridges: answers like the real ones, logs every request."""
    import http.server
    import threading

    log = []
    jpeg = cv2.imencode(".jpg", np.full((36, 64, 3), 128, np.uint8))[1].tobytes()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("content-type", ctype)
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            log.append(("GET", self.path, None))
            if self.path == "/limits":
                self._send(200, {"1": {"min": 818, "max": 3487}, "6": {"min": 1995, "max": 3525}})
            elif self.path == "/status":
                self._send(200, {"joints": [{"id": i, "position": 2000 + i} for i in range(1, 6)] + [{"id": 6, "position": None}]})
            elif self.path == "/poses":
                self._send(200, {"home": {"ids": [1, 2, 3, 4, 5], "positions": [1, 2, 3, 4, 5]}})
            elif self.path == "/snapshot":
                self._send(200, jpeg, "image/jpeg")
            elif self.path == "/html":
                self._send(502, b"<html>Bad Gateway</html>", "text/html")
            else:
                self._send(404, {"error": "no such route"})

        def _body(self):
            return json.loads(self.rfile.read(int(self.headers["content-length"])))

        def do_POST(self):
            body = self._body()
            log.append(("POST", self.path, body))
            if self.path == "/pose/go":
                self._send(200, {"reached": True}) if body["name"] == "home" else self._send(404, {"error": 'no pose "x"'})
            elif self.path == "/move":
                if "6" in body["targets"]:
                    self._send(409, {"reached": False, "error": "gripper jammed; stopped everything where it is", "stalled": [6]})
                else:
                    self._send(200, {"reached": True, "final": body["targets"]})
            else:
                self._send(200, {})

        def do_PUT(self):
            body = self._body()
            log.append(("PUT", self.path, body))
            self._send(200, {"stored": self.path.rsplit("/", 1)[1]})

    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_address[1]}", log


def test_camera_source_is_an_index_or_a_stream_url():
    assert camera_source("1") == 1
    assert camera_source("http://pi:8080/stream") == "http://pi:8080/stream"


def test_arm_selection_follows_the_environment(monkeypatch):
    monkeypatch.delenv("SO101_ARM", raising=False)
    assert isinstance(arm(), CliArm)
    monkeypatch.setenv("SO101_ARM", "http://pi:8101/")
    remote = arm()
    assert isinstance(remote, HttpArm) and remote.base == "http://pi:8101"


def test_http_arm_speaks_the_serve_api():
    server, base, log = _stub_pi()
    try:
        a = HttpArm(base)
        assert a.limits() == {1: (818, 3487), 6: (1995, 3525)}
        assert a.positions() == {1: 2001, 2: 2002, 3: 2003, 4: 2004, 5: 2005}
        with pytest.raises(SystemExit, match="gripper"):
            read_joint_ticks(a)  # the gripper gave no position
        assert a.poses() == {"home": {"ids": [1, 2, 3, 4, 5], "positions": [1, 2, 3, 4, 5]}}
        assert a.go_pose("home", speed=150) == (True, "reached home")
        ok, msg = a.go_pose("x")
        assert not ok and "no pose" in msg
        assert a.move({1: 2100}, speed=200, torque=70) == (True, "reached")
        ok, msg = a.move({6: 2000})
        assert not ok and "jammed" in msg  # the pick reads a jam as a grasp
        a.grip(2010, torque=30)
        a.torque(False)
        a.publish_calibration("camera", {"rms": 0.3})
        assert ("POST", "/pose/go", {"name": "home", "speed": 150, "release": False}) in log
        assert ("POST", "/move", {"targets": {"1": 2100}, "speed": 200, "torque": 70, "release": False}) in log
        assert ("POST", "/grip", {"position": 2010, "torque": 30}) in log
        assert ("POST", "/torque", {"on": False}) in log
        assert ("PUT", "/calibration/camera", {"rms": 0.3}) in log

        ranges = {j: JointRange(800, 3300) for j in JOINTS}
        mover = ArmMove(ranges, a, speed=200, torque=70)
        ok, _ = mover.go("step", {"shoulder_pan": 0.0})
        assert ok
        assert log[-1] == ("POST", "/move", {"targets": {"1": 2050}, "speed": 200, "torque": 70, "release": False})
        with pytest.raises(SystemExit, match="outside"):
            mover.go("bad", {"shoulder_pan": 200.0})
    finally:
        server.shutdown()


def test_http_camera_reads_snapshots_from_ustreamer():
    server, base, _ = _stub_pi()
    try:
        cap = open_camera(base + "/stream", 64, 36)
        assert grab(cap).shape == (36, 64, 3)
        with pytest.raises(SystemExit, match="--resolution=1280x720"):
            open_camera(base + "/stream", 1280, 720)
    finally:
        server.shutdown()


def test_http_camera_normalizes_the_stream_url():
    from common import HttpCamera

    for url in ("http://pi:8080/stream", "http://pi:8080/stream/", "http://pi:8080/stream?x=1", "http://pi:8080/snapshot"):
        assert HttpCamera(url).url == "http://pi:8080/snapshot", url


def test_http_arm_turns_a_non_json_reply_into_an_error():
    server, base, _ = _stub_pi()
    try:
        code, d = HttpArm(base)._call("GET", "/html")
        assert code == 502 and d["error"].startswith("not a JSON reply: <html>")
    finally:
        server.shutdown()
