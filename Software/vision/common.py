"""Shared pieces for the ArUco pick: camera, markers, poses, servo units, and the bridge to the so101 CLI.

Frames: every 4x4 matrix here is T_a_b, "pose of b expressed in a", so T_a_b @ p_b = p_a.
Units: meters and radians inside, millimeters and degrees at the edges where humans look.
"""

from __future__ import annotations

import json
import os
import os
import subprocess
import urllib.error
import urllib.parse
import urllib.request
import tempfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "Simulation/SO101/so101_new_calib.urdf"
POSES = REPO / "poses.json"  # where `so101 pose save` writes when run from the repo root
DICTIONARY = cv2.aruco.DICT_4X4_50
CUBE_IDS = (0, 1, 2, 3, 4, 5)
GRIPPER_ID = 10
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
SERVO_IDS = (1, 2, 3, 4, 5, 6)
RESOLUTION = 4095  # LeRobot's max_res for the STS3215's degrees normalization


# ---------------------------------------------------------------- transforms
def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).ravel()
    return T


def invert(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rodrigues_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
    return make_T(R, tvec)


def rotation_about_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def average_T(Ts: list[np.ndarray]) -> np.ndarray:
    """Mean of nearby poses: translation averaged, rotation via SVD projection of the mean matrix."""
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    M = np.mean([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return make_T(R, t)


# ---------------------------------------------------------------- servo units
@dataclass(frozen=True)
class JointRange:
    min: int
    max: int

    @property
    def mid(self) -> float:
        return (self.min + self.max) / 2


def load_joint_ranges(calibration_json: Path | None = None) -> dict[str, JointRange]:
    """The per-joint min/max LeRobot calibrated; degrees are measured from the middle of each range."""
    path = calibration_json or (
        Path(os.environ.get("HF_LEROBOT_CALIBRATION", Path.home() / ".cache/huggingface/lerobot/calibration"))
        / "robots/so_follower/so101_follower.json"
    )
    data = json.loads(path.read_text())
    return {name: JointRange(int(data[name]["range_min"]), int(data[name]["range_max"])) for name in JOINTS}


def ticks_to_degrees(ticks: float, rng: JointRange) -> float:
    return (ticks - rng.mid) * 360.0 / RESOLUTION


def degrees_to_ticks(deg: float, rng: JointRange) -> int:
    return int(round(deg * RESOLUTION / 360.0 + rng.mid))


# ---------------------------------------------------------------- camera
@dataclass
class Intrinsics:
    width: int
    height: int
    K: np.ndarray
    dist: np.ndarray
    rms: float = 0.0

    @staticmethod
    def load(path: Path) -> "Intrinsics":
        d = json.loads(path.read_text())
        return Intrinsics(d["width"], d["height"], np.array(d["K"]), np.array(d["dist"]), d.get("rms", 0.0))

    def save(self, path: Path) -> None:
        path.write_text(
            json.dumps(
                {"width": self.width, "height": self.height, "K": self.K.tolist(), "dist": self.dist.ravel().tolist(), "rms": self.rms},
                indent=1,
            )
        )


def camera_source(text: str) -> int | str:
    """`--camera 1` is an OpenCV index; `--camera http://pi:8080/stream` is the ustreamer on the Pi."""
    return int(text) if text.isdigit() else text


class HttpCamera:
    """Frames from ustreamer: each read fetches /snapshot, the newest frame, so there is no buffered lag.

    Quacks like cv2.VideoCapture for the few calls the kit makes."""

    def __init__(self, url: str):
        # http://pi:8080/stream, with or without a trailing slash or a query, becomes http://pi:8080/snapshot;
        # the endless MJPEG stream itself would never finish a read.
        parts = urllib.parse.urlsplit(url)
        path = parts.path.rstrip("/")
        if path.endswith("/stream"):
            path = path[: -len("/stream")] + "/snapshot"
        self.url = urllib.parse.urlunsplit((parts.scheme, parts.netloc, path or "/snapshot", "", ""))

    def isOpened(self) -> bool:
        return True

    def set(self, *_: object) -> bool:
        return False  # the resolution is ustreamer's --resolution

    def grab(self) -> bool:
        return True

    def read(self) -> tuple[bool, np.ndarray | None]:
        try:
            with urllib.request.urlopen(self.url, timeout=5) as r:
                data = r.read()
        except (urllib.error.URLError, TimeoutError, OSError):
            return False, None
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        return frame is not None, frame

    def release(self) -> None:
        pass


def open_camera(source: int | str, width: int, height: int) -> cv2.VideoCapture | HttpCamera:
    cap = HttpCamera(source) if isinstance(source, str) else cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f"camera {source} did not open; `lerobot-find-cameras opencv` lists what exists")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):  # let exposure settle and drop stale frames
        cap.read()
    ok, frame = cap.read()
    if not ok or frame is None:
        raise SystemExit(f"camera {source} opened but returned no frame")
    if frame.shape[1] != width or frame.shape[0] != height:
        fix = f"; restart ustreamer with --resolution={width}x{height}" if isinstance(source, str) else ""
        raise SystemExit(f"camera {source} gives {frame.shape[1]}x{frame.shape[0]}, not the {width}x{height} it was calibrated at{fix}")
    return cap


def grab(cap: cv2.VideoCapture | HttpCamera) -> np.ndarray:
    cap.grab()  # discard the buffered frame so the next one is current
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("camera stopped delivering frames")
    return frame


# ---------------------------------------------------------------- markers
def detector() -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICTIONARY), params)


def marker_object_points(size_m: float) -> np.ndarray:
    """Marker corners in the marker's own frame: z out of the face, origin at the center, corner order as OpenCV detects it."""
    h = size_m / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


@dataclass
class Detection:
    id: int
    corners: np.ndarray  # 4x2 pixels
    T_cam_marker: np.ndarray | None = None  # set when intrinsics and the marker's size are known
    reprojection_px: float = float("nan")
    # how clearly the planar pose was decided: second-best over best reprojection error (bigger is clearer)
    ambiguity: float = float("nan")


def detect(
    frame_bgr: np.ndarray,
    det: cv2.aruco.ArucoDetector,
    intr: Intrinsics | None = None,
    sizes_m: dict[int, float] | float | None = None,
) -> list[Detection]:
    """Find markers; with intrinsics and sizes (per id, or one for all) also estimate each pose.

    IPPE_SQUARE returns two candidate poses for a planar marker; the one that reprojects better wins, and
    `ambiguity` records how clearly, so callers can drop frames where a near-frontal marker could flip.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    found: list[Detection] = []
    if ids is None:
        return found
    for c, i in zip(corners, ids.ravel()):
        d = Detection(int(i), c.reshape(4, 2))
        size = sizes_m.get(d.id) if isinstance(sizes_m, dict) else sizes_m
        if intr is not None and size is not None:
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                marker_object_points(size), d.corners.astype(np.float32), intr.K, intr.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if n:
                errs = np.asarray(errs).ravel()
                best = int(np.argmin(errs))
                d.T_cam_marker = rodrigues_T(rvecs[best], tvecs[best])
                d.reprojection_px = float(errs[best])
                d.ambiguity = float(np.sort(errs)[1] / max(errs[best], 1e-6)) if n > 1 else float("inf")
        found.append(d)
    return found


def median_T(Ts: list[np.ndarray]) -> np.ndarray:
    """Translation by coordinate-wise median (robust to a few bad frames), rotation by the SVD mean."""
    T = average_T(Ts)
    T[:3, 3] = np.median([t[:3, 3] for t in Ts], axis=0)
    return T


# ---------------------------------------------------------------- the arm
# Two ways to reach it, with the same supervision underneath: the so101 CLI on this machine (SO101_PORT),
# or `so101 serve` on the Pi (SO101_ARM=http://pi:8101). Every move is a supervised pose move; the CLI
# and the server own the serial port, the limit margins, and the jam detection.


def so101(*args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    """Run the TypeScript CLI."""
    try:
        r = subprocess.run(["so101", *args], text=True, capture_output=True)
    except FileNotFoundError:
        raise SystemExit("`so101` is not on PATH; run from the repo's direnv shell") from None
    if check and r.returncode != 0:
        raise SystemExit(f"so101 {' '.join(args)} failed: {last_line(r.stdout + r.stderr)}")
    return r


def last_line(text: str) -> str:
    lines = [ln for ln in text.strip().splitlines() if ln.strip()]
    return lines[-1] if lines else "(no output)"


class CliArm:
    """The arm on this machine, through the so101 CLI."""

    def __init__(self, poses_file: Path = POSES):
        self.poses_file = poses_file
        self._dir = tempfile.TemporaryDirectory()
        self._scratch = Path(self._dir.name) / "poses.json"

    def __str__(self) -> str:
        return "the arm on this machine (so101 CLI)"

    def limits(self) -> dict[int, tuple[int, int]]:
        out = {}
        for sid in SERVO_IDS:
            lo = int(so101("read", str(sid), "Min_Position_Limit").stdout.split("=")[1].split()[0])
            hi = int(so101("read", str(sid), "Max_Position_Limit").stdout.split("=")[1].split()[0])
            out[sid] = (lo, hi)
        return out

    def positions(self) -> dict[int, int]:
        so101("pose", "save", "now", "--file", str(self._scratch))
        pose = json.loads(self._scratch.read_text())["now"]
        return {int(sid): int(p) for sid, p in zip(pose["ids"], pose["positions"])}

    def poses(self) -> dict:
        return json.loads(self.poses_file.read_text()) if self.poses_file.exists() else {}

    def go_pose(self, name: str, *, speed: int = 200, release: bool = False) -> tuple[bool, str]:
        args = ["pose", "go", name, "--file", str(self.poses_file), "--speed", str(speed)] + (["--release"] if release else [])
        r = so101(*args, check=False)
        return r.returncode == 0, (r.stdout + r.stderr).strip()

    def move(self, targets: dict[int, int], *, speed: int = 250, torque: int = 70, release: bool = False) -> tuple[bool, str]:
        self._scratch.write_text(json.dumps({"step": {"ids": list(targets), "positions": list(targets.values())}}))
        args = ["pose", "go", "step", "--file", str(self._scratch), "--speed", str(speed), "--torque", str(torque)] + (["--release"] if release else [])
        r = so101(*args, check=False)
        return r.returncode == 0, (r.stdout + r.stderr).strip()

    def grip(self, position: int, *, torque: int = 30) -> None:
        so101("move", "6", str(position), "--speed", "300", "--torque", str(torque), "--hold")

    def torque(self, on: bool) -> None:
        so101("torque", "on" if on else "off", check=False)

    def publish_calibration(self, kind: str, data: dict) -> None:
        pass  # the files already live here


class HttpArm:
    """The arm behind `so101 serve`, usually on the Pi. Same API as CliArm."""

    def __init__(self, base: str):
        self.base = base.rstrip("/")

    def __str__(self) -> str:
        return f"the arm at {self.base}"

    def _call(self, method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data, method=method, headers={"content-type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                return r.status, self._json(r.read())
        except urllib.error.HTTPError as e:
            return e.code, self._json(e.read())  # the server answers errors as JSON
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise SystemExit(f"{self.base}: {getattr(e, 'reason', e)}; is `so101 serve` running there?") from None

    @staticmethod
    def _json(raw: bytes) -> dict:
        """The server's JSON, or a proxy's or a restarting Pi's HTML turned into an error the callers can show."""
        try:
            data = json.loads(raw or b"{}")
        except ValueError:
            return {"error": "not a JSON reply: " + raw[:200].decode("utf-8", "replace").strip()}
        return data if isinstance(data, dict) else {"error": f"unexpected reply: {data!r}"[:200]}

    def limits(self) -> dict[int, tuple[int, int]]:
        _, d = self._call("GET", "/limits")
        return {int(k): (v["min"], v["max"]) for k, v in d.items()}

    def positions(self) -> dict[int, int]:
        _, d = self._call("GET", "/status")
        return {j["id"]: j["position"] for j in d["joints"] if j["position"] is not None}

    def poses(self) -> dict:
        return self._call("GET", "/poses")[1]

    def go_pose(self, name: str, *, speed: int = 200, release: bool = False) -> tuple[bool, str]:
        code, d = self._call("POST", "/pose/go", {"name": name, "speed": speed, "release": release})
        return code == 200 and bool(d.get("reached")), d.get("error") or f"reached {name}"

    def move(self, targets: dict[int, int], *, speed: int = 250, torque: int = 70, release: bool = False) -> tuple[bool, str]:
        body = {"targets": {str(k): v for k, v in targets.items()}, "speed": speed, "torque": torque, "release": release}
        code, d = self._call("POST", "/move", body)
        return code == 200 and bool(d.get("reached")), d.get("error") or "reached"

    def grip(self, position: int, *, torque: int = 30) -> None:
        code, d = self._call("POST", "/grip", {"position": position, "torque": torque})
        if code != 200:
            raise SystemExit(f"grip refused: {d.get('error')}")

    def torque(self, on: bool) -> None:
        self._call("POST", "/torque", {"on": on})

    def publish_calibration(self, kind: str, data: dict) -> None:
        code, d = self._call("PUT", f"/calibration/{kind}", data)
        if code != 200:
            raise SystemExit(f"could not publish {kind} calibration: {d.get('error')}")


Arm = CliArm | HttpArm


def arm(poses_file: Path = POSES) -> Arm:
    """SO101_ARM=http://pi:8101 selects the arm behind `so101 serve`; otherwise the CLI on this machine."""
    url = os.environ.get("SO101_ARM")
    return HttpArm(url) if url else CliArm(poses_file)


def servo_ranges_match(ranges: dict[str, JointRange], arm: Arm) -> list[str]:
    """Joints whose servo limits differ from the calibration file (a stale file would send the arm to the wrong angles)."""
    bad = []
    limits = arm.limits()
    for joint, sid in zip(JOINTS, SERVO_IDS):
        lo, hi = limits.get(sid, (None, None))
        if (lo, hi) != (ranges[joint].min, ranges[joint].max):
            bad.append(f"{joint}: servo {lo}-{hi}, file {ranges[joint].min}-{ranges[joint].max}")
    return bad


def read_joint_ticks(arm: Arm) -> dict[str, int]:
    """Current servo positions, by joint name."""
    positions = arm.positions()
    missing = [JOINTS[SERVO_IDS.index(sid)] for sid in SERVO_IDS if sid not in positions]
    if missing:
        raise SystemExit(f"no position from {', '.join(missing)}")
    return {JOINTS[SERVO_IDS.index(sid)]: int(positions[sid]) for sid in SERVO_IDS}


def read_joint_degrees(ranges: dict[str, JointRange], arm: Arm) -> np.ndarray:
    ticks = read_joint_ticks(arm)
    return np.array([ticks_to_degrees(ticks[j], ranges[j]) for j in JOINTS])


class ArmMove:
    """Supervised moves to joint targets in degrees, validated against the calibrated ranges first."""

    def __init__(self, ranges: dict[str, JointRange], arm: Arm, speed: int = 250, torque: int = 70):
        self.ranges = ranges
        self.arm = arm
        self.speed = speed
        self.torque = torque

    def validate(self, degrees: dict[str, float]) -> tuple[list[int], list[int]]:
        """Joint targets as servo ids and ticks, inside the calibrated range by the supervisor's own 30-tick margin, or a clear error."""
        ids, positions = [], []
        for joint, deg in degrees.items():
            rng = self.ranges[joint]
            ticks = degrees_to_ticks(deg, rng)
            if not (rng.min + 30 <= ticks <= rng.max - 30):
                raise SystemExit(f"{joint} target {deg:.1f}° is {ticks} ticks, outside its usable {rng.min + 30}-{rng.max - 30}")
            ids.append(SERVO_IDS[JOINTS.index(joint)])
            positions.append(ticks)
        return ids, positions

    def go(self, name: str, degrees: dict[str, float], *, release: bool = False) -> tuple[bool, str]:
        """Returns (reached, message). A jam on the way is reported, not raised, so a gripper closing on an object reads as a grasp."""
        ids, positions = self.validate(degrees)
        return self.arm.move(dict(zip(ids, positions)), speed=self.speed, torque=self.torque, release=release)
