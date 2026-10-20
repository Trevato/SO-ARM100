"""Camera intrinsics from a ChArUco board, hands-free: hold the board in view and move it around; views are
captured automatically whenever the board is seen well and from a new angle. Finishes itself.

    python Software/vision/calibrate_camera.py --camera 1 --out outputs/camera.json
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rerun as rr

from common import Intrinsics, arm, camera_source, grab, open_camera
from markers import charuco_board

NEEDED = 15
MIN_CORNERS = 12
TILTED_NEEDED = 6      # views where the board is clearly not parallel to the image plane
TILT_SKEW_DEG = 4.0    # opposite board edges converge by at least this much in the image


def skew_deg(corners: np.ndarray) -> float:
    """How much the board's outline departs from a parallelogram: 0 when parallel to the sensor, larger when tilted."""
    hull = cv2.convexHull(corners.astype(np.float32)).reshape(-1, 2)
    quad = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True).reshape(-1, 2)
    if len(quad) != 4:
        return 0.0
    def angle(a, b):
        return np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0]))
    def diff(a, b):
        d = abs(a - b) % 180
        return min(d, 180 - d)
    return diff(angle(quad[0], quad[1]), angle(quad[3], quad[2])) + diff(angle(quad[1], quad[2]), angle(quad[0], quad[3]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=camera_source, default=1, help="an OpenCV index, or the ustreamer URL on the Pi")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--out", type=Path, default=Path("outputs/camera.json"))
    ap.add_argument("--publish-as", default="camera", help="name on the arm's server: camera for the first camera, camera2 for the second")
    args = ap.parse_args()

    rr.init("so101 camera calibration", spawn=True)
    board = charuco_board()
    cdet = cv2.aruco.CharucoDetector(board)
    cap = open_camera(args.camera, args.width, args.height)

    views: list[tuple[np.ndarray, np.ndarray]] = []  # (charuco corners, ids)
    fingerprints: list[np.ndarray] = []
    tilted = 0
    last_capture = 0.0
    print(f"show the board to camera {args.camera}; move it around AND tilt it; {NEEDED} views, at least {TILTED_NEEDED} tilted")
    while len(views) < NEEDED or tilted < TILTED_NEEDED:
        frame = grab(cap)
        corners, ids, _, _ = cdet.detectBoard(frame)
        n = 0 if ids is None else len(ids)
        rr.log("camera/image", rr.Image(frame[..., ::-1]))
        status = f"{len(views)}/{NEEDED} views, {tilted}/{TILTED_NEEDED} tilted"
        if n >= MIN_CORNERS:
            pts = corners.reshape(-1, 2)
            rr.log("camera/corners", rr.Points2D(pts, radii=4, colors=[0, 255, 0]))
            tilt = skew_deg(pts)
            # A new view must look different: position, spread, or tilt moved by a margin.
            fp = np.array([*pts.mean(axis=0), *pts.std(axis=0), tilt * 10])
            fresh = all(np.linalg.norm(fp - f) > 40 for f in fingerprints)
            need_tilt = len(views) >= NEEDED and tilted < TILTED_NEEDED
            if fresh and time.time() - last_capture > 0.7 and (tilt >= TILT_SKEW_DEG or not need_tilt):
                views.append((corners, ids))
                fingerprints.append(fp)
                tilted += tilt >= TILT_SKEW_DEG
                last_capture = time.time()
                status += "  captured" + (" (tilted)" if tilt >= TILT_SKEW_DEG else "")
            elif need_tilt:
                status += "  tilt the board more"
        else:
            rr.log("camera/corners", rr.Clear(recursive=False))
        rr.log("status", rr.TextLog(status))
        print(f"\r{status}   ", end="", flush=True)
    cap.release()

    objp, imgp = [], []
    for corners, ids in views:
        o, i = board.matchImagePoints(corners, ids)
        objp.append(o)
        imgp.append(i)
    rms, K, dist, _, _ = cv2.calibrateCamera(objp, imgp, (args.width, args.height), None, None, flags=cv2.CALIB_FIX_K3)
    intr = Intrinsics(args.width, args.height, K, dist, float(rms))
    f = (K[0, 0] + K[1, 1]) / 2
    print(f"\nfocal {K[0, 0]:.0f}/{K[1, 1]:.0f} px, center ({K[0, 2]:.0f}, {K[1, 2]:.0f}), reprojection rms {rms:.2f} px")
    if not (0.5 * args.width < f < 3.0 * args.width):
        raise SystemExit("that focal length is not a webcam's; the views did not constrain it. Run again with the board tilted more")
    if rms > 1.0:
        print("rms above 1 px is mediocre: flatter board, better light, or more varied angles would help")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    intr.save(args.out)
    print(f"saved {args.out}")
    arm().publish_calibration(args.publish_as, json.loads(args.out.read_text()))


if __name__ == "__main__":
    main()
