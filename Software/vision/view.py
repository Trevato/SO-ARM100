"""Live view of what the camera sees and which markers it recognizes, with poses in millimeters.

    python Software/vision/view.py --camera 1 [--intrinsics outputs/camera.json] [--handeye outputs/handeye.json]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rerun as rr

from common import CUBE_IDS, GRIPPER_ID, Intrinsics, camera_source, detect, detector, grab, open_camera

SIZES = {**{i: 0.040 for i in CUBE_IDS}, GRIPPER_ID: 0.025}



def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=camera_source, default=1, help="an OpenCV index, or the ustreamer URL on the Pi")
    ap.add_argument("--intrinsics", type=Path, default=Path("outputs/camera.json"))
    ap.add_argument("--handeye", type=Path, default=Path("outputs/handeye.json"))
    args = ap.parse_args()

    intr = Intrinsics.load(args.intrinsics) if args.intrinsics.exists() else None
    T_base_cam = np.array(json.loads(args.handeye.read_text())["T_base_cam"]) if args.handeye.exists() else None
    cap = open_camera(args.camera, intr.width if intr else 1280, intr.height if intr else 720)
    det = detector()
    rr.init("so101 vision", spawn=True)
    print("viewer open; ctrl-c to stop")
    try:
        while True:
            frame = grab(cap)
            rr.log("camera/image", rr.Image(frame[..., ::-1]))
            strips, labels, lines = [], [], []
            for d in sorted(detect(frame, det, intr, SIZES), key=lambda d: d.id):
                label = f"id {d.id}"
                T = d.T_cam_marker
                if T is not None:
                    t = T[:3, 3] * 1000
                    label += f"  cam {t[0]:.0f},{t[1]:.0f},{t[2]:.0f} mm"
                    if T_base_cam is not None:
                        b = (T_base_cam @ T)[:3, 3] * 1000
                        label += f"  base {b[0]:.0f},{b[1]:.0f},{b[2]:.0f} mm"
                    if d.ambiguity < 1.5:
                        label += "  (pose unsure: tilt the marker a little)"
                strips.append(np.vstack([d.corners, d.corners[:1]]))
                labels.append(label)
                lines.append(label)
            rr.log("camera/markers", rr.LineStrips2D(strips, labels=labels, colors=[0, 255, 0]) if strips else rr.Clear(recursive=False))
            print("\r" + (" | ".join(lines) if lines else "no markers") + " " * 20, end="", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        cap.release()


if __name__ == "__main__":
    main()
