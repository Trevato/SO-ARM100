"""Generate the printables: a ChArUco board for camera calibration, six 40 mm cube markers, one 25 mm gripper marker.

    python Software/vision/markers.py out/           # PNGs at 300 DPI; print at 100 % scale, matte paper
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

from common import CUBE_IDS, DICTIONARY, GRIPPER_ID

DPI = 300
MM = DPI / 25.4  # pixels per millimeter

BOARD_SQUARES = (7, 5)
BOARD_SQUARE_MM = 30.0
BOARD_MARKER_MM = 22.0
CUBE_MARKER_MM = 40.0
GRIPPER_MARKER_MM = 25.0


def charuco_board() -> cv2.aruco.CharucoBoard:
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    # Board markers use ids after the cube and gripper ones so nothing collides in a frame.
    ids = np.arange(20, 20 + (BOARD_SQUARES[0] * BOARD_SQUARES[1]) // 2, dtype=np.int32)
    return cv2.aruco.CharucoBoard(BOARD_SQUARES, BOARD_SQUARE_MM / 1000, BOARD_MARKER_MM / 1000, dictionary, ids)


def marker_tile(marker_id: int, size_mm: float, label: str) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    px = int(round(size_mm * MM))
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, px)
    border = int(round(size_mm * 0.25 * MM))  # white quiet zone: a quarter of the marker on each side
    tile = cv2.copyMakeBorder(img, border, border + int(8 * MM), border, border, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(tile, label, (border, tile.shape[0] - int(2 * MM)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
    # Cut marks at the corners only: a closed rectangle around the tile is itself a candidate square and confuses detection.
    h, w = tile.shape
    tick = int(4 * MM)
    for (x, y, dx, dy) in ((0, 0, 1, 1), (w - 1, 0, -1, 1), (0, h - 1, 1, -1), (w - 1, h - 1, -1, -1)):
        cv2.line(tile, (x, y), (x + dx * tick, y), 128, 1)
        cv2.line(tile, (x, y), (x, y + dy * tick), 128, 1)
    return tile


def sheet(tiles: list[np.ndarray], columns: int) -> np.ndarray:
    h = max(t.shape[0] for t in tiles)
    w = max(t.shape[1] for t in tiles)
    rows = (len(tiles) + columns - 1) // columns
    canvas = np.full((rows * h, columns * w), 255, np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, columns)
        canvas[r * h : r * h + t.shape[0], c * w : c * w + t.shape[1]] = t
    return canvas


def main(out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    board = charuco_board()
    margin = int(10 * MM)  # generateImage fits the squares inside the margins, so add them to the canvas
    board_px = (int(BOARD_SQUARES[0] * BOARD_SQUARE_MM * MM) + 2 * margin, int(BOARD_SQUARES[1] * BOARD_SQUARE_MM * MM) + 2 * margin)
    board_img = board.generateImage(board_px, marginSize=margin)
    cv2.imwrite(str(out / "charuco_board.png"), board_img)

    cube = [marker_tile(i, CUBE_MARKER_MM, f"cube {i}  {CUBE_MARKER_MM:.0f}mm") for i in CUBE_IDS]
    cv2.imwrite(str(out / "cube_markers.png"), sheet(cube, 3))
    cv2.imwrite(str(out / "gripper_marker.png"), sheet([marker_tile(GRIPPER_ID, GRIPPER_MARKER_MM, f"gripper {GRIPPER_ID}  {GRIPPER_MARKER_MM:.0f}mm")], 1))

    for name in ("charuco_board", "cube_markers", "gripper_marker"):
        img = cv2.imread(str(out / f"{name}.png"), cv2.IMREAD_GRAYSCALE)
        print(f"{name}.png  {img.shape[1] / MM:.0f} x {img.shape[0] / MM:.0f} mm at {DPI} DPI")
    print(f"print at 100 % scale; check with a ruler: a cube marker's black square is {CUBE_MARKER_MM:.0f} mm, a board square {BOARD_SQUARE_MM:.0f} mm")


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "printables"))
