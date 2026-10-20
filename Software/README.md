# SO-101 workbench

Software for building, driving and teaching an
[SO-101](https://github.com/TheRobotStudio/SO-ARM100) robot arm: the open
hardware by The Robot Studio, run with [Hugging Face LeRobot](https://huggingface.co/docs/lerobot/so101).
Everything here was built on one real pair of arms, a leader and a follower, in
the order a first build happens: one servo on a bench, ids and gear ratios, a
calibrated follower, poses and replay, the leader, teleoperation, the follower
on a Raspberry Pi behind a browser app with a live 3D model and a safe zone,
and a camera-guided pick. Build the arms from the hardware docs in the
repository root, then start here.

**What you need.** The SO-101 kit (both arms, two Waveshare bus servo boards,
their power supplies); a computer with Nix (macOS or Linux); optionally a
Raspberry Pi 4 for the follower, one or two USB webcams, and a printer for the
calibration board. Nothing here needs a GPU.

**The short path.**

1. `direnv allow` in the checkout: the dev shell brings Python, LeRobot, Bun
   and Node, and syncs them on every entry.
2. `just scan` with one servo plugged in; `so101 set-id` and the bench scripts
   in `so101/bench/` to id a chain; `just calibrate` for each arm.
3. `just teleop <follower port> <leader port>` to drive the follower from the
   leader on one machine.
4. For a remote follower: the Pi section below, then open its page in a
   browser.
5. For the camera-guided pick: `vision/QUEST.md`.

Everything here is under the repository's Apache-2.0 license. Issues and pull
requests are welcome.

| Piece | What it is |
| --- | --- |
| `flake.nix` + `.envrc` | Nix dev shell via direnv: Python 3.12, uv, Bun, Node 24, FFmpeg 7, just |
| `pyproject.toml` + `uv.lock` | LeRobot with the Feetech extra, in a uv-managed `.venv` |
| `justfile` | The bring-up recipes below, `just` lists them |
| `vision/` | ArUco pick: printable markers, hands-free camera calibration, hand-eye calibration, a live viewer, and the pick itself. Start with its `QUEST.md` |
| `so101/` | A small TypeScript driver + CLI that speaks the Feetech bus protocol directly. Bun installs and tests it; the CLI itself runs on Node because `serialport`'s native bindings crash under Bun ([bun#18546](https://github.com/oven-sh/bun/issues/18546)) |
| `WEBUI_CALIBRATION.md` | Guide for the browser-based 3-point calibration studio |

## First entry

```sh
direnv allow     # once; the hook then syncs .venv and node_modules on every entry
just             # list recipes
```

The first shell entry downloads the LeRobot wheels (torch is most of it).
Later entries are instant. Editing `pyproject.toml` is followed by `just sync`.

## Plugging in a servo

The Waveshare bus servo board enumerates on macOS as `/dev/cu.usbmodem…` with
no driver; the LeRobot guide notes its two jumpers must sit on the USB (B)
channel. The servos only answer when the board also has its barrel-jack
supply connected: USB powers the adapter, not the motors. STS3215s ship at
id 1 and 1 Mbaud, which is the only baud rate anything here uses.

```sh
just ports                  # find the board
just scan                   # who answers: id, firmware, baud, position
so101 watch --ids 1         # turn the horn by hand, watch ticks and degrees
so101 move 1 180 --deg      # drive it, gently (500 ticks/s)
so101 status --ids 1        # load, current, voltage, temperature, error bits
so101 read 1 Homing_Offset  # any register by its LeRobot name
just release                # torque off everything
```

If `ports` shows the board but `scan` finds nothing at any baud rate, look at
the USB link before the wiring: on this machine the first plug-in produced
thousands of `transaction error` lines per second in `log stream --predicate
'eventMessage CONTAINS "USB Single Serial"'`, and a replug cleared it. A
cable that is marginal for data shows the same symptom.

Once the arm is assembled and calibrated, move it by demonstration rather
than by joint numbers. Per-joint limits do not stop links from colliding with
each other or the table, and a stalled STS3215 strips its gears before its
overload protection reacts; the follower's first elbow lost its gearbox that
way. Record a path with your hands and replay it:

```sh
so101 pose save home          # torque off, pose the arm by hand, name it (poses.json)
so101 pose save reach
so101 pose go reach           # every joint moves at once and arrives together
so101 pose go home --via reach --release     # route through a waypoint; let go at the end
so101 record path.json        # torque off; move the arm; ctrl-c saves the path
so101 play path.json          # eases to the first pose, then replays at recorded speed
```

Every supervised move (`pose go`, `play`) caps torque at 70 % (`--torque`),
aims 30 ticks inside each servo's calibrated limits so it never drives into
a hard stop, and watches load and progress: a joint that pushes hard without
moving stops all of them where they are and names itself. Ctrl-C does the
same. Torque stays on afterward, so the arm holds; `so101 torque off`
releases it, which drops an extended arm, so go home first.

`so101 set-id 1 3` re-addresses the one servo on the bus. That is the manual
version of what `just setup-motors` does, which also walks the six joints in
LeRobot's order. Every servo ships as id 1, so assign ids from the top down,
gripper (6) first, and each new servo can join the chain without colliding
with one already renamed. That is why LeRobot asks for the gripper first.

On real hardware a six-servo sync read completes in about 1 ms, so `watch`
and LeRobot's teleop loop are limited by their own frame rate, not the bus.

Auto-detection only works with a single adapter plugged in. Exporting
`SO101_PORT` pins the port for the `so101` commands and the LeRobot recipes
alike:

```sh
export SO101_PORT=/dev/cu.usbmodemXXXX
```

## Remote: the follower on a Raspberry Pi

`so101 serve` runs where the follower's board is plugged in and exposes the arm
on the network; `so101 leader` runs where the leader is and streams targets to
it. Both are the same CLI, so the Pi needs this repo, Node 24 and `bun install`
in `Software/so101` (or Nix: the flake has an `aarch64-linux` dev shell).

```sh
# on the Pi (follower board is /dev/ttyACM0)
SO101_PORT=/dev/ttyACM0 so101 serve --listen 0.0.0.0:8101 --file poses.json

# on the Mac (leader board)
SO101_PORT=<leader port> so101 leader --to ws://<pi>:8101
```

The API, all JSON (the full list with bodies is at the top of
`so101/src/server.ts`): `GET /status`, `/limits`, `/poses`, `/recordings`,
`/calibration`; `POST /pose/save`, `/pose/delete`, `/pose/go`, `/move`
(explicit `{"targets": {"3": 2048}}`), `/grip`, `/torque`, `/stop`,
`/record/start`, `/record/stop`, `/recording/delete`, `/play`;
`PUT /calibration/camera|handeye`; and the WebSocket `/follow` taking
`{"1": ticks, ...}` frames. One thing drives the arm at a time. Every move
goes through the joint supervisor; `/stop` ends a move, a playback, a
recording, or a stream, and the arm holds. The stream is clamped to the
follower's calibrated limits, eases over to the first frame, drops jumps no
hand makes, and holds the arm after half a second of silence. There is no
authentication: keep it on a LAN or behind Tailscale.

`GET /` is the browser app: up to two camera panes, a live 3D model of the
arm (its own URDF and meshes, drag a joint and the arm follows), a slider and
hold-to-jog buttons per joint that stream targets like a leader does, a
button per saved pose plus save and delete, hold and release, record and
play, a stop button (space bar), fullscreen (F), a metrics panel, a settings
panel for the safe zone, and a 3D-calibration panel that explains the vision
quest and shows its status. `bench/fake-serve.ts` runs the same server on a
simulated bus for working on the page. The page expects MJPEG streams on
ports 8080 and 8081 of the same host (`?stream=<url>` overrides the first).

**The safe zone.** The server loads `Simulation/SO101/so101_new_calib.urdf`
and its meshes (`--model <dir>`, or the repo layout, or `model/` beside the
package) and runs every target through the arm's geometry: a pose that puts
any part below the table or into the base is a violation. `enforce` refuses
moves and holds a stream at the boundary until the leader backs off; `warn`
only reports; `off` skips the check. Margins and per-joint trims live in
`settings.json` beside the poses. The model takes the middle of each
calibrated range as zero (LeRobot's convention), which can sit a centimetre
or two off the real arm at the gripper: compare the 3D view with the camera
and trim. On a stream, a joint that pushes hard without moving is a jam: it
is parked where it is and follows again once the leader backs off. Jams,
blocks, stalls, peak loads and temperatures go to `metrics.jsonl`.

**Running on a Raspberry Pi.** The service files in `so101/deploy/pi/` are
the ones running on the author's Pi 4 (DietPi, Node 24, Bun for the install):

```sh
# on the Pi, from a copy of Software/so101 at /home/dietpi/so101 (rsync, without node_modules)
cd /home/dietpi/so101 && bun install --frozen-lockfile
rsync Simulation/SO101/{*.urdf,assets} to /home/dietpi/so101/model/      # the 3D model and the safe zone
rsync Software/vision/printables to /home/dietpi/so101/printables/        # the sheets the page offers
sudo apt-get install ustreamer
sudo install -m 755 deploy/pi/so101-camera /usr/local/bin/
sudo install -m 644 deploy/pi/so101.service deploy/pi/so101-camera@.service /etc/systemd/system/
ls /dev/v4l/by-id/*-video-index0 | head -1 | sudo tee /etc/so101/camera1   # pin camera 1 by name
sudo systemctl enable --now so101.service so101-camera@1.service so101-camera@2.service
```

`so101.service` runs `so101 serve` on port 8101 with the board at
`/dev/ttyACM0`. Camera 1 is the device named in `/etc/so101/camera1` and
streams on port 8080; `so101-camera@2` serves the first other camera on 8081
as soon as it is plugged in, so plugging a second camera never swaps the
first. A page change deploys with the rsync alone; a server change needs
`systemctl restart so101.service`, which ends a running stream (the arm
holds), so do it while the arm is idle. To run a stream by hand:

```sh
sudo apt-get install ustreamer
ustreamer --device=/dev/v4l/by-id/<camera>-video-index0 --host=0.0.0.0 --port=8080 \
  --resolution=1280x720 --format=MJPEG --desired-fps=20 --allow-origin='*'
```

`--allow-origin` lets the viewer read the stream from another port. Chrome
reads the stream in a script (frame count, 3 s stall watchdog); Safari and
the iPhone show it as a plain image and reconnect when the picture stops
changing. On an iPhone, "Add to Home Screen" gives the fullscreen view.

The vision kit in `Software/vision` works against this server and stream:
`SO101_ARM=http://<pi>:8101 SO101_CAMERA=http://<pi>:8080/stream`, then the
`just vision-*` recipes. See `Software/vision/QUEST.md`.

## LeRobot flow

Follows the [SO-101 guide](https://huggingface.co/docs/lerobot/so101). The
recipes default to the follower; pass `leader` for the other arm.

```sh
just find-port                         # interactive unplug/replug port discovery
just setup-motors                      # follower: ids 1-6, one servo at a time
just setup-motors leader
just calibrate                         # saved as so101_follower
just calibrate leader                  # saved as so101_leader
just teleop /dev/cu.usbmodemAAA /dev/cu.usbmodemBBB   # follower port, leader port
```

Calibration lands in `~/.cache/huggingface/lerobot/calibration/`, under
`robots/so_follower/` and `teleoperators/so_leader/`, named by id. Recording
datasets needs the `dataset` extra as well: add it to `pyproject.toml` and
`just sync`. Its video encoder, TorchCodec, loads FFmpeg 7 at import time,
which is why the shell carries that major version rather than the newest.

## Joint map

Both arms use the same ids. The follower runs 1/345 gearing throughout; the
leader mixes ratios so it back-drives easily. `lerobot-setup-motors` asks for
the gripper first and then walks down to id 1, one servo on the bus at a time.

| id | joint | leader gear |
| --- | --- | --- |
| 1 | shoulder_pan | 1/191 |
| 2 | shoulder_lift | 1/345 |
| 3 | elbow_flex | 1/191 |
| 4 | wrist_flex | 1/147 |
| 5 | wrist_roll | 1/147 |
| 6 | gripper | 1/147 |

## References

The methods behind the vision kit, so you can read the originals:

- **Camera calibration.** Z. Zhang, *A Flexible New Technique for Camera
  Calibration*, IEEE TPAMI 22(11), 2000. [doi:10.1109/34.888718](https://doi.org/10.1109/34.888718).
  OpenCV's `calibrateCamera` implements it; the ChArUco flow is in the
  [Calibration with ArUco and ChArUco](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html) tutorial.
- **ArUco markers and ChArUco boards.** S. Garrido-Jurado, R. Muñoz-Salinas,
  F. J. Madrid-Cuevas, M. J. Marín-Jiménez, *Automatic generation and detection
  of highly reliable fiducial markers under occlusion*, Pattern Recognition
  47(6), 2014. [doi:10.1016/j.patcog.2014.01.005](https://doi.org/10.1016/j.patcog.2014.01.005).
  OpenCV: [Detection of ArUco markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).
- **Pose of one square marker (PnP).** T. Collins, A. Bartoli, *Infinitesimal
  Plane-Based Pose Estimation*, IJCV 109(3), 2014.
  [doi:10.1007/s11263-014-0725-5](https://doi.org/10.1007/s11263-014-0725-5).
  This is `SOLVEPNP_IPPE_SQUARE`; it returns the two planar solutions, and the
  kit keeps the clearly better one (`ambiguity`).
- **Hand-eye calibration, the classic forms (AX = XB).** R. Y. Tsai, R. K. Lenz,
  *A new technique for fully autonomous and efficient 3D robotics hand/eye
  calibration*, IEEE T-RA 5(3), 1989. [doi:10.1109/70.34770](https://doi.org/10.1109/70.34770).
  F. C. Park, B. J. Martin, *Robot sensor calibration: solving AX = XB on the
  Euclidean group*, IEEE T-RA 10(5), 1994. [doi:10.1109/70.326576](https://doi.org/10.1109/70.326576).
  K. Daniilidis, *Hand-Eye Calibration Using Dual Quaternions*, IJRR 18(3), 1999.
  [doi:10.1177/02783649922066213](https://doi.org/10.1177/02783649922066213).
- **Robot-world / hand-eye in one solve (AX = ZB), what the kit uses.**
  M. Shah, *Solving the Robot-World/Hand-Eye Calibration Problem Using the
  Kronecker Product*, ASME J. Mechanisms and Robotics 5(3), 2013.
  [doi:10.1115/1.4024473](https://doi.org/10.1115/1.4024473). A. Li, L. Wang,
  D. Wu, *Simultaneous robot-world and hand-eye calibration using
  dual-quaternions and Kronecker product*, Int. J. Physical Sciences 5(10), 2010.
  OpenCV: `calibrateRobotWorldHandEye` and `calibrateHandEye` in the
  [calib3d module](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).
- **Two cameras, a full orientation.** R. Hartley, A. Zisserman, *Multiple View
  Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004
  (epipolar geometry, `stereoCalibrate`; triangulation, `triangulatePoints`).
- **The arm's model.** The URDF and meshes in `Simulation/SO101` from this
  repository; LeRobot's `RobotKinematics` runs them through
  [placo](https://github.com/Rhoban/placo) for the IK; the TypeScript server
  reads the same files for its own forward kinematics and the safe zone.
- **The robot and the stack.** [SO-ARM100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100)
  by The Robot Studio; [Hugging Face LeRobot](https://huggingface.co/docs/lerobot/so101),
  whose calibration convention (zero at the middle of each joint's range) the
  model follows; [three.js](https://threejs.org) and
  [urdf-loader](https://github.com/gkjohnson/urdf-loaders) for the 3D view;
  the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API)
  for live mode; [µStreamer](https://github.com/pikvm/ustreamer) for the cameras.
