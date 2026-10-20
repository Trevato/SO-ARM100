# SO-101 workbench. `just` lists recipes; the dev shell (direnv) provides the tools.
# Set SO101_PORT to the servo board's port once (`just ports` shows it) and every
# recipe below picks it up.

set shell := ["bash", "-euo", "pipefail", "-c"]

default:
    @just --list --unsorted

# ── Feetech bus, directly (Software/so101) ──────────────────────────────

# List serial ports; * marks USB adapters
ports:
    so101 ports

# Ping every id on the bus and show what answers
scan *args:
    so101 scan {{ args }}

# One-shot telemetry for ids 1-6
status *args:
    so101 status {{ args }}

# Live positions — move the arm by hand
watch *args:
    so101 watch {{ args }}

# Release every servo so the arm can be posed by hand
release:
    so101 torque off

# Re-address the single connected servo: `just set-id 1 3`
set-id current new:
    so101 set-id {{ current }} {{ new }}

# ── LeRobot flow (https://huggingface.co/docs/lerobot/so101) ────────────

# Interactive: unplug/replug the board to learn its port
find-port:
    lerobot-find-port

# Assign ids 1-6 one servo at a time, in LeRobot's order. `just setup-motors leader` for the other arm
setup-motors arm="follower":
    role=$([ "{{ arm }}" = leader ] && echo teleop || echo robot); \
    lerobot-setup-motors --$role.type=so101_{{ arm }} ${SO101_PORT:+--$role.port=$SO101_PORT}

# Record joint ranges; saved under ~/.cache/huggingface/lerobot/calibration/ as so101_<arm>
calibrate arm="follower":
    role=$([ "{{ arm }}" = leader ] && echo teleop || echo robot); \
    lerobot-calibrate --$role.type=so101_{{ arm }} --$role.id=so101_{{ arm }} ${SO101_PORT:+--$role.port=$SO101_PORT}

# Drive the follower from the leader: `just teleop <follower port> <leader port>`
teleop follower_port leader_port:
    lerobot-teleoperate \
      --robot.type=so101_follower --robot.port={{ follower_port }} --robot.id=so101_follower \
      --teleop.type=so101_leader --teleop.port={{ leader_port }} --teleop.id=so101_leader

# ── Housekeeping ────────────────────────────────────────────────────────

# Typecheck and test the TypeScript app
check:
    cd Software/so101 && bunx tsc --noEmit && bun test

# Re-lock and re-sync the Python side after editing pyproject.toml
sync:
    uv lock && uv sync --locked

# ── Vision: ArUco pick (Software/vision, see its QUEST.md) ──────────────
# SO101_CAMERA is an OpenCV index (default 1; `lerobot-find-cameras opencv` lists them) or the
# ustreamer URL on the Pi, http://<pi>:8080/stream. SO101_ARM=http://<pi>:8101 sends every arm
# move to `so101 serve` there instead of the so101 CLI on this machine.

camera := env("SO101_CAMERA", "1")

# Regenerate the printable markers and calibration board in Software/vision/printables/ (the arm's page serves them too)
vision-markers:
    cd Software/vision && python markers.py printables

# Live view of detected markers, with poses once calibrated
vision-view:
    cd Software/vision && python view.py --camera {{ camera }} --intrinsics ../../outputs/camera.json --handeye ../../outputs/handeye.json

# Calibrate camera intrinsics from the ChArUco board, hands-free
vision-calibrate:
    cd Software/vision && python calibrate_camera.py --camera {{ camera }} --out ../../outputs/camera.json

# Solve where the camera is relative to the arm: `just vision-handeye he1,he2,he3,he4,he5,he6`
vision-handeye poses:
    cd Software/vision && python handeye.py --camera {{ camera }} --poses {{ poses }} --intrinsics ../../outputs/camera.json --out ../../outputs/handeye.json

# Rehearse the pick: see the cube, plan the joints, move nothing
vision-rehearse:
    cd Software/vision && python pick.py --camera {{ camera }} --intrinsics ../../outputs/camera.json --handeye ../../outputs/handeye.json --dry-run

# Pick the cube and put it at the `drop` pose
vision-pick:
    cd Software/vision && python pick.py --camera {{ camera }} --intrinsics ../../outputs/camera.json --handeye ../../outputs/handeye.json

# Offline tests for the vision math
vision-test:
    cd Software/vision && python -m pytest -q
