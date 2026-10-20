# Quest: the arm picks up a cube it can see

One session, roughly an hour of your time, most of it watching. Every arm
move is a supervised pose move, so a mistake ends in "stopped, holding", not a
stripped gear. Run everything from the repo root in your shell, with the arm
clamped.

With the arm and the webcam on the Pi (the usual setup), point the kit there:

    export SO101_ARM=http://<pi>:8101
    export SO101_CAMERA=http://<pi>:8080/stream

With the arm and the webcam on this machine instead: `SO101_PORT` exported and
`SO101_CAMERA` set to the webcam's index from `lerobot-find-cameras opencv`.

Poses are saved from the arm's web page (`http://<arm>:8101/`): press
**Release**, move the arm by hand, press **Save pose**, and uncheck *with
gripper*. The page's **3D calibration** button shows this quest's status.

## 0. Loot (before you start)

- Print the three sheets in `Software/vision/printables/` at 100 % scale on
  matte paper: the arm's page offers them for download under **3D
  calibration**, and `just vision-markers` regenerates them. Check a cube
  marker's black square with a ruler: 40 mm. Cut out the six cube markers and
  the gripper marker. Tape the cube markers to the faces of a cube-ish object
  of about 40 to 50 mm (a wooden block, a stack of Lego). Tape the gripper
  marker flat on the outside of the fixed jaw, facing where the camera will be.
- Put the webcam on the overhead mount, or on anything rigid, looking down at
  the table in front of the arm. It must not move again after step 2.
- The arm grasps straight down, so the cube must sit where a vertical gripper
  can reach. Computed from the arm's own model and your calibration: the sweet
  spot is 18 to 24 cm in front of the base, up to 12 cm to either side. Right
  in front of the base closer than 16 cm is awkward for the wrist; avoid it.

## 1. See (2 min)

    just vision-view

A viewer opens with the camera image and every marker it recognizes.
Confirm the cube and the gripper marker are both detected where you'll work.
Ctrl-C.

## 2. Calibrate the camera (5 min, hands-free capture)

    just vision-calibrate

The board is the ChArUco board: a checkerboard with a small marker inside
each white square, `Software/vision/printables/charuco_board.png`, printed at 100 %. Hold it in
front of the camera and slowly move and tilt it.
Views are captured automatically as the angle changes; the counter stops at 15.
Reprojection error under 1 px is good. Saves `outputs/camera.json`.

## 3. Tell the arm where the camera is (10 min)

First two poses of the arm alone (no gripper), used by every later step:
`home` (the arm resting, nothing in reach) and `drop` (gripper over the bin
or a mark). Save them from the web page without the gripper, or from a shell
on the machine with the arm:

    so101 torque off
    so101 pose save home --ids 1-5
    so101 pose save drop --ids 1-5

Then, by hand, put the arm in six to eight poses where the gripper marker
faces the camera from different places AND different tilts and rolls, saving
each as `he1`, `he2`, ... `he8`, without the gripper. Variety of orientation
is what the solver needs; six positions at the same angle will be refused.

Then step back:

    just vision-handeye he1,he2,he3,he4,he5,he6,he7,he8

The arm visits each pose on its own, parks at `home`, and the solver prints
the camera's position relative to the base plus a residual per pose. It saves
only when the mean residual is under 12 mm and the worst under 25 mm, and
publishes the result to the arm's server so the web page shows it. A
correct setup gives a few millimeters. If it refuses: between 8 and 12 mm,
add two more poses with bigger tilts and rerun; above that, a joint direction
disagrees between the servo and the model, so say so and we'll find which
with the viewer.

## 4. Pick (the payoff)

Put the cube in the zone, then first a rehearsal that moves nothing:

    just vision-rehearse

It prints where it sees the cube in arm coordinates and the joint angles it
plans. If that looks sane, the real thing:

    just vision-pick

Sequence: open gripper, pre-grasp above the cube, descend, close until the
jaws meet the cube (the jam detector is the grasp sensor, and the jaw
position confirms something is between them), squeeze gently, lift, go to
`drop`, open, return to `home`, release.

## When something is off

- Nothing detected: light, glare, or a marker too small in the image.
- Hand-eye residuals large: run `just vision-view` with the arm holding a pose
  and compare the printed "base" coordinates of the gripper marker with where
  the gripper physically is.
- Pick misses by a constant offset: the grasp depth and approach heights are
  constants at the top of `pick.py`. A block that is not 40 mm wide:
  `--cube-mm` (the recipes pass none; run `python Software/vision/pick.py`
  directly for that).

## References

The methods behind this quest, so you can read the originals:

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
