# Conveyor pick-and-place cell (ROS 2 Humble · Gazebo Fortress · MoveIt 2 · OpenCV)

Boxes ride a conveyor. An overhead camera finds them, and a 6-DOF arm picks each one
**from the moving belt** and places it on a table beside it. The cycle repeats for 6 boxes.
`mode:=stop` keeps the simpler version: stop the belt, then pick.

## Run

```bash
./run.sh                 # builds the Docker image on first run, then launches everything
./run.sh rviz:=true      # + RViz: planning scene, planned paths, camera debug image
./run.sh mode:=stop      # stop-and-pick instead of picking on the fly
./stop.sh                # stop the simulation (container keeps running)
./shell.sh               # another shell inside the container
```

Everything runs inside the `pick_place` container. Your host Python/OpenCV is not touched.

## What happens

| Stage | Where |
|---|---|
| Belt drive and box feeding | `scripts/box_spawner.py`: `/conveyor/speed` (Float64, m/s) |
| Box detection: HSV → contour → `minAreaRect` | `scripts/box_detector.py` |
| Pixel → X,Y,Z: intrinsics + TF2 (`world → camera_link → camera_optical`) | `scripts/box_detector.py` |
| Cycle: stop belt → IK → MoveIt plan → linear approach → grip → place | `scripts/pick_place.py` |
| Arm + gripper model and ros2_control | `urdf/arm.urdf.xacro`, `config/controllers.yaml` |
| MoveIt (SRDF, KDL IK, OMPL, controllers) | `config/moveit/`, `config/ompl_planning.yaml` |
| World: belt, table, camera | `worlds/factory.sdf` |

Useful topics: `/boxes/detections` (PoseArray in `world`), `/boxes/debug_image`,
`/conveyor/speed`, `/joint_states`.

## Picking from the moving belt (conveyor tracking)

1. **Wait.** The arm hovers above the belt at x = 0.10 with the gripper open.
2. **Detect.** The camera sees each box upstream. Detections touching the image edge are
   ignored because their centroids are biased. A few frames are averaged and projected to a
   common time with the known belt speed (the role an encoder plays on a real line). Vision
   cross-checks that speed.
3. **Predict.** The pick node computes when the box will reach the gripper (`t_meet`).
4. **Plan.** It builds a trajectory sampled every 0.1 s: move with the belt (x = x0 + v·t),
   line up with the box's lane, descend, hold while the fingers close, and lift. Each waypoint
   gets its own IK solution, seeded from the previous one.
5. **Execute.** The trajectory goes straight to `arm_controller` with `header.stamp = t_meet`,
   so it starts exactly when the box arrives. The gripper close is scheduled the same way.
6. **Measure the grip.** After the lift, the box-in-gripper offset is measured and compensated
   when placing.

Typical numbers: about 8 mm interception error, box placed within 1 mm of its slot, about
21 s per box (boxes arrive 1.5 m apart at 0.07 m/s).

## Simulation shortcuts (and why)

- **Gripping.** Friction grasps in Gazebo slip and jitter. The fingers close around the box.
  Then the pick node swaps the belt box for an identical `held_i` box at the same pose.
  A `DetachableJoint` on the wrist attaches it. Publishing to `/gripper/detach_i` releases it.
  One slot exists per box, because a Fortress DetachableJoint can't be reused.
- **Belt.** The Fortress `TrackController` did not move objects on this machine. Instead,
  each box carries a `VelocityControl` plugin that listens to one shared belt-speed topic.
  The belt surface is frictionless, so friction doesn't drag boxes below the commanded speed.
- **Camera calibration** is the static TF in the launch file. It matches the camera pose in the world.
- **`IGN_IP=127.0.0.1`.** Gazebo transport otherwise binds to the first network interface. Here
  that was a VPN interface, and it silently dropped gripper-release messages.

## Measured results (headless runs in the container)

| | moving belt (default) | `mode:=stop` |
|---|---|---|
| Boxes placed correctly | 6/6 | 6/6 |
| Vision / interception error | about 8 mm interception | 0.7–1.8 mm vision |
| Time per box (sim) | about 21 s | about 24 s |

Every final pose is checked in Gazebo. A box that isn't on its slot is reported as MISPLACED.

Other launch options: `headless:=true` (no Gazebo window), `gz_verbosity:=4` (Gazebo debug log).

## Real hardware

A real cell needs properly rated robot hardware, emergency stops, guarding or light curtains,
safety-rated controls and a risk assessment. Simulation does not establish real-world safety.
