#!/usr/bin/env python3
"""Conveyor pick-and-place cell controller.

mode:=moving (default): pick on the fly from the running belt (see pick_moving()).
mode:=stop:
  1. Run the belt until the camera sees a box in the pick zone, then stop it.
  2. Average a few detections -> box X, Y, yaw in the world frame.
  3. MoveIt: plan to a pre-grasp pose above the box (IK -> joint goal -> OMPL).
  4. Cartesian descent, close the gripper, grip (attach), Cartesian lift.
  5. Plan to a pre-place pose above the next free slot on the place table.
  6. Cartesian descent, release (detach), lift, open gripper, return home.
"""
import math
import threading
import time

import rclpy
import rclpy.time
from builtin_interfaces.msg import Duration, Time as TimeMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

import gz_utils

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
HOME = [2.07, -0.2, 1.6, 0.0, 1.74, 0.0]
GRIPPER_OPEN = 0.075     # finger inner face distance from tool axis (gap = 2x)
GRIPPER_CLOSED = 0.066   # gap 0.132 for a 0.12 box: fingers hug it without pushing


def top_down_quat(yaw):
    """Tool Z pointing straight down, rotated by yaw about world Z.  q = Rz(yaw) * Rx(pi)."""
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def yaw_from_quat(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def make_pose(x, y, z, yaw):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    qx, qy, qz, qw = top_down_quat(yaw)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def box_object(oid, frame, size, pose, op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = oid
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = op
    return co


class PickPlace(Node):
    def __init__(self):
        super().__init__('pick_place')
        self.declare_parameter('pick_x', 0.30)
        self.declare_parameter('belt_speed', 0.15)
        self.declare_parameter('belt_top', 0.725)
        self.declare_parameter('table_top', 0.52)
        self.declare_parameter('place_center', [0.45, -0.75])
        self.declare_parameter('slots', 6)
        self.declare_parameter('mode', 'moving')          # 'moving' or 'stop'
        self.declare_parameter('track_speed', 0.07)       # belt speed while picking on the fly (m/s)
        self.declare_parameter('track_start_x', 0.10)     # where the gripper meets the box
        self.at_wait = False
        self.swap_lat = 0.3          # s, refined after every swap

        cb = ReentrantCallbackGroup()
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.cart_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=cb)
        self.scene_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=cb)
        self.move_ac = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self.exec_ac = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=cb)
        self.arm_ac = ActionClient(self, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.grip_ac = ActionClient(self, FollowJointTrajectory,
                                    '/gripper_controller/follow_joint_trajectory', callback_group=cb)
        self.belt_pub = self.create_publisher(Float64, '/conveyor/speed', 10)
        # persistent publishers (bridged to Gazebo): one-shot CLI publishes can be lost
        self.detach_pubs = [self.create_publisher(Empty, f'/gripper/detach_{i}', 10)
                            for i in range(self.get_parameter('slots').value)]

        self.joint_state = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detections = None
        self.det_lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self.on_js, 10, callback_group=cb)
        self.create_subscription(PoseArray, '/boxes/detections', self.on_det, 10, callback_group=cb)

    # ------------------------------------------------------------------ utils
    def on_js(self, msg):
        self.joint_state = msg

    def on_det(self, msg):
        with self.det_lock:
            self.detections = (self.sim_now(), msg)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sim_sleep(self, sec):
        end = self.sim_now() + sec
        while rclpy.ok() and self.sim_now() < end:
            time.sleep(0.02)

    def wait(self, fut, timeout=60.0):
        end = time.time() + timeout
        while rclpy.ok() and not fut.done():
            if time.time() > end:
                return None
            time.sleep(0.01)
        return fut.result()

    def run_action(self, client, goal, timeout=120.0):
        gh = self.wait(client.send_goal_async(goal), 15.0)
        if gh is None or not gh.accepted:
            return None
        res = self.wait(gh.get_result_async(), timeout)
        return res.result if res else None

    def arm_positions(self):
        js = self.joint_state
        idx = {n: i for i, n in enumerate(js.name)}
        return [js.position[idx[j]] for j in ARM_JOINTS]

    def set_belt(self, v, repeat=3):
        for _ in range(repeat):
            self.belt_pub.publish(Float64(data=float(v)))
            time.sleep(0.05)

    # ------------------------------------------------------------- planning
    def setup_scene(self):
        """Tell MoveIt about the static obstacles so plans avoid them."""
        def pose(x, y, z):
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
            p.orientation.w = 1.0
            return p
        cx, cy = self.get_parameter('place_center').value
        tt = self.get_parameter('table_top').value
        bt = self.get_parameter('belt_top').value
        objs = [
            box_object('floor', 'world', (4.0, 4.0, 0.02), pose(0, 0, -0.011)),
            box_object('conveyor', 'world', (3.0, 0.42, bt), pose(0, 0, bt / 2)),
            box_object('place_table', 'world', (0.5, 0.4, tt), pose(cx, cy, tt / 2)),
            box_object('camera', 'world', (0.1, 0.1, 0.1), pose(0.3, 0, 2.0)),
        ]
        ps = PlanningScene(is_diff=True)
        ps.world.collision_objects = objs
        self.apply_scene(ps)

    def apply_scene(self, ps):
        res = self.wait(self.scene_cli.call_async(ApplyPlanningScene.Request(scene=ps)), 10.0)
        return bool(res and res.success)

    def attach_box(self, oid, size):
        aco = AttachedCollisionObject()
        aco.link_name = 'tcp'
        centre = Pose()
        centre.orientation.w = 1.0
        aco.object = box_object(oid, 'tcp', size, centre)
        aco.touch_links = ['finger_left', 'finger_right', 'link6', 'link5']
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        self.apply_scene(ps)

    def detach_box(self, oid):
        """Detaching moves the object back into the world: placed boxes become obstacles."""
        aco = AttachedCollisionObject()
        aco.link_name = 'tcp'
        aco.object.id = oid
        aco.object.operation = CollisionObject.REMOVE
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        self.apply_scene(ps)

    def ik(self, pose, seed, avoid_collisions=True, timeout_ns=300_000_000):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = 'arm'
        r.ik_link_name = 'tcp'
        r.avoid_collisions = avoid_collisions
        r.timeout = Duration(sec=0, nanosec=timeout_ns)
        js = JointState(name=list(ARM_JOINTS), position=list(seed))
        r.robot_state.joint_state = js
        r.pose_stamped = PoseStamped(pose=pose)
        r.pose_stamped.header.frame_id = 'world'
        res = self.wait(self.ik_cli.call_async(req), 10.0)
        if not res or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    def best_ik(self, x, y, z, yaws):
        """Try every equivalent gripper yaw and seed; keep the solution closest to now."""
        cur = self.arm_positions()
        # elbow-up seed with the base already turned towards the target (robot base at y=-0.55)
        aim = [math.atan2(y + 0.55, x), 0.4, 1.2, 0.0, 1.54, 0.0]
        best = None
        for yaw in yaws:
            for seed in (aim, cur):
                q = self.ik(make_pose(x, y, z, yaw), seed)
                if q is None:
                    continue
                cost = sum((a - b) ** 2 * w for a, b, w in zip(q, cur, (3, 2, 2, 1, 1, 0.5)))
                if best is None or cost < best[0]:
                    best = (cost, q, yaw)
        return (best[1], best[2]) if best else (None, None)

    def move_joints(self, q, vel=0.8):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = 'arm'
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = vel
        r.start_state.is_diff = True
        r.workspace_parameters.header.frame_id = 'world'
        r.workspace_parameters.min_corner.x = r.workspace_parameters.min_corner.y = -2.0
        r.workspace_parameters.min_corner.z = -0.5
        r.workspace_parameters.max_corner.x = r.workspace_parameters.max_corner.y = 2.0
        r.workspace_parameters.max_corner.z = 2.5
        c = Constraints()
        for name, pos in zip(ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(pos), tolerance_above=0.01,
                tolerance_below=0.01, weight=1.0))
        r.goal_constraints = [c]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        res = self.run_action(self.move_ac, goal)
        ok = res is not None and res.error_code.val == 1
        if not ok:
            self.get_logger().error(f'MoveGroup failed: {res.error_code.val if res else "timeout"}')
        return ok

    def move_to(self, x, y, z, yaws, label, attempts=3):
        for k in range(attempts):
            q, yaw = self.best_ik(x, y, z, yaws)
            if q is None:
                self.get_logger().error(f'No IK solution for {label} ({x:.3f}, {y:.3f}, {z:.3f})')
                return None
            self.get_logger().info(f'-> {label}' + (f' (retry {k})' if k else ''))
            if self.move_joints(q):
                return yaw
        return None

    def move_linear(self, x, y, z, yaw, label, speed=0.5):
        """Straight-line Cartesian move of the TCP (approach / retreat)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state.is_diff = True
        req.group_name = 'arm'
        req.link_name = 'tcp'
        req.waypoints = [make_pose(x, y, z, yaw)]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        res = self.wait(self.cart_cli.call_async(req), 15.0)
        if not res or res.fraction < 0.95:
            self.get_logger().error(f'Cartesian path for {label} failed '
                                    f'(fraction={res.fraction if res else "n/a"})')
            return False
        traj = res.solution
        self.retime(traj.joint_trajectory, speed)
        self.get_logger().info(f'-> {label} (linear)')
        goal = ExecuteTrajectory.Goal(trajectory=traj)
        out = self.run_action(self.exec_ac, goal)
        return out is not None and out.error_code.val == 1

    @staticmethod
    def retime(jt, speed):
        """Constant-speed timing so approach/retreat moves are slow and predictable."""
        t = 0.0
        prev = None
        for pt in jt.points:
            if prev is not None:
                dq = max(abs(a - b) for a, b in zip(pt.positions, prev))
                t += max(dq / speed, 0.03)
            prev = pt.positions
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pt.velocities, pt.accelerations = [], []

    def gripper(self, q, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['finger_left_joint', 'finger_right_joint']
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[q, q], time_from_start=Duration(nanosec=500_000_000))]
        self.get_logger().info(f'Gripper {label}')
        res = self.run_action(self.grip_ac, goal, 15.0)
        return res is not None

    # ------------------------------------------------------------- perception
    def boxes_in_zone(self, max_age=0.5):
        pick_x = self.get_parameter('pick_x').value
        with self.det_lock:
            if not self.detections:
                return []
            t, msg = self.detections
        if self.sim_now() - t > max_age:
            return []
        return [p for p in msg.poses if pick_x - 0.03 <= p.position.x <= pick_x + 0.25]

    def wait_for_box(self, timeout=180.0):
        """Run the belt until a box reaches the pick zone, then stop it."""
        speed = self.get_parameter('belt_speed').value
        start = self.sim_now()
        last_cmd = -1.0
        while rclpy.ok() and self.sim_now() - start < timeout:
            if self.boxes_in_zone():
                self.set_belt(0.0)
                self.get_logger().info('Box in pick zone -> conveyor stopped')
                return True
            if self.sim_now() - last_cmd > 1.0:          # keep commanding (idempotent)
                self.set_belt(speed, repeat=1)
                last_cmd = self.sim_now()
            time.sleep(0.05)
        self.set_belt(0.0)
        return False

    def measure_box(self, duration=0.6):
        """Average detections of the box nearest the pick line while the belt is stopped."""
        pick_x = self.get_parameter('pick_x').value
        samples = []
        end = self.sim_now() + duration
        while self.sim_now() < end:
            zone = self.boxes_in_zone()
            if zone:
                p = min(zone, key=lambda p: abs(p.position.x - pick_x))
                q = p.orientation
                samples.append((p.position.x, p.position.y, p.position.z,
                                yaw_from_quat((q.x, q.y, q.z, q.w))))
            time.sleep(0.1)
        if not samples:
            return None
        n = len(samples)
        x, y, z = (sum(s[i] for s in samples) / n for i in range(3))
        # average yaw on the 90-degree circle
        sx = sum(math.sin(4 * s[3]) for s in samples)
        cx = sum(math.cos(4 * s[3]) for s in samples)
        return x, y, z, math.atan2(sx, cx) / 4

    # ------------------------------------------------------------------ cycle
    def place_slot(self, i):
        cx, cy = self.get_parameter('place_center').value
        col, row = i % 3, i // 3
        return cx + (col - 1) * 0.15, cy + (row - 0.5) * 0.16

    def pick_one(self, i):
        bt = self.get_parameter('belt_top').value
        tt = self.get_parameter('table_top').value
        size = gz_utils.BOX_SIZE
        grasp_z = bt + size[2] / 2            # TCP at box centre height
        place_z = tt + size[2] / 2 + 0.015    # release 1.5 cm above the table
        above = 0.15

        if not self.wait_for_box():
            self.get_logger().info('No more boxes arriving')
            return 'done'
        self.sim_sleep(1.0)                    # let the box settle
        m = self.measure_box()
        if m is None:
            self.get_logger().warn('Lost the box after stopping, retrying')
            return 'retry'
        x, y, ztop, yaw = m

        # ground truth from Gazebo (used to report vision accuracy and for the grip swap)
        truth = gz_utils.model_poses('box_')
        name, (tp, tq) = min(truth.items(), key=lambda kv: math.hypot(kv[1][0][0] - x, kv[1][0][1] - y))
        err = math.hypot(tp[0] - x, tp[1] - y) * 1000
        self.get_logger().info(
            f'[{name}] vision: x={x:.3f} y={y:.3f} z={ztop:.3f} yaw={math.degrees(yaw):+.1f} | '
            f'gazebo: x={tp[0]:.3f} y={tp[1]:.3f} | error {err:.1f} mm')

        yaws = [yaw + k * math.pi / 2 for k in range(4)]      # gripper/box symmetry
        yaw = self.move_to(x, y, grasp_z + above, yaws, 'pre-grasp')
        if yaw is None:
            return self.recover()
        self.gripper(GRIPPER_OPEN, 'open')
        if not self.move_linear(x, y, grasp_z, yaw, 'grasp'):
            return self.recover()
        self.gripper(GRIPPER_CLOSED, 'close')

        # grip: swap the belt box for the slot's attachable twin at the identical pose
        held = f'held_{i}'
        tp, tq = gz_utils.model_poses(name).get(name, (tp, tq))
        gz_utils.remove(name)
        gz_utils.spawn(held, gz_utils.box_sdf(held), tp, tq)
        self.attach_box(f'carried_{i}', size)
        self.sim_sleep(0.3)

        if not self.move_linear(x, y, grasp_z + above, yaw, 'lift'):
            return self.recover(holding=i)
        return self.place_box(i, held, go_back=lambda: self.move_joints(HOME))

    def place_box(self, i, held, go_back, offset_tool=(0.0, 0.0)):
        """Carry the held box to slot i, release it, verify it in Gazebo, then go_back().

        offset_tool: measured box-centre offset from the TCP, in the tool frame (x, y).
        The TCP target is shifted so that the *box* lands on the slot.
        """
        tt = self.get_parameter('table_top').value
        place_z = tt + gz_utils.BOX_SIZE[2] / 2 + 0.015    # release 1.5 cm above the table
        above = 0.15
        px, py = self.place_slot(i)
        ox, oy = offset_tool
        pyaw = None
        for cand in (0.0, math.pi, math.pi / 2, -math.pi / 2):
            c, s_ = math.cos(cand), math.sin(cand)
            # tool frame Rz(yaw)*Rx(pi): tool x -> (c, s), tool y -> (s, -c) in the world
            tx = px - (ox * c + oy * s_)
            ty = py - (ox * s_ - oy * c)
            pyaw = self.move_to(tx, ty, place_z + above, [cand], f'pre-place slot {i}', attempts=1)
            if pyaw is not None:
                px_t, py_t = tx, ty
                break
        if pyaw is None:
            return self.recover(holding=i)
        if not self.move_linear(px_t, py_t, place_z, pyaw, 'place'):
            return self.recover(holding=i)

        self.release(i)
        self.detach_box(f'carried_{i}')
        self.sim_sleep(0.25)
        self.move_linear(px_t, py_t, place_z + above, pyaw, 'retreat')
        # verify in Gazebo that the box stayed on the table (did not ride up with the gripper)
        for attempt in range(3):
            pose = gz_utils.model_poses(held).get(held)
            if pose and pose[0][2] < tt + 0.1:
                break
            self.get_logger().warn(f'{held} still on the gripper -> releasing again')
            self.move_linear(px_t, py_t, place_z, pyaw, 'place (retry)')
            self.release(i)
            self.sim_sleep(0.5)
            self.move_linear(px_t, py_t, place_z + above, pyaw, 'retreat')
        self.gripper(GRIPPER_OPEN, 'open')
        pose = gz_utils.model_poses(held).get(held)
        if pose and pose[0][2] >= tt + 0.1:
            # last resort: a stuck box would collide with every later pick
            self.get_logger().error(f'{held} could not be released -> removing it')
            gz_utils.remove(held)
        go_back()
        p = gz_utils.model_poses(held).get(held)
        ok = bool(p and abs(p[0][0] - px) < 0.04 and abs(p[0][1] - py) < 0.04 and p[0][2] < tt + 0.1)
        self.get_logger().info(f'{held} final pose {tuple(round(v, 3) for v in p[0]) if p else None} '
                               f'(slot {i} at {px:.2f}, {py:.2f}) -> {"OK" if ok else "MISPLACED"}')
        return 'placed' if ok else 'misplaced'


    # ------------------------------------------------------ pick on the fly
    #  Conveyor tracking: the belt speed v is known (a real line would read an encoder),
    #  and every detection carries the camera timestamp, so a box seen at x_d at time t_d
    #  is at x_d + v * (t - t_d). The arm waits above the belt at x = x_w. For the next
    #  box we precompute a Cartesian trajectory that moves with the belt (x = x_w + v*t)
    #  while descending, holding (gripper closes) and lifting, and hand it to the joint
    #  trajectory controller with a *future start time*: the moment the box reaches x_w.
    T_DESC, T_HOLD, T_LIFT, DT = 1.2, 1.2, 0.8, 0.1

    def wait_pose(self):
        bt = self.get_parameter('belt_top').value
        return (self.get_parameter('track_start_x').value, 0.0,
                bt + gz_utils.BOX_SIZE[2] / 2 + 0.15)

    def go_to_wait(self):
        x, y, z = self.wait_pose()
        yaw = self.move_to(x, y, z, [0.0, math.pi], 'wait above belt')
        self.at_wait = yaw is not None
        self.wait_yaw = yaw
        return self.at_wait

    def next_upstream_box(self, timeout=120.0):
        """Return (x_ref, y, t_ref) of the next box far enough upstream to intercept."""
        v = self.get_parameter('track_speed').value
        xw = self.get_parameter('track_start_x').value
        lead = v * 1.5                       # need >= 1.5 s to plan and start in time
        start = self.sim_now()
        samples, t_ref = [], None
        raw = []                             # (stamp, measured x) to cross-check belt speed
        last_stamp = None
        while rclpy.ok() and self.sim_now() - start < timeout:
            with self.det_lock:
                det = self.detections
            if det is None or det[1].header.stamp == last_stamp:
                time.sleep(0.02)
                continue
            msg = det[1]
            last_stamp = msg.header.stamp
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t_ref is None:
                t_ref = ts
            # predict every detection to the common reference time t_ref
            cands = [(p.position.x + v * (t_ref - ts), p.position.y) for p in msg.poses
                     if -0.4 < p.position.x < xw - lead]
            if samples:   # keep following the same box (closest to the running estimate)
                mx = sum(c[0] for c in samples) / len(samples)
                cands = [c for c in cands if abs(c[0] - mx) < 0.05]
            elif cands:
                cands = [max(cands)]          # most downstream box first
            if cands:
                samples.append(cands[0])
                raw.append((ts, cands[0][0] - v * (t_ref - ts)))
            if len(samples) >= 4 or (samples and not cands):
                if len(samples) < 2:          # left the window after a single frame: start over
                    samples, t_ref = [], None
                    continue
                x = sum(c[0] for c in samples) / len(samples)
                y = sum(c[1] for c in samples) / len(samples)
                if len(raw) >= 3 and raw[-1][0] > raw[0][0]:
                    v_meas = (raw[-1][1] - raw[0][1]) / (raw[-1][0] - raw[0][0])
                    self.get_logger().info(f'  belt speed: commanded {v:.3f} m/s, '
                                           f'measured by vision {v_meas:.3f} m/s')
                return x, y, t_ref
            if not cands and not samples:
                t_ref = None
        return None

    def tracking_waypoints(self, y_box, yaw):
        """TCP poses every DT seconds: follow the belt, descend, hold, lift."""
        v = self.get_parameter('track_speed').value
        xw, y0, z_pre = self.wait_pose()
        z_g = self.get_parameter('belt_top').value + gz_utils.BOX_SIZE[2] / 2
        smooth = lambda u: 3 * u * u - 2 * u * u * u   # noqa: E731
        total = self.T_DESC + self.T_HOLD + self.T_LIFT
        pts = []
        n = int(round(total / self.DT))
        for k in range(1, n + 1):
            t = k * self.DT
            u = min(t / self.T_DESC, 1.0)
            if t <= self.T_DESC:
                z = z_pre - (z_pre - z_g) * smooth(u)
            elif t <= self.T_DESC + self.T_HOLD:
                z = z_g
            else:
                z = z_g + (z_pre - z_g) * smooth(min((t - self.T_DESC - self.T_HOLD) / self.T_LIFT, 1.0))
            # line up with the box's lane first, so the fingertips never come down on its edge
            uy = min(t / (0.4 * self.T_DESC), 1.0)
            pts.append(make_pose(xw + v * t, y0 + (y_box - y0) * smooth(uy), z, yaw))
        return pts

    def plan_tracking(self, y_box, yaw):
        """IK for every waypoint, each seeded with the previous solution (no branch flips)."""
        from trajectory_msgs.msg import JointTrajectory
        wps = self.tracking_waypoints(y_box, yaw)
        q = self.arm_positions()
        jt = JointTrajectory(joint_names=list(ARM_JOINTS))
        jt.points.append(JointTrajectoryPoint(positions=list(q)))
        for k, wp in enumerate(wps):
            qn = self.ik(wp, q, avoid_collisions=False, timeout_ns=50_000_000)
            if qn is None or max(abs(a - b) for a, b in zip(qn, q)) > 0.35:
                self.get_logger().error(f'Tracking IK failed at waypoint {k + 1}/{len(wps)} '
                                        f'({wp.position.x:.3f}, {wp.position.y:.3f}, {wp.position.z:.3f})')
                return None
            jt.points.append(JointTrajectoryPoint(positions=list(qn)))
            q = qn
        # time each point exactly: point k is waypoint k at k*DT; velocities by finite differences
        n = len(jt.points)
        for k, pt in enumerate(jt.points):
            t = k * self.DT
            sec, ns = divmod(int(round(t * 1e9)), 1_000_000_000)
            pt.time_from_start = Duration(sec=sec, nanosec=ns)
            a, b = max(k - 1, 0), min(k + 1, n - 1)
            dt = (b - a) * self.DT
            pt.velocities = [(jb - ja) / dt for ja, jb in zip(jt.points[a].positions, jt.points[b].positions)]
            pt.accelerations = []
        jt.points[0].velocities = [0.0] * len(jt.joint_names)
        jt.points[-1].velocities = [0.0] * len(jt.joint_names)
        return jt

    @staticmethod
    def stamp(t):
        return TimeMsg(sec=int(t), nanosec=int((t % 1) * 1e9))

    def pick_moving(self, i):
        v = self.get_parameter('track_speed').value
        xw = self.get_parameter('track_start_x').value
        bt = self.get_parameter('belt_top').value
        self.set_belt(v)
        if not self.at_wait and not self.go_to_wait():
            return self.recover()
        self.gripper(GRIPPER_OPEN, 'open')

        det = self.next_upstream_box()
        if det is None:
            self.get_logger().info('No more boxes arriving')
            return 'done'
        x_ref, y, t_ref = det
        t_meet = t_ref + (xw - x_ref) / v          # when the box reaches the gripper
        self.get_logger().info(f'Box tracked: y={y:+.3f}, reaches x={xw:.2f} in '
                               f'{t_meet - self.sim_now():.1f} s')
        t_plan = time.time()
        jt = self.plan_tracking(y, self.wait_yaw)
        self.get_logger().info(f'Tracking trajectory planned in {time.time() - t_plan:.2f} s')
        if jt is None:
            return 'retry'
        if t_meet - self.sim_now() < 0.3:
            self.get_logger().warn('Too late to intercept this box, letting it pass')
            return 'retry'

        # arm and gripper goals with future start times (controllers run on sim time)
        jt.header.stamp = self.stamp(t_meet)
        arm_goal = FollowJointTrajectory.Goal(trajectory=jt)
        arm_fut = self.arm_ac.send_goal_async(arm_goal)
        g = FollowJointTrajectory.Goal()
        g.trajectory.header.stamp = self.stamp(t_meet + self.T_DESC)
        g.trajectory.joint_names = ['finger_left_joint', 'finger_right_joint']
        g.trajectory.points = [JointTrajectoryPoint(positions=[GRIPPER_CLOSED] * 2,
                                                    time_from_start=Duration(nanosec=400_000_000))]
        self.grip_ac.send_goal_async(g)
        self.get_logger().info('-> tracking the box on the moving belt')

        # grip: once the fingers are closed around the moving box, swap it for the attachable twin
        t_swap = t_meet + self.T_DESC + 0.45
        logged = False
        while rclpy.ok() and self.sim_now() < t_swap:
            if not logged and self.sim_now() >= t_meet:
                tr = gz_utils.model_poses('box_')
                bx = max((p[0] for p, _ in tr.values() if p[0] < xw + 0.3 and abs(p[1] - y) < 0.03), default=None)
                if bx is not None:
                    self.get_logger().info(f'  intercept: box at x={bx:.3f} when the gripper arrived '
                                           f'(target {xw:.3f}, error {(bx - xw) * 1000:+.1f} mm)')
                logged = True
            time.sleep(0.01)
        held = f'held_{i}'
        truth = gz_utils.model_poses('box_')
        x_now = xw + v * (self.sim_now() - t_meet)
        name, (tp, tq) = min(truth.items(), key=lambda kv: math.hypot(kv[1][0][0] - x_now, kv[1][0][1] - y))
        err = math.hypot(tp[0] - x_now, tp[1] - y) * 1000
        # the gripper keeps moving during remove+spawn: place the twin where the box will be
        # when it appears (latency measured on every swap, in sim time)
        t0 = self.sim_now()
        gz_utils.remove(name)
        gz_utils.spawn(held, gz_utils.box_sdf(held), (tp[0] + v * self.swap_lat, tp[1], tp[2]), tq)
        self.swap_lat = 0.5 * self.swap_lat + 0.5 * (self.sim_now() - t0)
        self.get_logger().info(f'[{name}] gripped on the fly (prediction error {err:.1f} mm)')
        self.attach_box(f'carried_{i}', gz_utils.BOX_SIZE)

        gh = self.wait(arm_fut, 10.0)
        res = self.wait(gh.get_result_async(), 30.0) if gh and gh.accepted else None
        if res is None or res.result.error_code != 0:
            self.get_logger().error(f'Tracking trajectory failed: {res.result.error_code if res else "rejected"}')
            return self.recover(holding=i)
        self.at_wait = False

        p = gz_utils.model_poses(held).get(held)
        if not p or p[0][2] < bt + 0.1:
            self.get_logger().error(f'{held} not lifted (grip missed) -> removing it')
            gz_utils.remove(held)
            self.detach_box(f'carried_{i}')
            return 'failed'
        # measure where the box actually sits in the gripper (part-in-gripper offset)
        off = (0.0, 0.0)
        try:
            tcp = self.tf_buffer.lookup_transform('world', 'tcp', rclpy.time.Time()).transform.translation
            dx, dy = p[0][0] - tcp.x, p[0][1] - tcp.y
            c, s_ = math.cos(self.wait_yaw), math.sin(self.wait_yaw)
            off = (dx * c + dy * s_, dx * s_ - dy * c)        # world -> tool frame
            self.get_logger().info(f'  box offset in gripper: {off[0] * 1000:+.1f}, {off[1] * 1000:+.1f} mm '
                                   f'(compensated when placing)')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'offset measurement skipped: {e}')
        return self.place_box(i, held, go_back=self.go_to_wait, offset_tool=off)

    def release(self, i):
        for _ in range(3):
            self.detach_pubs[i].publish(Empty())
            time.sleep(0.1)

    def recover(self, holding=None):
        self.get_logger().warn('Step failed: returning home')
        if holding is not None:      # never go home with a box still attached
            self.release(holding)
        self.move_joints(HOME)
        self.at_wait = False
        return 'failed' if holding is not None else 'retry'

    def run(self):
        self.get_logger().info('Waiting for MoveIt, controllers and camera...')
        for c in (self.ik_cli, self.cart_cli, self.scene_cli):
            while rclpy.ok() and not c.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'  waiting for {c.srv_name}')
        for a in (self.move_ac, self.exec_ac, self.grip_ac, self.arm_ac):
            while rclpy.ok() and not a.wait_for_server(timeout_sec=2.0):
                self.get_logger().info(f'  waiting for action {a._action_name}')
        while rclpy.ok() and (self.joint_state is None or self.detections is None):
            time.sleep(0.2)

        self.setup_scene()
        self.gripper(GRIPPER_OPEN, 'open')
        self.move_joints(HOME)
        moving = self.get_parameter('mode').value == 'moving'
        self.get_logger().info('=== Cell ready: starting pick-and-place '
                               f'({"on the moving belt" if moving else "stop-and-pick"}) ===')

        slots = self.get_parameter('slots').value
        i = 0              # next gripper slot (each DetachableJoint can be used once)
        placed = 0
        t0 = self.sim_now()
        while rclpy.ok() and i < slots:
            result = self.pick_moving(i) if moving else self.pick_one(i)
            if result == 'done':
                break
            if result == 'retry':
                continue
            i += 1         # slot consumed (placed, misplaced or dropped)
            if result == 'placed':
                placed += 1
            self.get_logger().info(f'=== Box {i}/{slots}: {result.upper()} '
                                   f'({self.sim_now() - t0:.0f} s sim time) ===')
        self.set_belt(0.0)
        self.get_logger().info(f'=== Done: {placed}/{i} boxes placed correctly ===')


def main():
    rclpy.init()
    node = PickPlace()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
