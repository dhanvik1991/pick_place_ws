#!/usr/bin/env python3
"""Detects boxes in the overhead camera image and publishes their 3D poses.

Pipeline
  1. OpenCV: HSV threshold on the box colour -> contours -> minAreaRect
  2. Pixel (u, v) -> ray in the camera optical frame using the intrinsics K
  3. TF2: transform the ray into the `world` frame (camera calibration)
  4. Intersect the ray with the plane z = box top height  ->  X, Y, Z
  5. Box yaw from the rectangle edge, projected the same way

Publishes
  /boxes/detections   geometry_msgs/PoseArray (frame: world, pose = box top centre)
  /boxes/debug_image  sensor_msgs/Image with overlays
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.declare_parameter('camera_frame', 'camera_optical')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('plane_z', 0.805)          # belt top 0.725 + box height 0.08
        self.declare_parameter('hsv_low', [0, 150, 60])
        self.declare_parameter('hsv_high', [22, 255, 255])
        self.declare_parameter('min_area', 300)
        self.declare_parameter('max_area', 11000)   # one 0.12 m box at 0.8 m is ~7800 px

        self.bridge = CvBridge()
        self.K = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/image', self.on_image, 10)
        self.pub = self.create_publisher(PoseArray, '/boxes/detections', 10)
        self.dbg_pub = self.create_publisher(Image, '/boxes/debug_image', 10)
        self.get_logger().info('Box detector ready')

    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def pixel_to_world(self, u, v, R, t, plane_z):
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        ray_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray_w = R @ ray_cam
        if abs(ray_w[2]) < 1e-6:
            return None
        s = (plane_z - t[2]) / ray_w[2]
        return t + s * ray_w

    def on_image(self, msg):
        if self.K is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('world_frame').value,
                self.get_parameter('camera_frame').value, Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=5.0)
            return
        R = quat_to_mat(tf.transform.rotation)
        tr = tf.transform.translation
        t = np.array([tr.x, tr.y, tr.z])
        plane_z = self.get_parameter('plane_z').value

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.get_parameter('hsv_low').value, dtype=np.uint8),
                           np.array(self.get_parameter('hsv_high').value, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        out = PoseArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.get_parameter('world_frame').value
        dbg = img.copy()
        for c in contours:
            if cv2.contourArea(c) < self.get_parameter('min_area').value:
                continue
            bx, by, bw, bh = cv2.boundingRect(c)
            H, W = mask.shape
            if bx <= 2 or by <= 2 or bx + bw >= W - 2 or by + bh >= H - 2:
                continue        # cut off by the image edge: centroid would be biased
            (u, v), (w, h), ang = cv2.minAreaRect(c)
            # reject blobs that are not one square box (merged boxes, box on the gripper...)
            if max(w, h) / max(min(w, h), 1) > 1.3 or w * h > self.get_parameter('max_area').value:
                continue
            p = self.pixel_to_world(u, v, R, t, plane_z)
            # a second point along one rectangle edge gives the yaw in world
            a = math.radians(ang)
            p2 = self.pixel_to_world(u + 20 * math.cos(a), v + 20 * math.sin(a), R, t, plane_z)
            if p is None or p2 is None:
                continue
            yaw = math.atan2(p2[1] - p[1], p2[0] - p[0])
            yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4   # square box: 90 deg symmetric

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
            pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            out.poses.append(pose)

            box = cv2.boxPoints(((u, v), (w, h), ang)).astype(np.int32)
            cv2.drawContours(dbg, [box], 0, (0, 255, 0), 2)
            cv2.circle(dbg, (int(u), int(v)), 4, (0, 0, 255), -1)
            cv2.putText(dbg, f'x={p[0]:.3f} y={p[1]:.3f} yaw={math.degrees(yaw):.0f}',
                        (int(u) - 80, int(v) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        self.pub.publish(out)
        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        dbg_msg.header = msg.header
        self.dbg_pub.publish(dbg_msg)


def main():
    rclpy.init()
    rclpy.spin(BoxDetector())


if __name__ == '__main__':
    main()
