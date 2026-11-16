#!/usr/bin/env python3
"""Feeds boxes onto the start of the conveyor and drives the belt.

A new box is spawned every `spacing` metres of belt travel, so boxes never pile
up while the belt is stopped for a pick. The belt itself is driven by the
pick-and-place node via /conveyor/speed (std_msgs/Float64, m/s).

Belt model: every box on the belt carries a VelocityControl plugin listening to
/conveyor/box_cmd_vel. We republish the belt speed there at 5 Hz, so all boxes
move (and stop) together exactly like product riding a real belt.
"""
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import gz_utils

BELT_TOP_Z = 0.725   # conveyor pose z (0.70) + half belt thickness (0.025)


class BoxSpawner(Node):
    def __init__(self):
        super().__init__('box_spawner')
        self.declare_parameter('max_boxes', 6)
        self.declare_parameter('spacing', 0.9)       # m of belt travel between boxes
        self.declare_parameter('spawn_x', -1.3)

        self.count = 0
        self.speed = 0.0
        self.travel = 1e9                            # spawn first box right away
        self.last_t = None

        self.create_subscription(Float64, '/conveyor/speed', self.on_speed, 10)
        self.belt_pub = self.create_publisher(Twist, gz_utils.CONVEYOR_TOPIC, 10)
        self.create_timer(0.2, self.tick)

    def on_speed(self, msg):
        self.speed = msg.data

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:                       # /clock not received yet
            return
        cmd = Twist()
        cmd.linear.x = float(self.speed)
        self.belt_pub.publish(cmd)
        if self.last_t is not None:
            self.travel += abs(self.speed) * (now - self.last_t)
        self.last_t = now

        if self.count >= self.get_parameter('max_boxes').value:
            return
        if self.travel < self.get_parameter('spacing').value:
            return
        name = f'box_{self.count}'
        y = random.uniform(-0.06, 0.06)
        z = BELT_TOP_Z + gz_utils.BOX_SIZE[2] / 2 + 0.005
        if gz_utils.spawn(name, gz_utils.box_sdf(name, on_belt=True), (self.get_parameter('spawn_x').value, y, z)):
            self.get_logger().info(f'Spawned {name} (y={y:+.3f})')
            self.count += 1
            self.travel = 0.0
        else:
            self.get_logger().warn(f'Failed to spawn {name}, retrying')


def main():
    rclpy.init()
    rclpy.spin(BoxSpawner())


if __name__ == '__main__':
    main()
