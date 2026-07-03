#!/usr/bin/env python3

import rclpy
import math
import time
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveGap(Node):

    def __init__(self):
        super().__init__('reactive_gap')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        self.start_position = None
        self.last_lap_time = time.time()

        self.lap_count = 0
        self.in_start_zone = False

        self.x = 0.0
        self.y = 0.0

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.start_position is None:
            self.start_position = (self.x, self.y)
            return

        dx = self.x - self.start_position[0]
        dy = self.y - self.start_position[1]

        distance = math.sqrt(dx*dx + dy*dy)

        if distance < 1.5:

            if not self.in_start_zone:

                self.in_start_zone = True

                self.lap_count += 1

                current_time = time.time()

                lap_time = current_time - self.last_lap_time

                self.last_lap_time = current_time

                self.get_logger().info(
                    f'VUELTA {self.lap_count} | '
                    f'TIEMPO: {lap_time:.2f} s'
                )

        else:
            self.in_start_zone = False

    def preprocess_lidar(self, ranges):

        proc = np.array(ranges)

        proc = np.clip(proc, 0, 10)

        window = 5

        proc = np.convolve(
            proc,
            np.ones(window)/window,
            mode='same')

        return proc

    def scan_callback(self, scan_msg):

        ranges = self.preprocess_lidar(scan_msg.ranges)

        closest = np.argmin(ranges)

        bubble_radius = 120

        start = max(0, closest - bubble_radius)
        end = min(len(ranges)-1, closest + bubble_radius)

        ranges[start:end] = 0

        gap_start = 0
        gap_end = 0

        max_len = 0

        current_start = 0

        in_gap = False

        for i in range(len(ranges)):

            if ranges[i] > 1.0:

                if not in_gap:
                    current_start = i
                    in_gap = True

            else:

                if in_gap:

                    length = i - current_start

                    if length > max_len:
                        max_len = length
                        gap_start = current_start
                        gap_end = i

                    in_gap = False

        best_index = (gap_start + gap_end)//2

        angle = (
            scan_msg.angle_min +
            best_index * scan_msg.angle_increment
        )

        steering = angle

        steering = np.clip(
            steering,
            -0.4,
            0.4
        )

        abs_angle = abs(steering)

        if abs_angle < 0.08:
            speed = 8.0

        elif abs_angle < 0.20:
            speed = 6.0

        elif abs_angle < 0.35:
            speed = 4.0

        else:
            speed = 2.5

        msg = AckermannDriveStamped()

        msg.drive.speed = speed
        msg.drive.steering_angle = steering

        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 

if __name__ == '__main__':
    main()