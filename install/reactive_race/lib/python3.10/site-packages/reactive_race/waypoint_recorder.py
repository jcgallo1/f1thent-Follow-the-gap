#!/usr/bin/env python3

import csv
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class WaypointRecorder(Node):

    def __init__(self):

        super().__init__('waypoint_recorder')

        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.waypoints = []

        self.last_x = None
        self.last_y = None

        # guardar un punto cada 0.5 m
        self.min_distance = 0.5

        self.get_logger().info(
            'Grabando waypoints...'
        )

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None:

            self.last_x = x
            self.last_y = y

            self.waypoints.append((x, y))

            return

        distance = math.sqrt(
            (x - self.last_x) ** 2 +
            (y - self.last_y) ** 2
        )

        if distance >= self.min_distance:

            self.waypoints.append((x, y))

            self.last_x = x
            self.last_y = y

            print(
                f"Waypoint {len(self.waypoints)}: "
                f"{x:.2f}, {y:.2f}"
            )

    def save_waypoints(self):

        with open('waypoints.csv', 'w', newline='') as file:

            writer = csv.writer(file)

            writer.writerow(['x', 'y'])

            for wp in self.waypoints:

                writer.writerow(wp)

        self.get_logger().info(
            f'{len(self.waypoints)} waypoints guardados'
        )


def main(args=None):

    rclpy.init(args=args)

    node = WaypointRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.save_waypoints()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()