#!/usr/bin/env python3
"""
F1Tenth Opponent Controller
Mueve el segundo vehículo como obstáculo dinámico lento.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math


WAYPOINTS = [
    (-0.10,  -0.50), ( 5.71, -21.61), ( 8.06, -25.40), (10.86, -26.03),
    (15.17, -22.14), (16.66, -21.63), (22.65, -24.38), (26.89, -24.72),
    (30.30, -23.37), (33.80, -20.50), (35.69, -17.26), (42.57,  10.25),
    (48.37,  31.40), (47.68,  35.22), (34.97,  36.87), (30.73,  34.48),
    (12.53,  10.00), ( 3.04,  12.60), ( 1.53,  14.92), ( 0.06,  24.97),
    ( 1.63,  26.42), ( 7.09,  24.15), (10.06,  24.38), (11.53,  27.42),
    (10.41,  29.98), ( 4.57,  36.90), ( 2.70,  42.54), ( 3.70,  44.85),
    ( 6.81,  44.36), (13.06,  37.69), (18.79,  36.62), (25.42,  42.44),
    (30.80,  51.81), (31.13,  54.08), (23.88,  57.66), (15.91,  57.01),
    ( 6.41,  53.18), ( 2.00,  49.56), (-0.86,  42.57)
]


class P:
    # Lectura LiDAR
    CLIP = 10.0
    MAX_STEER = 0.35
    STEER_SMOOTH = 0.25

    # Follow the Gap
    SAFE_GAP_DIST = 4.0
    BUBBLE_R = 0.80
    MAX_GAP_LOOK_ANGLE = 1.00

    # Distancias de seguridad
    DIST_SAFE = 6.0
    DIST_CAUTION = 0.90

    # Repulsión
    REPULSION_K = 1.10

    # Velocidades bajas para obstáculo dinámico
    SPEED_MAX = 1.8
    SPEED_CORNER = 1.2
    SPEED_DANGER = 0.5
    SPEED_BRAKE_K = 0.90


class OpponentRacer(Node):
    def __init__(self):
        super().__init__('opponent_racer')

        # Tópicos del segundo vehículo
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/opp_drive',
            10
        )

        self.create_subscription(
            LaserScan,
            '/opp_scan',
            self.on_scan,
            10
        )

        self.create_subscription(
            Odometry,
            '/opp_racecar/odom',
            self.on_odom,
            10
        )

        self.wp_xy = np.array(WAYPOINTS, dtype=np.float32)
        self.n_wp = len(WAYPOINTS)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.wp_idx = 0
        self.wp_initialized = False

        self.prev_steer = 0.0
        self.prev_speed = P.SPEED_CORNER

        self.angles = None
        self.nr = None

        self.get_logger().info("OPPONENT RACER ACTIVADO - vehículo 2 en movimiento ")

    def initialize_waypoint(self):
        dists = np.linalg.norm(self.wp_xy - np.array([self.x, self.y]), axis=1)
        nearest_idx = int(np.argmin(dists))
        self.wp_idx = (nearest_idx + 1) % self.n_wp
        self.wp_initialized = True
        self.get_logger().info(
            f"Waypoint inicial del oponente: {self.wp_idx}"
        )
 
    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        if not self.wp_initialized:
            self.initialize_waypoint()

        target_wp = self.wp_xy[self.wp_idx]
        dist_to_wp = math.hypot(target_wp[0] - self.x, target_wp[1] - self.y)

        if dist_to_wp < 3.5:
            self.wp_idx = (self.wp_idx + 1) % self.n_wp

    def on_scan(self, scan):
        if not self.wp_initialized:
            return

        raw = np.array(scan.ranges, dtype=np.float32)

        if self.angles is None:
            self.nr = len(raw)
            self.angles = np.linspace(scan.angle_min, scan.angle_max, self.nr)

        r = np.clip(raw, 0.0, P.CLIP)
        r = np.where(np.isfinite(r), r, P.CLIP)

        # Distancia frontal
        front_mask = np.abs(self.angles) < 0.15
        front_dist = float(np.mean(r[front_mask]))

        # Estado de velocidad
        if front_dist >= P.DIST_SAFE:
            target_speed = P.SPEED_MAX
            state = "SEGURA"
        elif P.DIST_CAUTION <= front_dist < P.DIST_SAFE:
            target_speed = P.SPEED_CORNER
            state = "CUIDADO"
        else:
            target_speed = P.SPEED_DANGER
            state = "PELIGRO"

        # Dirección hacia waypoint
        wp = self.wp_xy[self.wp_idx]
        goal_angle = math.atan2(wp[1] - self.y, wp[0] - self.x) - self.yaw
        goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi
        goal_angle = np.clip(goal_angle, -0.7, 0.7)

        # Follow the Gap
        driving_mask = np.abs(self.angles) < P.MAX_GAP_LOOK_ANGLE
        proc_lidar = np.where(driving_mask, r, 0.0)

        closest_idx = int(np.argmin(np.where(driving_mask, proc_lidar, P.CLIP)))
        closest_dist = proc_lidar[closest_idx]

        if closest_dist < 3.0:
            angle_inc = (self.angles[-1] - self.angles[0]) / (self.nr - 1)
            b_half = math.atan2(P.BUBBLE_R, max(closest_dist, 0.1))
            b_idx = int(b_half / angle_inc) + 1

            left = max(0, closest_idx - b_idx)
            right = min(self.nr, closest_idx + b_idx + 1)
            proc_lidar[left:right] = 0.0

        free_rays = proc_lidar >= P.SAFE_GAP_DIST

        if not np.any(free_rays):
            gap_steer = float(self.angles[np.argmax(proc_lidar)])
        else:
            free_indices = np.where(free_rays)[0]
            best_idx = free_indices[
                np.argmin(np.abs(self.angles[free_indices] - goal_angle))
            ]
            gap_steer = float(self.angles[best_idx])

        # Repulsión en peligro
        repulsion_steer_offset = 0.0

        if state == "PELIGRO":
            active_mask = np.abs(self.angles) < 1.30
            active_indices = np.where(active_mask)[0]

            for idx in active_indices:
                dist_obstacle = r[idx]

                if dist_obstacle < P.DIST_CAUTION:
                    angle = self.angles[idx]
                    force = P.REPULSION_K * (
                        (P.DIST_CAUTION - dist_obstacle) /
                        (dist_obstacle + 1e-3)
                    )
                    repulsion_steer_offset -= force * np.sign(angle) * math.cos(angle)

        chosen_steer = gap_steer + repulsion_steer_offset
        chosen_steer = float(np.clip(chosen_steer, -P.MAX_STEER, P.MAX_STEER))

        steer = P.STEER_SMOOTH * self.prev_steer + \
            (1.0 - P.STEER_SMOOTH) * chosen_steer
        self.prev_steer = steer
        if target_speed < self.prev_speed:
            speed = (1.0 - P.SPEED_BRAKE_K) * self.prev_speed + \
                P.SPEED_BRAKE_K * target_speed
        else:
            speed = 0.30 * self.prev_speed + 0.70 * target_speed
        if abs(steer) > 0.25:
            speed = min(speed, 0.9)

        self.prev_speed = speed

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)

        self.pub.publish(msg)

        self.get_logger().info(
            f"OPP -> speed={speed:.2f}, steer={steer:.2f}, wp={self.wp_idx}, state={state}",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = OpponentRacer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()