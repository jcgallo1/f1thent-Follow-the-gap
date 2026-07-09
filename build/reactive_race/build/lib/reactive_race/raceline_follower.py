#!/usr/bin/env python3
"""
F1Tenth Controller - Follow the Gap - Jcgallo

Arquitectura:
- Waypoints = guía principal.
- Follow the Gap = solo cuando el camino al waypoint está bloqueado.
- Follow Wall Assist = solo apoyo lateral en AVOID/PELIGRO.
""" 

import rclpy
from rclpy.node import Node

import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


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
    # ── VARIABLES DE CONTROL FÍSICO ──
    CLIP          = 10.0
    MAX_STEER     = 0.41
    STEER_SMOOTH  = 0.15

    # ── FOLLOW THE GAP ORIGINAL ──
    SAFE_GAP_DIST = 4.5 
    BUBBLE_R      = 0.90
    MAX_GAP_LOOK_ANGLE = 1.05

    # ── ZONAS ORIGINALES ──
    DIST_SAFE     = 4.5     
    DIST_CAUTION  = 1.50       

    # ── REPULSIÓN ORIGINAL ──
    REPULSION_K   = 1.25 
    MAX_REPULSION_OFFSET = 0.22

    # ── VELOCIDADES ORIGINALES ──
    SPEED_MAX     = 8.5  
    SPEED_CORNER  = 4.7 
    SPEED_DANGER  = 2.5
    SPEED_BRAKE_K = 0.95 
 
    # ── WAYPOINT + FOLLOW THE GAP PREVENTIVO ──
    PATH_CHECK_WIDTH = 0.20
    FRONT_CHECK_WIDTH = 0.25

    AVOID_ENTER_DIST = 3.2
    AVOID_EXIT_DIST  = 4.2 

    AVOID_FREE_DIST = 1.35
    GAP_MIN_LEN = 7

    GAP_GOAL_W = 2.80
    GAP_CENTER_W = 1.00
    GAP_DIST_W = 0.60
    GAP_SMOOTH_W = 0.25

    # ── BURBUJA FRENTE A OBSTÁCULOS ──
    BUBBLE_TRIGGER_DIST = 3.5
    BUBBLE_PATH_WIDTH = 0.35

    # ── FOLLOW WALL ASSIST SOLO EN AVOID/PELIGRO ──
    WALL_DESIRED_DIST = 1.00
    WALL_ACTIVE_DIST = 0.90
    WALL_LOOKAHEAD = 1.20
    WALL_THETA = math.radians(45.0)

    WALL_KP = 0.14
    WALL_MAX_CORRECTION = 0.10


class UnifiedRacer(Node):
    def __init__(self):
        super().__init__('unified_hybrid_racer')
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.on_odom, 10)

        self.wp_xy = np.array(WAYPOINTS, dtype=np.float32)
        self.n_wp = len(WAYPOINTS)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0 
        self.wp_idx = 0
        self.prev_steer = 0.0
        self.prev_speed = P.SPEED_CORNER

        self.angles = None
        self.nr = None

        self.avoid_mode = False

        self.lap_count = 0
        self.lap_start_time = None
        self.best_lap_time = float('inf')
        self.crossed_checkpoint = False

        self.get_logger().info("INICIA CARRERA") 

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def get_range_at_angle(self, r, angle):
        idx = int(np.argmin(np.abs(self.angles - angle)))
        return float(r[idx])

    def percentile_dist(self, r, mask, percentile=10):
        values = r[mask]

        if values.size == 0:
            return P.CLIP

        return float(np.percentile(values, percentile))

    def compute_goal_angle(self):
        wp = self.wp_xy[self.wp_idx]

        goal_angle = math.atan2(
            float(wp[1]) - self.y,
            float(wp[0]) - self.x
        ) - self.yaw

        goal_angle = self.normalize_angle(goal_angle)
        goal_angle = float(np.clip(goal_angle, -0.75, 0.75))

        return goal_angle

    def follow_wall_assist(self, r):
        theta = P.WALL_THETA
        lookahead = P.WALL_LOOKAHEAD
        desired = P.WALL_DESIRED_DIST

        correction = 0.0

        b_right = self.get_range_at_angle(r, -math.pi / 2.0)
        a_right = self.get_range_at_angle(r, -math.pi / 4.0)

        b_left = self.get_range_at_angle(r, math.pi / 2.0)
        a_left = self.get_range_at_angle(r, math.pi / 4.0)

        if b_right < P.WALL_ACTIVE_DIST and a_right < P.CLIP:
            alpha_r = math.atan2(
                a_right * math.cos(theta) - b_right,
                max(a_right * math.sin(theta), 1e-3)
            )

            dt_right = b_right * math.cos(alpha_r)
            dt1_right = dt_right + lookahead * math.sin(alpha_r)

            error_right = desired - dt1_right

            if error_right > 0.0:
                correction += P.WALL_KP * error_right

        if b_left < P.WALL_ACTIVE_DIST and a_left < P.CLIP:
            alpha_l = math.atan2(
                a_left * math.cos(theta) - b_left,
                max(a_left * math.sin(theta), 1e-3)
            )

            dt_left = b_left * math.cos(alpha_l)
            dt1_left = dt_left + lookahead * math.sin(alpha_l)

            error_left = desired - dt1_left

            if error_left > 0.0:
                correction -= P.WALL_KP * error_left

        return float(np.clip(correction, -P.WALL_MAX_CORRECTION, P.WALL_MAX_CORRECTION))

    def apply_obstacle_bubble(self, r, goal_angle):
        driving_mask = np.abs(self.angles) < P.MAX_GAP_LOOK_ANGLE
        proc_lidar = np.where(driving_mask, r.copy(), 0.0)

        path_bubble_mask = (
            driving_mask &
            (np.abs(self.angles - goal_angle) < P.BUBBLE_PATH_WIDTH) &
            (r < P.BUBBLE_TRIGGER_DIST)
        )

        candidate_indices = np.where(path_bubble_mask)[0]

        if candidate_indices.size == 0:
            front_bubble_mask = (
                driving_mask &
                (np.abs(self.angles) < P.FRONT_CHECK_WIDTH) &
                (r < P.BUBBLE_TRIGGER_DIST)
            )
            candidate_indices = np.where(front_bubble_mask)[0]

        if candidate_indices.size == 0:
            return proc_lidar

        closest_idx = candidate_indices[int(np.argmin(r[candidate_indices]))]
        closest_dist = max(float(r[closest_idx]), 0.10)

        angle_inc = abs((self.angles[-1] - self.angles[0]) / max(1, self.nr - 1))

        bubble_angle = math.atan2(P.BUBBLE_R, closest_dist)
        bubble_idx = int(bubble_angle / angle_inc) + 1

        left = max(0, closest_idx - bubble_idx)
        right = min(self.nr, closest_idx + bubble_idx + 1)

        proc_lidar[left:right] = 0.0

        return proc_lidar

    def choose_best_gap(self, proc_lidar, goal_angle):
        free_rays = proc_lidar >= P.SAFE_GAP_DIST
        free_indices = np.where(free_rays)[0]

        if free_indices.size == 0:
            free_rays = proc_lidar >= P.AVOID_FREE_DIST
            free_indices = np.where(free_rays)[0]

        if free_indices.size == 0:
            return float(self.angles[int(np.argmax(proc_lidar))])

        breaks = np.where(np.diff(free_indices) > 1)[0]
        starts = np.r_[0, breaks + 1]
        ends = np.r_[breaks, len(free_indices) - 1]

        best_score = -1e9
        best_angle = goal_angle
        valid_gap = False

        for s, e in zip(starts, ends):
            gap = free_indices[s:e + 1]

            if len(gap) < P.GAP_MIN_LEN:
                continue

            valid_gap = True

            gap_angles = self.angles[gap]
            gap_dists = proc_lidar[gap]

            n = len(gap)

            center_margin = np.minimum(np.arange(n), np.arange(n)[::-1])
            center_score = center_margin / max(1.0, n / 2.0)

            goal_score = 1.0 - np.clip(
                np.abs(gap_angles - goal_angle) / P.MAX_GAP_LOOK_ANGLE,
                0.0,
                1.0
            )

            dist_score = np.clip(gap_dists / P.CLIP, 0.0, 1.0)

            smooth_score = 1.0 - np.clip(
                np.abs(gap_angles - self.prev_steer) / P.MAX_STEER,
                0.0,
                1.0
            )

            scores = (
                P.GAP_GOAL_W * goal_score +
                P.GAP_CENTER_W * center_score +
                P.GAP_DIST_W * dist_score +
                P.GAP_SMOOTH_W * smooth_score
            )

            local_best = int(np.argmax(scores))
            score = float(scores[local_best])

            if score > best_score:
                best_score = score
                best_angle = float(gap_angles[local_best])

        if not valid_gap:
            best_angle = float(self.angles[int(np.argmax(proc_lidar))])

        return best_angle

    def compute_danger_repulsion_original(self, r):
        repulsion_steer_offset = 0.0

        active_mask = np.abs(self.angles) < 1.30
        active_indices = np.where(active_mask)[0]

        for idx in active_indices:
            dist_muro = float(r[idx])

            if dist_muro < P.DIST_CAUTION:
                angle = float(self.angles[idx])

                force = P.REPULSION_K * (
                    (P.DIST_CAUTION - dist_muro) /
                    (dist_muro + 1e-3)
                )

                repulsion_steer_offset -= force * np.sign(angle) * math.cos(angle)

        return float(
            np.clip(
                repulsion_steer_offset,
                -P.MAX_REPULSION_OFFSET,
                P.MAX_REPULSION_OFFSET
            )
        )

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        target_wp = self.wp_xy[self.wp_idx]

        dist_to_wp = math.hypot(
            float(target_wp[0]) - self.x,
            float(target_wp[1]) - self.y
        )

        if dist_to_wp < 3.5:
            self.wp_idx = (self.wp_idx + 1) % self.n_wp

        if 20.0 < self.x < 25.0 and -26.0 < self.y < -20.0:
            self.crossed_checkpoint = True

        if -1.8 < self.x < 1.8 and -2.2 < self.y < 2.2:
            if self.crossed_checkpoint:
                current_time = time.time()

                if self.lap_start_time is not None:
                    lap_duration = current_time - self.lap_start_time
                    self.lap_count += 1

                    if lap_duration < self.best_lap_time:
                        self.best_lap_time = lap_duration

                    print("\n==========================================")
                    print(f"🏁 ¡VUELTA {self.lap_count} COMPLETADA!")
                    print(f"⏱️ Tiempo: {lap_duration:.3f} s | 🏆 Mejor: {self.best_lap_time:.3f} s")
                    print("==========================================\n")

                self.lap_start_time = current_time
                self.crossed_checkpoint = False

            else:
                if self.lap_start_time is None:
                    self.lap_start_time = time.time()
                    print("⏱️ Cronómetro iniciado. ¡Vuelta 1 en marcha!")

    def on_scan(self, scan):
        raw = np.array(scan.ranges, dtype=np.float32)

        if raw.size == 0:
            return

        if self.angles is None:
            self.nr = len(raw)
            self.angles = np.linspace(scan.angle_min, scan.angle_max, self.nr)

        r = np.clip(raw, 0.0, P.CLIP)
        r = np.where(np.isfinite(r), r, P.CLIP)

        goal_angle = self.compute_goal_angle()

        path_mask = np.abs(self.angles - goal_angle) < P.PATH_CHECK_WIDTH
        path_dist = self.percentile_dist(r, path_mask, 10)

        front_mask = np.abs(self.angles) < P.FRONT_CHECK_WIDTH
        front_dist = self.percentile_dist(r, front_mask, 10)

        control_dist = min(path_dist, front_dist)

        if control_dist >= P.DIST_SAFE:
            target_speed = P.SPEED_MAX
            state = "SEGURA"
        elif P.DIST_CAUTION <= control_dist < P.DIST_SAFE:
            target_speed = P.SPEED_CORNER
            state = "CUIDADO"
        else:
            target_speed = P.SPEED_DANGER
            state = "PELIGRO"

        if path_dist < P.AVOID_ENTER_DIST:
            self.avoid_mode = True

        if self.avoid_mode and path_dist > P.AVOID_EXIT_DIST:
            self.avoid_mode = False

        if self.avoid_mode:
            proc_lidar = self.apply_obstacle_bubble(r, goal_angle)
            gap_steer = self.choose_best_gap(proc_lidar, goal_angle)
            mode = "AVOID"
        else:
            gap_steer = goal_angle
            mode = "WAYPOINT"

        wall_correction = 0.0

        if self.avoid_mode or state == "PELIGRO":
            wall_correction = self.follow_wall_assist(r)

        repulsion_steer_offset = 0.0

        if state == "PELIGRO":
            repulsion_steer_offset = self.compute_danger_repulsion_original(r)

        chosen_steer = gap_steer + wall_correction + repulsion_steer_offset

        chosen_steer = float(
            np.clip(
                chosen_steer,
                -P.MAX_STEER,
                P.MAX_STEER
            )
        )

        steer = P.STEER_SMOOTH * self.prev_steer + \
            (1.0 - P.STEER_SMOOTH) * chosen_steer

        steer = float(
            np.clip(
                steer,
                -P.MAX_STEER,
                P.MAX_STEER
            )
        )

        self.prev_steer = steer

        if target_speed < self.prev_speed:
            speed = (1.0 - P.SPEED_BRAKE_K) * self.prev_speed + \
                P.SPEED_BRAKE_K * target_speed
        else:
            speed = 0.20 * self.prev_speed + 0.80 * target_speed

        self.prev_speed = speed

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.pub.publish(msg) 


def main(args=None):
    rclpy.init(args=args)

    node = UnifiedRacer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 