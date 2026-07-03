#!/usr/bin/env python3
"""
Advanced Predictive Goal-Directed FTG - F1Tenth | ROS2 Humble
Optimizado para alta velocidad. Evalúa un abanico dinámico hacia el Waypoint 
y corrige la trayectoria vectorialmente en tiempo real antes de perder el horizonte.
"""
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import time
import math

# ── WAYPOINTS (x, y, speed_m/s) ──────────────────────────────────────────────
WAYPOINTS = [
    (-0.10,  -0.50,  6.5),  # WP  0 META - Velocidades base incrementadas para test
    ( 5.71, -21.61,  7.0),  # WP  1
    ( 8.06, -25.40,  6.0),  # WP  2
    (10.86, -26.03,  6.0),  # WP  3
    (15.17, -22.14,  7.5),  # WP  4
    (16.66, -21.63,  8.0),  # WP  5
    (22.65, -24.38,  8.0),  # WP  6
    (26.89, -24.72,  8.0),  # WP  7
    (30.30, -23.37,  7.0),  # WP  8
    (33.80, -20.50,  7.0),  # WP  9
    (35.69, -17.26,  7.0),  # WP 10
    (42.57,  10.25,  8.5),  # WP 11
    (48.37,  31.40,  9.5),  # WP 12
    (47.68,  35.22,  6.5),  # WP 13
    (34.97,  36.87,  7.0),  # WP 14
    (30.73,  34.48,  7.5),  # WP 15
    (12.53,  10.00,  8.5),  # WP 16
    ( 3.04,  12.60,  6.5),  # WP 17
    ( 1.53,  14.92,  5.5),  # WP 18
    ( 0.06,  24.97,  6.5),  # WP 19
    ( 1.63,  26.42,  6.5),  # WP 20
    ( 7.09,  24.15,  7.5),  # WP 21
    (10.06,  24.38,  7.5),  # WP 22
    (11.53,  27.42,  6.5),  # WP 23
    (10.41,  29.98,  6.5),  # WP 24
    ( 4.57,  36.90,  6.5),  # WP 25
    ( 2.70,  42.54,  6.5),  # WP 26
    ( 3.70,  44.85,  6.5),  # WP 27
    ( 6.81,  44.36,  7.5),  # WP 28
    (13.06,  37.69,  8.5),  # WP 29
    (18.79,  36.62,  8.5),  # WP 30
    (25.42,  42.44,  7.5),  # WP 31
    (30.80,  51.81,  6.5),  # WP 32
    (31.13,  54.08,  6.5),  # WP 33
    (23.88,  57.66,  6.5),  # WP 34
    (15.91,  57.01,  7.5),  # WP 35
    ( 6.41,  53.18,  7.5),  # WP 36
    ( 2.00,  49.56,  7.5),  # WP 37
    (-0.86,  42.57,  7.5),  # WP 38
]

# ── PARÁMETROS DINÁMICOS DE ALTA VELOCIDAD ──────────────────────────────────
class P:
    # LiDAR general
    CLIP          = 12.0
    SMOOTH        = 3        # Menor suavizado para reducir latencia de datos a alta velocidad
    FREE_THRESHOLD= 2.2      # Umbral exigente de espacio libre para altas velocidades

    # Burbuja física protectora
    BUBBLE_R      = 0.70     # Cubre el ancho del chasis con margen de seguridad dinámico

    # Umbral de aceptación de Waypoint
    WP_REACH      = 1.8      # Mayor distancia de corte para liberar rápido el siguiente tramo

    # Dirección (Steering)
    MAX_STEER     = 0.38
    STEER_SMOOTH  = 0.08     # Muy bajo = Respuesta ultra inmediata de los servo motores

    # Velocidad General
    SPEED_MIN     = 2.2
    SPEED_SMOOTH  = 0.15     # Permite aceleraciones y frenadas más contundentes
    STEER_SPEED_K = 0.80     # Freno severo en curvas de alto G (evita derrapes por subviraje)

    # Seguridad reactiva por LiDAR frontal
    BRAKE_START   = 7.0      # Comienza a desacelerar si ve un muro al frente a esta distancia
    BRAKE_FULL    = 1.5      # Freno máximo tolerable

    # Ajustes Preventivos por Horizonte Cerrado (Mecanismo de Aprendizaje)
    FAN_ANGLE_RAD      = 0.25 # ~14 grados a cada lado del objetivo para el abanico de inspección
    PREVENTIVE_SHIFT   = 0.45 # Distancia (metros) de corrección lateral del WP por ciclo
    PREVENTIVE_DECAY   = 0.92 # Reducción del 8% de velocidad en puntos críticos

    # Salvavidas extremo (Recovery)
    REC_TRIGGER   = 0.50
    REC_REVERSE_V = -1.2
    REC_STEER     = 0.38
    REC_TIMEOUT   = 1.8
    REC_MIN_TIME  = 0.40
    REC_EXIT_DIST = 1.8

    TOTAL_LAPS    = 10


# ── NODO PREDICTIVO DE ALTO RENDIMIENTO ──────────────────────────────────────
class RaceNode(Node):
    def __init__(self):
        super().__init__('advanced_predictive_ftg')

        self.pub  = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan',             self.on_scan, 10)
        self.create_subscription(Odometry,  '/ego_racecar/odom', self.on_odom, 10)

        self.wp_xy    = np.array([[w[0], w[1]] for w in WAYPOINTS], dtype=np.float32)
        self.wp_spd   = np.array([w[2] for w in WAYPOINTS], dtype=np.float32)
        self.n_wp     = len(WAYPOINTS)

        self.x = self.y = self.yaw = 0.0
        self.wp_idx   = 0
        self.laps     = 0
        self.finished = False

        self.prev_steer = 0.0
        self.prev_speed = P.SPEED_MIN

        self.rec_state = 'normal'
        self.rec_start = 0.0
        self.rec_dir   = 1.0
        self._rec_cooldown = 0.0

        self.angles = None
        self.nr     = None

        self._last_wp_log  = -1
        self._lap_cooldown = 0.0
        self._prev_d_start = None

        self.get_logger().info("Controlador de Abanico Predictivo para Alta Velocidad Activo.")

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        target_wp = self.wp_xy[self.wp_idx]
        dist_to_wp = math.hypot(target_wp[0] - self.x, target_wp[1] - self.y)

        if dist_to_wp < P.WP_REACH:
            prev = self.wp_idx
            self.wp_idx = (self.wp_idx + 1) % self.n_wp
            if self.wp_idx != self._last_wp_log and self.laps == 0:
                self._last_wp_log = self.wp_idx
                self.get_logger().info(f"[WP Objetivo]: {self.wp_idx} | Velocidad de Curva: {self.wp_spd[self.wp_idx]:.1f} m/s")

        # Monitoreo de meta
        d_meta = math.hypot(self.x - self.wp_xy[0,0], self.y - self.wp_xy[0,1])
        now = time.time()
        if self._prev_d_start is not None:
            if (self._prev_d_start > P.WP_REACH and d_meta <= P.WP_REACH and now - self._lap_cooldown > 8.0):
                self.laps += 1
                self._lap_cooldown = now
                self.get_logger().info(f"🏁 >>> [LAP COMPLETADA: {self.laps}/{P.TOTAL_LAPS}] <<< 🏁")
                if self.laps >= P.TOTAL_LAPS:
                    self.finished = True
        self._prev_d_start = d_meta

    def on_scan(self, scan):
        if self.finished:
            self._drive(0.0, 0.0); return

        raw = np.array(scan.ranges, dtype=np.float32)
        if self.angles is None:
            self.nr     = len(raw)
            self.angles = np.linspace(scan.angle_min, scan.angle_max, self.nr)

        r = self._preprocess(raw)

        # Diagnóstico rápido de seguridad frontal espacial
        fmask_narrow = np.abs(self.angles) < 0.22
        fmask_wide   = np.abs(self.angles) < 0.55
        front        = float(np.min(r[fmask_narrow]))
        front_rec    = float(np.percentile(r[fmask_wide], 5))

        if self._recovery(r, front_rec):
            return

        # 1. Calcular orientación teórica al waypoint objetivo
        wp = self.wp_xy[self.wp_idx]
        dx, dy = wp[0] - self.x, wp[1] - self.y
        goal_angle = math.atan2(dy, dx) - self.yaw
        goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi

        # 2. LOGICA AVANZADA: Inspección en Abanico Predictivo (Previene choques a alta velocidad)
        # Definimos la distancia crítica de prospección según la velocidad real del coche (Frenado Seguro)
        dynamic_lookahead_dist = max(3.5, self.prev_speed * 0.65)

        # Máscara de índices del LiDAR que caen dentro del abanico apuntando hacia el Waypoint
        fan_mask = np.abs(self.angles - goal_angle) <= P.FAN_ANGLE_RAD
        
        if np.any(fan_mask):
            # Medimos la distancia mínima real en ese cono de trayectoria proyectado
            fan_min_dist = float(np.min(r[fan_mask]))
            
            # Si el abanico detecta una pared u obstáculo interfiriendo en nuestro horizonte predictivo:
            if fan_min_dist < dynamic_lookahead_dist:
                # El camino está obstruído. Buscamos de forma inmediata las zonas transitables abiertas
                # Evaluamos los dos extremos laterales para ver cuál ofrece un horizonte de escape real profundo
                left_search  = float(np.percentile(r[self.angles > 0.1], 40))   # Percentil robusto para evitar ruido
                right_search = float(np.percentile(r[self.angles < -0.1], 40))
                
                escape_direction = 1.0 if left_search > right_search else -1.0
                
                # Desplazamiento perpendicular adaptativo: empuja el Waypoint hacia el centro de la pista
                shift_yaw = self.yaw + (escape_direction * (math.pi / 2.0))
                self.wp_xy[self.wp_idx][0] += P.PREVENTIVE_SHIFT * math.cos(shift_yaw)
                self.wp_xy[self.wp_idx][1] += P.PREVENTIVE_SHIFT * math.sin(shift_yaw)
                
                # Castigo de velocidad al WP modificado para evitar subvirajes inerciales en la siguiente vuelta
                self.wp_spd[self.wp_idx] = max(P.SPEED_MIN, self.wp_spd[self.wp_idx] * P.PREVENTIVE_DECAY)
                
                # Recalcular inmediatamente el objetivo corregido en este mismo frame
                wp = self.wp_xy[self.wp_idx]
                dx, dy = wp[0] - self.x, wp[1] - self.y
                goal_angle = math.atan2(dy, dx) - self.yaw
                goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi

        # 3. Aplicar Burbuja de Seguridad Reactiva Física de proximidad
        processed_lidar = r.copy()
        closest_idx = int(np.argmin(processed_lidar))
        closest_dist = max(float(processed_lidar[closest_idx]), 0.05)
        
        angle_increment = (self.angles[-1] - self.angles[0]) / (self.nr - 1)
        bubble_half_angle = math.atan2(P.BUBBLE_R, closest_dist)
        bubble_idx_range = int(bubble_half_angle / angle_increment) + 2
        
        start_idx = max(0, closest_idx - bubble_idx_range)
        end_idx = min(self.nr, closest_idx + bubble_idx_range + 1)
        processed_lidar[start_idx:end_idx] = 0.0

        # 4. Encontrar el mejor rayo libre guiado por el Waypoint balanceado
        chosen_steer = self._find_best_ray(processed_lidar, goal_angle)

        # 5. Suavizado asíncrono rápido del actuador
        chosen_steer = float(np.clip(chosen_steer, -P.MAX_STEER, P.MAX_STEER))
        steer = P.STEER_SMOOTH * self.prev_steer + (1 - P.STEER_SMOOTH) * chosen_steer
        self.prev_steer = steer

        # 6. Cálculo e inyección de velocidad final
        speed = self._calculate_speed(steer, front)
        self._drive(steer, speed)

    def _find_best_ray(self, processed_lidar, goal_angle):
        free_rays_mask = processed_lidar > P.FREE_THRESHOLD

        if not np.any(free_rays_mask):
            return float(self.angles[np.argmax(processed_lidar)])

        free_indices = np.where(free_rays_mask)[0]
        angular_offsets = np.abs(self.angles[free_indices] - goal_angle)
        best_free_idx = free_indices[np.argmin(angular_offsets)]
        return float(self.angles[best_free_idx])

    def _recovery(self, r, front_rec):
        now = time.time()
        if self.rec_state == 'normal':
            if front_rec < P.REC_TRIGGER and now - self._rec_cooldown > 2.0:
                lm = self.angles >  0.3
                rm = self.angles < -0.3
                ld = float(np.mean(r[lm])) if lm.any() else 0.0
                rd = float(np.mean(r[rm])) if rm.any() else 0.0
                self.rec_dir   = 1.0 if ld > rd else -1.0
                self.rec_state = 'reverse'
                self.rec_start = now
                self.get_logger().error("🚨 [SISTEMA CRÍTICO RECOVERY] Activado.")

        if self.rec_state == 'reverse':
            elapsed = now - self.rec_start
            if elapsed >= P.REC_MIN_TIME and front_rec > P.REC_EXIT_DIST:
                self.rec_state     = 'normal'
                self._rec_cooldown = now
                return False
            if elapsed > P.REC_TIMEOUT:
                self.rec_state     = 'normal'
                self._rec_cooldown = now
                return False
            self._drive(self.rec_dir * P.REC_STEER, P.REC_REVERSE_V)
            return True
        return False

    def _preprocess(self, r):
        r = np.clip(r, 0.0, P.CLIP)
        r = np.where(np.isfinite(r), r, P.CLIP)
        k = P.SMOOTH
        return np.convolve(np.pad(r, k//2, 'edge'), np.ones(k)/k, 'valid')[:self.nr]

    def _calculate_speed(self, steer, front):
        target = float(self.wp_spd[self.wp_idx])

        # Atenuación agresiva por curvatura angular (Crucial para mantener adherencia centrípeta)
        steer_ratio = abs(steer) / P.MAX_STEER
        target *= (1.0 - P.STEER_SPEED_K * (steer_ratio ** 2)) # Exponencial para conservar velocidad en rectas

        # Desaceleración por distancia de aproximación frontal
        if front < P.BRAKE_START:
            if front <= P.BRAKE_FULL:
                brake = 0.0
            else:
                brake = (front - P.BRAKE_FULL) / (P.BRAKE_START - P.BRAKE_FULL)
            target *= brake

        target = max(P.SPEED_MIN, target)

        s = P.SPEED_SMOOTH * self.prev_speed + (1 - P.SPEED_SMOOTH) * target
        self.prev_speed = float(s)
        return float(s)

    def _drive(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed          = float(speed)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo detenido.")
    finally:
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()