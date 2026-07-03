#!/usr/bin/env python3
"""
Follow The Gap (FTG) Controller - F1Tenth Competition
Mapa: SaoPaulo | ROS2 Humble | Python
v2 — corrige bucle de emergencia y comportamiento en curvas
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import time
import math


# ─────────────────────────────────────────────
#   PARÁMETROS
# ─────────────────────────────────────────────
class Params:
    # ── LiDAR ──
    ANGLE_MIN        = -2.35
    ANGLE_MAX        =  2.35
    RANGE_MAX        = 30.0
    CLIP_RANGE       = 8.0           # ↓ reducido: ignora lecturas lejanas irrelevantes
    SMOOTH_WINDOW    = 5             # ↑ más suavizado para eliminar spikes

    # ── Bubble ──
    CAR_WIDTH        = 0.50          # ↑ margen extra
    BUBBLE_BASE      = 0.45          # ↑ burbuja más grande = más conservador
    BUBBLE_SCALE     = 0.06

    # ── Gap ──
    MIN_GAP_WIDTH    = 10            # ↑ gaps más anchos para asegurar paso
    PREFERRED_W      = 0.20

    # ── Steering ──
    MAX_STEER        = 0.38
    STEER_SMOOTH     = 0.35          # ↑ más suavizado, menos oscilación
    STEER_GAIN       = 0.95

    # ── Velocidad ──
    SPEED_MAX        = 3.5            # ↓ más conservador en rectas
    SPEED_MID        = 2.8
    SPEED_MIN        = 1.5
    SPEED_SMOOTH     = 0.35

    STEER_THR_MID    = 0.10
    STEER_THR_LOW    = 0.35           # ↓ frena antes en curvas

    FRONT_SLOW_DIST  = 2.5           # ↑ empieza a frenar más lejos
    WALL_DIST_MIN    = 0.50

    # ── Emergencia — CORREGIDO ──
    RECOVERY_DIST    = 0.30          # ↓↓ solo emergencia REAL (era 0.55, causaba el bucle)
    RECOVERY_SPEED   = 1.0
    RECOVERY_STEER   = 0.32
    RECOVERY_TIMEOUT = 0.4           # s máximo en modo emergencia

    # ── Vueltas ──
    LAP_RADIUS       = 1.2
    LAP_MIN_DIST     = 15.0
    TOTAL_LAPS       = 10
 

class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub  = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub  = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.prev_steer  = 0.0
        self.prev_speed  = Params.SPEED_MIN
        self.n_ranges    = None
        self.angle_arr   = None

        # Odometría
        self.x = self.y = 0.0
        self.start_x = self.start_y = None
        self.total_dist = 0.0
        self.prev_x = self.prev_y = None
        self.lap_count = 0
        self.in_start_zone = False
        self.last_lap_dist = 0.0

        # Tiempos
        self.lap_start  = time.time()
        self.race_start = time.time()
        self.lap_times  = []

        # Estado de emergencia — NUEVO
        self.recovery_active    = False
        self.recovery_start     = 0.0
        self.recovery_direction = 0.0   # ±1, se calcula UNA vez al entrar

        self.finished = False
        self.get_logger().info("🏁  FTG v2 listo — esperando datos...")

    # ══════════════════════════════════════════
    #   ODOMETRÍA
    # ══════════════════════════════════════════
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.prev_x is not None:
            self.total_dist += math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        self.prev_x, self.prev_y = self.x, self.y

        if self.start_x is None:
            self.start_x, self.start_y = self.x, self.y
            self.get_logger().info(f"📍 Checkpoint ({self.x:.2f}, {self.y:.2f})")
            return

        dist_to_start  = math.hypot(self.x - self.start_x, self.y - self.start_y)
        dist_since_lap = self.total_dist - self.last_lap_dist

        if dist_to_start < Params.LAP_RADIUS and dist_since_lap > Params.LAP_MIN_DIST:
            if not self.in_start_zone:
                self.in_start_zone = True
                self._register_lap()
        elif dist_to_start > Params.LAP_RADIUS * 1.8:
            self.in_start_zone = False

    def _register_lap(self):
        self.lap_count += 1
        now      = time.time()
        lap_time = now - self.lap_start
        total    = now - self.race_start
        self.lap_times.append(lap_time)
        self.last_lap_dist = self.total_dist
        self.lap_start     = now
        self.get_logger().info(
            f"🏆  VUELTA {self.lap_count:2d} | TIEMPO {lap_time:6.2f}s | TOTAL {total:7.2f}s")
        if self.lap_count >= Params.TOTAL_LAPS:
            self._finish_race(total)

    def _finish_race(self, total: float):
        self.finished = True
        self.get_logger().info("=" * 55)
        self.get_logger().info("🏁  ¡CARRERA COMPLETADA!")
        self.get_logger().info(f"    Tiempo total : {total:.2f}s")
        self.get_logger().info(f"    Mejor vuelta : {min(self.lap_times):.2f}s")
        self.get_logger().info(f"    Media vuelta : {sum(self.lap_times)/len(self.lap_times):.2f}s")
        for i, t in enumerate(self.lap_times, 1):
            self.get_logger().info(f"    Vuelta {i:2d}: {t:.2f}s")
        self.get_logger().info("=" * 55)
        self._publish_drive(0.0, 0.0)

    # ══════════════════════════════════════════
    #   PIPELINE FTG
    # ══════════════════════════════════════════
    def scan_callback(self, scan: LaserScan):
        if self.finished:
            self._publish_drive(0.0, 0.0)
            return

        ranges = np.array(scan.ranges, dtype=np.float32)

        if self.angle_arr is None:
            self.n_ranges  = len(ranges)
            self.angle_arr = np.linspace(scan.angle_min, scan.angle_max, self.n_ranges)

        ranges = self._preprocess(ranges)

        # ── Emergencia con timeout ──
        if self._handle_emergency(ranges):
            return

        bubble_ranges          = self._apply_bubble(ranges)
        gap_start, gap_end     = self._find_best_gap(bubble_ranges)
        target_idx             = self._best_point_in_gap(bubble_ranges, gap_start, gap_end)
        steer                  = self._calculate_steering(target_idx, ranges)
        speed                  = self._calculate_speed(steer, ranges)
        self._publish_drive(steer, speed)

    # ──────────────────────────────────────────
    def _preprocess(self, ranges: np.ndarray) -> np.ndarray:
        ranges = np.clip(ranges, 0.0, Params.CLIP_RANGE)
        ranges = np.where(np.isfinite(ranges), ranges, Params.CLIP_RANGE)

        # Mediana primero para eliminar spikes puntuales
        from numpy.lib.stride_tricks import sliding_window_view
        k = Params.SMOOTH_WINDOW
        pad = k // 2
        padded = np.pad(ranges, pad, mode='edge')
        # Media móvil (más rápida que mediana en tiempo real)
        kernel = np.ones(k) / k
        ranges = np.convolve(padded, kernel, mode='valid')
        return ranges[:self.n_ranges]

    # ──────────────────────────────────────────
    def _handle_emergency(self, ranges: np.ndarray) -> bool:
        """
        CORREGIDO:
        - Umbral reducido a 0.30 m (solo peligro real)
        - Dirección calculada UNA sola vez al entrar en emergencia
        - Timeout de salida obligatorio
        """
        front_mask = np.abs(self.angle_arr) < 0.45   # ±26°
        front_min  = float(np.min(ranges[front_mask]))
        now        = time.time()

        # ── Salir de emergencia por timeout ──
        if self.recovery_active:
            elapsed = now - self.recovery_start
            if elapsed > Params.RECOVERY_TIMEOUT or front_min > Params.RECOVERY_DIST * 1.5:
                self.recovery_active = False
                self.get_logger().info("✅  Emergencia resuelta")
                return False
            # Continuar con la MISMA dirección calculada al inicio
            self._publish_drive(
                self.recovery_direction * Params.RECOVERY_STEER,
                Params.RECOVERY_SPEED
            )
            return True

        # ── Detectar nueva emergencia ──
        if front_min < Params.RECOVERY_DIST:
            # Calcular dirección óptima mirando todo el scan lateral
            left_mask  = self.angle_arr > 0.3
            right_mask = self.angle_arr < -0.3
            left_space  = float(np.mean(ranges[left_mask]))  if left_mask.any()  else 0.0
            right_space = float(np.mean(ranges[right_mask])) if right_mask.any() else 0.0

            # +1 = izquierda, -1 = derecha
            self.recovery_direction = 1.0 if left_space > right_space else -1.0
            self.recovery_active    = True
            self.recovery_start     = now

            self.get_logger().warn(
                f"⚠️  EMERGENCIA: {front_min:.2f}m | "
                f"izq={left_space:.2f} der={right_space:.2f} → "
                f"{'IZQ' if self.recovery_direction > 0 else 'DER'}"
            )
            self._publish_drive(
                self.recovery_direction * Params.RECOVERY_STEER,
                Params.RECOVERY_SPEED
            )
            return True

        return False

    # ──────────────────────────────────────────
    def _apply_bubble(self, ranges: np.ndarray) -> np.ndarray:
        closest_idx = int(np.argmin(ranges))
        closest_r   = float(ranges[closest_idx])

        bubble_r   = Params.BUBBLE_BASE + Params.BUBBLE_SCALE * self.prev_speed
        angle_inc  = (Params.ANGLE_MAX - Params.ANGLE_MIN) / self.n_ranges
        half_angle = math.atan2(bubble_r, max(closest_r, 0.05))
        half_idx   = int(half_angle / angle_inc) + 2   # +2 de margen extra

        bubble = ranges.copy()
        lo = max(0, closest_idx - half_idx)
        hi = min(self.n_ranges - 1, closest_idx + half_idx)
        bubble[lo:hi + 1] = 0.0
        return bubble

    # ──────────────────────────────────────────
    def _find_best_gap(self, ranges: np.ndarray):
        """
        MEJORADO: threshold dinámico según distancia máxima disponible.
        En curvas cerradas acepta gaps con menor clearance.
        """
        max_r     = float(np.max(ranges))
        threshold = max(0.3, min(1.0, max_r * 0.15))   # dinámico 0.3–1.0 m

        free = (ranges > threshold).astype(int)
        gaps = []
        i = 0
        while i < len(free):
            if free[i] == 1:
                j = i
                while j < len(free) and free[j] == 1:
                    j += 1
                width = j - i
                if width >= Params.MIN_GAP_WIDTH:
                    gaps.append((i, j - 1))
                i = j
            else:
                i += 1

        if not gaps:
            # Fallback: abrir el umbral a la mitad y reintentar
            threshold *= 0.5
            free = (ranges > threshold).astype(int)
            gaps = []
            i = 0
            while i < len(free):
                if free[i] == 1:
                    j = i
                    while j < len(free) and free[j] == 1:
                        j += 1
                    if (j - i) >= max(4, Params.MIN_GAP_WIDTH // 2):
                        gaps.append((i, j - 1))
                    i = j
                else:
                    i += 1

        if not gaps:
            return 0, self.n_ranges - 1

        straight_idx = int(np.argmin(np.abs(self.angle_arr)))

        def gap_score(g):
            w    = g[1] - g[0]
            d    = float(np.max(ranges[g[0]:g[1] + 1]))
            mid  = (g[0] + g[1]) // 2
            # Penalizar gaps lejos del frente
            angle_cost = abs(self.angle_arr[mid])
            return w * d * math.exp(-0.8 * angle_cost)

        return max(gaps, key=gap_score)

    # ──────────────────────────────────────────
    def _best_point_in_gap(self, ranges: np.ndarray, gs: int, ge: int) -> int:
        seg = ranges[gs:ge + 1]
        n   = len(seg)
        if n == 0:
            return (gs + ge) // 2

        center  = n // 2
        sigma   = max(n / 3.5, 1.0)
        w_gauss = np.exp(-0.5 * ((np.arange(n) - center) / sigma) ** 2)

        straight_idx = int(np.argmin(np.abs(self.angle_arr)))
        dist_str     = np.abs(np.arange(gs, ge + 1) - straight_idx)
        sigma_str    = max(n / 2.5, 1.0)
        w_straight   = np.exp(-0.5 * (dist_str / sigma_str) ** 2)

        score = seg * (w_gauss + Params.PREFERRED_W * w_straight)
        return gs + int(np.argmax(score))

    # ──────────────────────────────────────────
    def _calculate_steering(self, target_idx: int, ranges: np.ndarray) -> float:
        target_angle = float(self.angle_arr[target_idx])
        wall_corr    = self._wall_correction(ranges)
        raw_steer    = np.clip(
            Params.STEER_GAIN * target_angle + wall_corr,
            -Params.MAX_STEER, Params.MAX_STEER
        )
        smooth = Params.STEER_SMOOTH * self.prev_steer + (1 - Params.STEER_SMOOTH) * raw_steer
        self.prev_steer = float(smooth)
        return float(smooth)

    def _wall_correction(self, ranges: np.ndarray) -> float:
        """Corrección suave para mantener distancia de paredes laterales."""
        right_mask = (self.angle_arr > -1.1) & (self.angle_arr < -0.25)
        left_mask  = (self.angle_arr >  0.25) & (self.angle_arr <  1.1)
        right_d = float(np.percentile(ranges[right_mask], 20)) if right_mask.any() else 5.0
        left_d  = float(np.percentile(ranges[left_mask],  20)) if  left_mask.any() else 5.0

        corr = 0.0
        thresh = Params.WALL_DIST_MIN
        if right_d < thresh:
            corr += 0.08 * (thresh - right_d) / thresh
        if left_d < thresh:
            corr -= 0.08 * (thresh - left_d) / thresh
        return float(np.clip(corr, -0.15, 0.15))

    # ──────────────────────────────────────────
    def _calculate_speed(self, steer: float, ranges: np.ndarray) -> float:
        """
        MEJORADO:
        - Frenar anticipando curvas mirando más lejos al frente
        - Velocidad mínima garantizada para no quedarse parado
        """
        abs_s = abs(steer)

        if abs_s < Params.STEER_THR_MID:
            target = Params.SPEED_MAX
        elif abs_s < Params.STEER_THR_LOW:
            t = (abs_s - Params.STEER_THR_MID) / (Params.STEER_THR_LOW - Params.STEER_THR_MID)
            target = Params.SPEED_MAX - t * (Params.SPEED_MAX - Params.SPEED_MID)
        else:
            # Curva cerrada: velocidad proporcional al steer
            excess = abs_s - Params.STEER_THR_LOW
            target = max(Params.SPEED_MIN,
                         Params.SPEED_MID - excess * 4.0)

        # Frenar si hay obstáculo cerca al frente (cono más estrecho = ±15°)
        front_mask = np.abs(self.angle_arr) < 0.26
        front_vals = ranges[front_mask]
        front_min  = float(np.percentile(front_vals, 10))   # percentil 10 = robusto a ruido

        if front_min < Params.FRONT_SLOW_DIST:
            ratio  = (front_min / Params.FRONT_SLOW_DIST) ** 1.5
            target = max(Params.SPEED_MIN, target * ratio)

        smooth = Params.SPEED_SMOOTH * self.prev_speed + (1 - Params.SPEED_SMOOTH) * target
        self.prev_speed = float(smooth)
        return float(smooth)

    # ──────────────────────────────────────────
    def _publish_drive(self, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp        = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed          = float(speed)
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑  Nodo detenido.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 