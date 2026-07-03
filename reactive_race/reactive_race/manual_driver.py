#!/usr/bin/env python3
"""
Follow The Gap v5 — F1Tenth / SaoPaulo
Filosofía: SIMPLE > COMPLEJO. Menos lógica = menos bugs.
- Un solo mecanismo de control: FTG puro con burbuja grande
- Sin modo emergencia separado (causaba los bucles)
- Sin corridor centering (causaba tambaleo)
- Velocidad = función directa de la apertura frontal del LiDAR
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import time
import math


# ══════════════════════════════════════════════════════════
#  PARÁMETROS — los únicos que necesitas tocar
# ══════════════════════════════════════════════════════════
BUBBLE_RADIUS   = 0.8    # [m] radio de burbuja de seguridad. SUBE si choca, BAJA si muy lento
SPEED_MAX       = 4.0    # [m/s] velocidad en recta despejada. Empieza aquí, sube de 0.5 en 0.5
SPEED_MIN       = 1.2    # [m/s] velocidad mínima (chicanes)
FRONT_DIST_MAX  = 4.0    # [m] distancia frontal libre → velocidad máxima
FRONT_DIST_MIN  = 0.8    # [m] distancia frontal libre → velocidad mínima
STEER_SMOOTH    = 0.15   # factor EMA steering [0=sin suavizado, 0.5=muy suave]
CLIP_RANGE      = 6.0    # [m] ignorar lecturas de LiDAR más lejanas que esto
LAPS_TARGET     = 10     # vueltas objetivo
LAP_RADIUS      = 1.5    # [m] radio del checkpoint de vuelta
LAP_MIN_DIST    = 12.0   # [m] distancia mínima recorrida para contar vuelta


class FTG(Node):

    def __init__(self):
        super().__init__('follow_the_gap')

        self.pub_drive = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(
            Odometry, '/ego_racecar/odom', self.on_odom, 10)

        # estado del scan
        self.angles   = None   # array de ángulos del LiDAR
        self.n        = None   # número de rayos
        self.prev_s   = 0.0   # steering previo (para EMA)

        # odometría
        self.x = self.y = 0.0
        self.px = self.py = None
        self.dist = 0.0
        self.start_x = self.start_y = None
        self.in_zone  = False
        self.dist_at_last_lap = 0.0

        # vueltas
        self.lap      = 0
        self.lap_times = []
        self.t_lap    = time.time()
        self.t_race   = time.time()
        self.finished = False

        self.get_logger().info("🏁  FTG v5 listo")

    # ──────────────────────────────────────────────────────
    #  ODOMETRÍA / CONTADOR DE VUELTAS
    # ──────────────────────────────────────────────────────
    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.px is not None:
            self.dist += math.hypot(self.x - self.px, self.y - self.py)
        self.px, self.py = self.x, self.y

        # Fijar punto de inicio
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.get_logger().info(
                f"📍 Checkpoint fijado en ({self.x:.2f}, {self.y:.2f})")
            return

        d2start = math.hypot(self.x - self.start_x, self.y - self.start_y)
        d_since  = self.dist - self.dist_at_last_lap

        if d2start < LAP_RADIUS and d_since > LAP_MIN_DIST:
            if not self.in_zone:
                self.in_zone = True
                self._register_lap()
        elif d2start > LAP_RADIUS * 2.5:
            self.in_zone = False

    def _register_lap(self):
        self.lap += 1
        now  = time.time()
        lt   = now - self.t_lap
        tot  = now - self.t_race
        self.lap_times.append(lt)
        self.dist_at_last_lap = self.dist
        self.t_lap = now
        self.get_logger().info(
            f"🏆  VUELTA {self.lap:2d} | TIEMPO {lt:6.2f}s | TOTAL {tot:7.2f}s")
        if self.lap >= LAPS_TARGET:
            self._finish(tot)

    def _finish(self, total):
        self.finished = True
        self.get_logger().info("=" * 52)
        self.get_logger().info("🏁  ¡CARRERA COMPLETADA!")
        self.get_logger().info(f"    Tiempo total : {total:.2f}s")
        self.get_logger().info(f"    Mejor vuelta : {min(self.lap_times):.2f}s")
        self.get_logger().info(
            f"    Media vuelta : {sum(self.lap_times)/len(self.lap_times):.2f}s")
        for i, t in enumerate(self.lap_times, 1):
            self.get_logger().info(f"    V{i:02d}: {t:.2f}s")
        self.get_logger().info("=" * 52)
        self._drive(0.0, 0.0)

    # ──────────────────────────────────────────────────────
    #  PIPELINE FTG
    # ──────────────────────────────────────────────────────
    def on_scan(self, scan: LaserScan):
        if self.finished:
            self._drive(0.0, 0.0)
            return

        r = np.array(scan.ranges, dtype=np.float32)

        # Inicializar array de ángulos (solo una vez)
        if self.angles is None:
            self.n = len(r)
            self.angles = np.linspace(
                scan.angle_min, scan.angle_max, self.n)

        # 1. Limpiar: clip + reemplazar NaN/Inf + suavizado
        r = self._clean(r)

        # 2. Aplicar burbuja alrededor del punto más cercano
        r = self._bubble(r)

        # 3. Encontrar el gap más grande
        start, end = self._find_gap(r)

        # 4. Elegir el mejor punto dentro del gap
        target = self._target_point(r, start, end)

        # 5. Steering = ángulo del punto objetivo (+ EMA)
        steer = self._steering(target)

        # 6. Velocidad = función de la apertura frontal libre
        speed = self._speed(r)

        self._drive(steer, speed)

    # ──────────────────────────────────────────────────────
    def _clean(self, r: np.ndarray) -> np.ndarray:
        """Clip + NaN → clip_range + media móvil de 5 puntos."""
        r = np.clip(r, 0.0, CLIP_RANGE)
        r = np.where(np.isfinite(r), r, CLIP_RANGE)
        kernel = np.ones(5) / 5.0
        r = np.convolve(r, kernel, mode='same')
        return r

    # ──────────────────────────────────────────────────────
    def _bubble(self, r: np.ndarray) -> np.ndarray:
        """
        Pone a 0 los índices alrededor del punto más cercano.
        Radio físico fijo → convierte a número de índices.
        """
        idx = int(np.argmin(r))
        d   = max(float(r[idx]), 0.05)

        # Cuántos índices cubre BUBBLE_RADIUS a esa distancia
        angle_per_idx = abs(self.angles[-1] - self.angles[0]) / (self.n - 1)
        half_angle    = math.atan2(BUBBLE_RADIUS, d)
        half_idx      = int(half_angle / angle_per_idx) + 1

        out = r.copy()
        lo  = max(0, idx - half_idx)
        hi  = min(self.n - 1, idx + half_idx)
        out[lo:hi + 1] = 0.0
        return out

    # ──────────────────────────────────────────────────────
    def _find_gap(self, r: np.ndarray):
        """
        Detecta todos los gaps libres (> 0) y devuelve el mejor.
        Score = ancho × distancia_max × penalización_lateral.
        """
        free = (r > 0.05).astype(np.int8)
        gaps = []
        i = 0
        while i < self.n:
            if free[i]:
                j = i
                while j < self.n and free[j]:
                    j += 1
                if (j - i) >= 5:          # mínimo 5 índices de ancho
                    gaps.append((i, j - 1))
                i = j
            else:
                i += 1

        if not gaps:
            # Fallback: todo el scan
            return 0, self.n - 1

        def score(g):
            mid  = (g[0] + g[1]) // 2
            ang  = abs(float(self.angles[mid]))
            w    = g[1] - g[0]
            dmax = float(np.max(r[g[0]:g[1] + 1]))
            # Penaliza fuerte los gaps laterales: preferir ir al frente
            return w * dmax * math.exp(-1.8 * ang)

        return max(gaps, key=score)

    # ──────────────────────────────────────────────────────
    def _target_point(self, r: np.ndarray, gs: int, ge: int) -> int:
        """
        Dentro del gap: punto de máxima distancia, ponderado
        por cercanía al centro del gap y al frente (ángulo 0).
        """
        seg = r[gs:ge + 1]
        n   = len(seg)
        if n == 0:
            return (gs + ge) // 2

        # Peso 1: gaussiana centrada en el gap
        cx    = n // 2
        w_gap = np.exp(-0.5 * ((np.arange(n) - cx) / max(n / 3.0, 1)) ** 2)

        # Peso 2: cercanía al frente (índice con ángulo = 0)
        front_idx = int(np.argmin(np.abs(self.angles)))
        dist_front = np.abs(np.arange(gs, ge + 1) - front_idx)
        w_front = np.exp(-0.5 * (dist_front / max(n / 2.5, 1)) ** 2)

        score = seg * (0.6 * w_gap + 0.4 * w_front)
        return gs + int(np.argmax(score))

    # ──────────────────────────────────────────────────────
    def _steering(self, target_idx: int) -> float:
        """Ángulo objetivo → steering con EMA."""
        angle = float(self.angles[target_idx])
        angle = float(np.clip(angle, -0.36, 0.36))

        # EMA: mezcla el nuevo ángulo con el steering anterior
        s = (1 - STEER_SMOOTH) * angle + STEER_SMOOTH * self.prev_s
        self.prev_s = s
        return float(s)

    # ──────────────────────────────────────────────────────
    def _speed(self, r: np.ndarray) -> float:
        """
        Velocidad = función lineal de la distancia libre al frente.
        Cono frontal ±15° para ser robusto a ruido lateral.
        """
        front_mask = np.abs(self.angles) < 0.26     # ±15°
        front_vals = r[front_mask]

        # Percentil 15: ignora rayos ruidosos, reacciona a obstáculos reales
        front_d = float(np.percentile(front_vals, 15))
        front_d = float(np.clip(front_d, FRONT_DIST_MIN, FRONT_DIST_MAX))

        # Mapeo lineal: front_d_min → speed_min, front_d_max → speed_max
        t = (front_d - FRONT_DIST_MIN) / (FRONT_DIST_MAX - FRONT_DIST_MIN)
        speed = SPEED_MIN + t * (SPEED_MAX - SPEED_MIN)
        return float(speed)

    # ──────────────────────────────────────────────────────
    def _drive(self, steer: float, speed: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed          = float(speed)
        self.pub_drive.publish(msg)


# ══════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = FTG()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("🛑  Detenido por el usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 