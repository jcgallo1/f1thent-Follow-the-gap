#!/usr/bin/env python3
"""
F1Tenth Controller - Follow the Gap - Jcgallo
"""
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math
import time
 
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
    # ── VARIABLES DE CONTROL FÍSICO  ──
    CLIP          = 10.0     # Distancia máxima que lee el LiDAR 
    MAX_STEER     = 0.41     # Límite físico de giro del servo de dirección (~24 grados)
    STEER_SMOOTH  = 0.15     # Suavizado de dirección 

    # ── PARÁMETROS DEL ALGORITMO FOLLOW THE GAP ──
    SAFE_GAP_DIST = 5.0       # Umbral de distancia para considerar un rayo como "espacio libre" (en metros)
    BUBBLE_R      = 0.90      # Radio de la burbuja de seguridad alrededor de los obstáculos (en metros)
    MAX_GAP_LOOK_ANGLE = 1.05 # Cono de visión frontal del LiDAR (en radianes, ~60 grados a cada lado)

    # ── CONFIGURACIÓN DE LOS 3 RANGOS DE ESTADO  ──
    DIST_SAFE     = 8.0      # Adelante despejado -> ZONA SEGURA 
    DIST_CAUTION  = 0.80     # Entre 0.80 y 8.0 m -> ZONA DE CUIDADO 
                             # Menos de 0.80 m -> ZONA DE PELIGRO   
 
    # ── ESCUDO ANTICHOQUE ──
    REPULSION_K   = 1.25     # Fuerza del empujón 
    # ── VELOCIDADES ASIGNADAS A CADA ZONA ──   
    SPEED_MAX     = 8.0      # Velocidad objetivo en la Zona Segura (Rectas)
    SPEED_CORNER  = 4.5      # Velocidad objetivo en la Zona de Cuidado (Curvas normales)
    SPEED_DANGER  = 2.5      # Velocidad objetivo en la Zona de Peligro (Emergencias)
    SPEED_BRAKE_K = 0.85     # Tasa de frenado  
 
class UnifiedRacer(Node):
    def __init__(self):
        super().__init__('unified_hybrid_racer')

        self.pub  = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan',             self.on_scan, 10)
        self.create_subscription(Odometry,  '/ego_racecar/odom', self.on_odom, 10)

        self.wp_xy  = np.array(WAYPOINTS, dtype=np.float32)
        self.n_wp   = len(WAYPOINTS)

        self.x = self.y = self.yaw = 0.0
        self.wp_idx   = 0
        self.prev_steer = 0.0
        self.prev_speed = P.SPEED_CORNER
        self.angles = None
        self.nr     = None

        # Telemetría por Coordenadas
        self.lap_count = 0
        self.lap_start_time = None
        self.best_lap_time = float('inf')
        self.crossed_checkpoint = False  
        
        self.get_logger().info("CONTROLADOR FOLLOW THE GAP 🏎️") 

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        # Seguimiento referencial de waypoints  
        target_wp = self.wp_xy[self.wp_idx]
        dist_to_wp = math.hypot(target_wp[0] - self.x, target_wp[1] - self.y)
        if dist_to_wp < 3.5: 
            self.wp_idx = (self.wp_idx + 1) % self.n_wp
        # Lógica del Cronómetro en Meta Real
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
                    print(f"\n==========================================")
                    print(f"🏁 ¡VUELTA {self.lap_count} COMPLETADA!")
                    print(f"⏱️ Tiempo: {lap_duration:.3f} s | 🏆 Mejor: {self.best_lap_time:.3f} s")
                    print(f"==========================================\n")
                self.lap_start_time = current_time
                self.crossed_checkpoint = False
            else:
                if self.lap_start_time is None:
                    self.lap_start_time = time.time()
                    print("⏱️ Cronómetro Iniciado. ¡Vuelta 1 en marcha!")

    def on_scan(self, scan):
        raw = np.array(scan.ranges, dtype=np.float32)
        if self.angles is None:
            self.nr     = len(raw)
            self.angles = np.linspace(scan.angle_min, scan.angle_max, self.nr)
        r = np.clip(raw, 0.0, P.CLIP)
        r = np.where(np.isfinite(r), r, P.CLIP)
        # 1. EVALUACIÓN CONTINUA DEL ENTORNO
        # Cono frontal estrecho para medir espacio libre directo hacia adelante
        front_dist = float(np.mean(r[np.abs(self.angles) < 0.15]))
        # DETERMINACIÓN DEL ESTADO Y VELOCIDAD OBJETIVO
        if front_dist >= P.DIST_SAFE:
            target_speed = P.SPEED_MAX
            state = "SEGURA"
        elif P.DIST_CAUTION <= front_dist < P.DIST_SAFE:
            target_speed = P.SPEED_CORNER
            state = "CUIDADO"
        else:
            target_speed = P.SPEED_DANGER
            state = "PELIGRO"

        # 2. EJECUCIÓN DE FOLLOW THE GAP  
        wp = self.wp_xy[self.wp_idx]
        goal_angle = math.atan2(wp[1] - self.y, wp[0] - self.x) - self.yaw
        goal_angle = (goal_angle + math.pi) % (2 * math.pi) - math.pi
        goal_angle = np.clip(goal_angle, -0.7, 0.7)

        driving_mask = np.abs(self.angles) < P.MAX_GAP_LOOK_ANGLE
        proc_lidar = np.where(driving_mask, r, 0.0)

        # Encontrar el punto más cercano para aplicar la burbuja
        closest_idx = int(np.argmin(np.where(driving_mask, proc_lidar, P.CLIP)))
        closest_dist = proc_lidar[closest_idx]

        if closest_dist < 3.0:
            angle_inc = (self.angles[-1] - self.angles[0]) / (self.nr - 1)
            b_half = math.atan2(P.BUBBLE_R, max(closest_dist, 0.1))
            b_idx = int(b_half / angle_inc) + 1
            proc_lidar[max(0, closest_idx - b_idx):min(self.nr, closest_idx + b_idx + 1)] = 0.0

        # Buscar el espacio libre más profundo alineado con el rumbo del mapa
        free_rays = proc_lidar >= P.SAFE_GAP_DIST
        if not np.any(free_rays):
            gap_steer = float(self.angles[np.argmax(proc_lidar)])
        else:
            free_indices = np.where(free_rays)[0]
            gap_steer = float(self.angles[free_indices[np.argmin(np.abs(self.angles[free_indices] - goal_angle))]])

        # 3. COMPORTAMIENTO ADAPTATIVO SEGÚN EL ESTADO ACTIVO
        repulsion_steer_offset = 0.0

        if state == "PELIGRO":
            # ÚNICAMENTE en estado de peligro se activa el escudo para forzar el esquive 
            active_mask = np.abs(self.angles) < 1.30
            active_indices = np.where(active_mask)[0]

            for idx in active_indices:
                dist_muro = r[idx]
                if dist_muro < P.DIST_CAUTION:
                    angle = self.angles[idx]
                    force = P.REPULSION_K * ((P.DIST_CAUTION - dist_muro) / (dist_muro + 1e-3))
                    repulsion_steer_offset -= force * np.sign(angle) * math.cos(angle)

        # Combinar el rumbo del Follow the Gap con el escudo 
        chosen_steer = gap_steer + repulsion_steer_offset

        # 4. FILTRADO FINAL Y SUAVIZADO
        chosen_steer = float(np.clip(chosen_steer, -P.MAX_STEER, P.MAX_STEER))
        steer = P.STEER_SMOOTH * self.prev_steer + (1.0 - P.STEER_SMOOTH) * chosen_steer
        self.prev_steer = steer

        # Transición asimétrica de velocidad (frena rápido, acelera progresivo)
        if target_speed < self.prev_speed:
            speed = (1.0 - P.SPEED_BRAKE_K) * self.prev_speed + P.SPEED_BRAKE_K * target_speed
        else:
            speed = 0.20 * self.prev_speed + 0.80 * target_speed
        self.prev_speed = speed

        # 5. ENVIAR COMANDOS
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steer
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