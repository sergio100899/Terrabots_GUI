#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nodo ROS2 que lee /odom y publica cada N segundos al endpoint /add_pose/
en el formato:
{
  "pose": {
    "position": {"x": .., "y": .., "z": ..},
    "orientation": {"roll": .., "pitch": .., "yaw": ..}
  }
}
"""

import math
import requests

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_rpy(x: float, y: float, z: float, w: float):
    """
    Convierte un cuaternión (x,y,z,w) a ángulos de Euler (roll, pitch, yaw) en radianes.
    Convención RPY intrínseca XYZ (roll sobre X, pitch sobre Y, yaw sobre Z).
    """
    # roll (x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y)
    sinp = 2.0 * (w * y - z * x)
    # proteger contra errores numéricos
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PoseUploader(Node):
    def __init__(self):
        super().__init__('pose_uploader')

        # Parámetros (puedes sobreescribirlos al lanzar):
        self.declare_parameter('endpoint', 'http://localhost:8000/add_pose/')
        self.declare_parameter('odom_topic', '/panther/odometry/filtered')
        self.declare_parameter('period_sec', 10.0)  # PUBLICA CADA 10s

        self.endpoint = self.get_parameter('endpoint').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.period = float(self.get_parameter('period_sec').value)

        self.latest_position = None  # (x, y, z)
        self.latest_quat = None      # (x, y, z, w)

        # Suscripción a /odom
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Temporizador de publicación
        self.create_timer(self.period, self.timer_callback)

        self.get_logger().info(
            f'PoseUploader listo. Odom="{self.odom_topic}", '
            f'endpoint="{self.endpoint}", periodo={self.period:.1f}s'
        )

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_position = (float(p.x), float(p.y), float(p.z))
        self.latest_quat = (float(q.x), float(q.y), float(q.z), float(q.w))

    def timer_callback(self):
        if self.latest_position is None or self.latest_quat is None:
            # Aún no llegó odometría; no spamear el log
            return

        roll, pitch, yaw = quat_to_rpy(*self.latest_quat)
        payload = {
            "pose": {
                "position": {
                    "x": self.latest_position[0],
                    "y": self.latest_position[1],
                    "z": self.latest_position[2],
                },
                "orientation": {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                },
            }
        }

        try:
            # Usamos 'json=' para cabecera Content-Type: application/json automáticamente
            resp = requests.post(self.endpoint, json=payload, timeout=10)
            if 200 <= resp.status_code < 300:
                self.get_logger().info(f'POST OK ({resp.status_code})')
            else:
                self.get_logger().error(
                    f'POST fallo: {resp.status_code} - {resp.text[:200]}'
                )
        except Exception as e:
            self.get_logger().error(f'POST error: {e}')


def main():
    rclpy.init()
    node = PoseUploader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
