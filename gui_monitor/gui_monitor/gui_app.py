#!/usr/bin/env python3

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import sys
from typing import List, Tuple
import signal

import rclpy
from rclpy.node import Node

from PySide6.QtCore import Qt, QTimer, QPointF, Signal, QSize, QBuffer, QIODevice, QThread, QObject


from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QListWidget, QLabel, QPushButton, QGroupBox, QSizePolicy, QFrame
)
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor, QPen

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import os
import random
import string

import io
import requests
import json


class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # evitar warning unused
        self.position = None
        self.timestamp = None

    def listener_callback(self, msg: Odometry):
        # Guardamos la posición y el timestamp cada vez que llega un mensaje
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

class UploadWorker(QObject):
    finished = Signal(str)
    error = Signal(str)

    def __init__(self, img_bytes: bytes, cam_id: int, position=None, timestamp=None):  ### NUEVO
        super().__init__()
        self.img_bytes = img_bytes
        self.cam_id = cam_id
        self.position = position
        self.timestamp = timestamp

    def run(self):
        try:
            files = {
                "image": (f"cam{self.cam_id+1}.png", self.img_bytes, "image/png")
            }
            metadata = {
                "position": {
                    "x": self.position[0] if self.position else 0.0,
                    "y": self.position[1] if self.position else 0.0,
                    "z": self.position[2] if self.position else 0.0
                },
                "timestamp": int(self.timestamp) if self.timestamp else 0
            }
            data = {"metadata_json": json.dumps(metadata)}

            print(data)

            response = requests.post(
                "http://localhost:8000/add_landmark/",
                files=files,
                data=data,
                timeout=60
            )
            if response.status_code == 201:
                print("Response ok")
            else:
                self.error.emit(f"Error {response.status_code}: {response.text}")
        except Exception as e:
            self.error.emit(str(e))


class TrajectoryCanvas(QWidget):
    def __init__(self, odom_node: OdometrySubscriber):
        super().__init__()
        self.odom_node = odom_node
        self.positions = []  # lista de (x, y)
        self.setMinimumHeight(250)
        self.scale = 40.0  # metros → pixeles
        self.offset = QPointF(200, 200)  # centro del canvas

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        qp.fillRect(self.rect(), Qt.black)

        # ejes
        qp.setPen(QPen(Qt.gray, 1, Qt.DashLine))
        qp.drawLine(0, self.offset.y(), self.width(), self.offset.y())
        qp.drawLine(self.offset.x(), 0, self.offset.x(), self.height())

        # trayectoria
        if len(self.positions) > 1:
            qp.setPen(QPen(Qt.green, 2))
            prev = self.to_canvas(self.positions[0])
            for pos in self.positions[1:]:
                current = self.to_canvas(pos)
                qp.drawLine(prev, current)
                prev = current

        # robot actual
        if self.positions:
            qp.setBrush(QColor(255, 0, 0))
            qp.setPen(QPen(Qt.red, 2))
            last = self.to_canvas(self.positions[-1])
            qp.drawEllipse(last, 5, 5)

    def to_canvas(self, pos):
        x, y = pos
        return QPointF(self.offset.x() + x * self.scale,
                       self.offset.y() - y * self.scale)

    def update_positions(self):
        if self.odom_node.position is not None:
            x, y, _ = self.odom_node.position
            self.positions.append((x, y))
            if len(self.positions) > 1000:  # limitar buffer
                self.positions.pop(0)
        self.update()



class VirtualJoystick(QWidget):
    moved = Signal(float, float)  # dx, dy
    released = Signal()

    def __init__(self):
        super().__init__()
        self.setMinimumSize(160, 160)
        self.setMaximumSize(200, 200)
        self.setMouseTracking(True)
        self.center = QPointF(self.width() / 2, self.height() / 2)
        self.knob_pos = self.center
        self.pressed = False

    def resizeEvent(self, event):
        self.center = QPointF(self.width() / 2, self.height() / 2)
        if not self.pressed:
            self.knob_pos = self.center

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background circle
        radius = min(self.width(), self.height()) / 2 - 5
        painter.setPen(QPen(QColor("#888"), 4))
        painter.setBrush(QColor("#ccc"))
        painter.drawEllipse(self.center, radius, radius)

        # Knob
        # knob_radius = 20
        # painter.setPen(Qt.NoPen)
        # painter.setBrush(QColor("#4CAF50"))
        # painter.drawEllipse(self.knob_pos, knob_radius, knob_radius)

        knob_radius = 20
        dx = self.knob_pos.x() - self.center.x()
        dy = self.knob_pos.y() - self.center.y()
        intensity = min(1.0, (dx**2 + dy**2)**0.5 / (self.width() / 2 - 20))

        color = QColor.fromHsvF(0.33 * (1 - intensity), 1.0, 1.0)  # verde → rojo
        painter.setPen(Qt.NoPen)
        painter.setBrush(color)
        painter.drawEllipse(self.knob_pos, knob_radius, knob_radius)

        painter.end()

    def mousePressEvent(self, event):
        self.pressed = True
        self._update_knob(event.pos())

    def mouseMoveEvent(self, event):
        if self.pressed:
            self._update_knob(event.pos())

    def mouseReleaseEvent(self, event):
        self.pressed = False
        self.knob_pos = self.center
        self.released.emit()
        self.update()

    def _update_knob(self, pos):
        dx = pos.x() - self.center.x()
        dy = pos.y() - self.center.y()
        radius = min(self.width(), self.height()) / 2 - 20

        dist = (dx**2 + dy**2)**0.5
        if dist > radius:
            scale = radius / dist
            dx *= scale
            dy *= scale

        self.knob_pos = QPointF(self.center.x() + dx, self.center.y() + dy)
        self.update()

        norm_dx = dx / radius
        norm_dy = -dy / radius  # invertir eje Y
        self.moved.emit(norm_dx, norm_dy)

class RosUINode(Node):
    """Nodo ROS para consultas ligeras desde la UI."""
    def __init__(self):
        super().__init__('robot_control_panel')

    def topic_names_and_types(self) -> List[Tuple[str, List[str]]]:
        return self.get_topic_names_and_types()


class MainWindow(QMainWindow):
    def __init__(self, node: RosUINode, odom_node: OdometrySubscriber):
        super().__init__()
        self.node = node
        self.odom_node = odom_node
        self.cmd_vel_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.current_dx = 0.0
        self.current_dy = 0.0
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.publish_cmd_vel)
        self.cmd_timer.start(100)  # publica cada 100ms
        self.setWindowTitle("Panel de Control del Robot")
        self.resize(1280, 800)

        root = QHBoxLayout()

        left_col = QVBoxLayout()

        # ─────────────── Lista de tópicos ───────────────
        topics_box = QGroupBox("Tópicos (auto)")
        topics_v = QVBoxLayout()
        self.topic_list = QListWidget()
        self.topic_status = QLabel("—")
        topics_v.addWidget(self.topic_list, 1)
        topics_v.addWidget(self.topic_status)
        topics_box.setLayout(topics_v)
        topics_box.setMinimumWidth(360)
        left_col.addWidget(topics_box, 0)

        # ─────────────── Trayectoria ───────────────
        trajectory_box = QGroupBox("Trayectoria del robot")
        traj_layout = QVBoxLayout()
        self.traj_canvas = TrajectoryCanvas(self.odom_node)
        traj_layout.addWidget(self.traj_canvas)
        trajectory_box.setLayout(traj_layout)
        left_col.addWidget(trajectory_box, 0)

        root.addLayout(left_col, 0)

        # ─────────────── Controles del robot ───────────────
        controls_box = QGroupBox("Controles del robot")
        controls_v = QVBoxLayout()

        # Joystick virtual
        self.joystick = VirtualJoystick()
        self.joystick.moved.connect(self.handle_joystick_move)
        self.joystick.released.connect(self.handle_joystick_release)
        controls_v.addWidget(self.joystick, alignment=Qt.AlignCenter)

        # ─────────────── Bloque: Velocidades en vivo ───────────────
        vel_box = QGroupBox("Velocidades (cmd_vel)")
        vel_layout = QVBoxLayout()

        self.linear_vel_label = QLabel("Lineal: 0.00 m/s")
        self.angular_vel_label = QLabel("Angular: 0.00 rad/s")
        for lbl in [self.linear_vel_label, self.angular_vel_label]:
            lbl.setStyleSheet("font-size: 14px; color: #0f0; background:#222; padding:4px;")

        vel_layout.addWidget(self.linear_vel_label)
        vel_layout.addWidget(self.angular_vel_label)
        vel_box.setLayout(vel_layout)

        controls_v.addWidget(vel_box)

        # ─────────────── Bloque: Odometría ───────────────
        odom_box = QGroupBox("Odometría")
        odom_layout = QVBoxLayout()

        self.odom_pos_label = QLabel("Posición: x=0.00, y=0.00, z=0.00")
        self.odom_time_label = QLabel("Timestamp: 0.00")

        for lbl in [self.odom_pos_label, self.odom_time_label]:
            lbl.setStyleSheet("font-size: 14px; color: #0af; background:#222; padding:4px;")

        odom_layout.addWidget(self.odom_pos_label)
        odom_layout.addWidget(self.odom_time_label)
        odom_box.setLayout(odom_layout)

        controls_v.addWidget(odom_box)

        # Placeholder para telemetría y estado
        placeholder_controls = QLabel("Telemetría / Estado / Diagnósticos…")
        placeholder_controls.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        placeholder_controls.setAlignment(Qt.AlignCenter)
        placeholder_controls.setStyleSheet("background:#222; color:#eee; font-size:14px;")
        placeholder_controls.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        controls_v.addWidget(placeholder_controls, 1)

        controls_box.setLayout(controls_v)
        root.addWidget(controls_box, 0)

        # ─────────────── Cámaras ───────────────
        cameras_box = QGroupBox("Cámaras")
        grid = QGridLayout()
        self.cam_labels = []
        self.screenshot_buttons = []

        camera_names = ["Frontal", "Trasera", "Izquierda", "Derecha"]

        for i, cam_name in enumerate(camera_names):
            vbox = QVBoxLayout()
            
            # Título encima de la cámara
            title = QLabel(f"Cámara {cam_name}")
            title.setAlignment(Qt.AlignCenter)
            title.setStyleSheet("font-weight: bold; color: #fff;")
            vbox.addWidget(title)

            # QLabel de cámara
            lbl = QLabel(f"Cam {i+1}\n(pendiente)")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setMinimumSize(400, 250)
            lbl.setMaximumHeight(300)
            lbl.setFrameStyle(QFrame.Panel | QFrame.Raised)
            lbl.setStyleSheet("""
                background:#111; color:#bbb;
                border: 2px solid #444; border-radius: 8px;
            """)
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.cam_labels.append(lbl)

            # Botón de screenshot
            btn = QPushButton(f"Screenshot Cam {i+1}")
            btn.clicked.connect(lambda _, cam_id=i: self.take_screenshot(cam_id))
            self.screenshot_buttons.append(btn)

            vbox.addWidget(lbl)
            vbox.addWidget(btn)

            grid.addLayout(vbox, i // 2, i % 2)


        cameras_box.setLayout(grid)
        root.addWidget(cameras_box, 0)

        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)

        # ─────────────── Timers ROS ───────────────
        self._last_topics_key = ""
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(10)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_topics)
        self.refresh_timer.start(1000)
        self._refresh_topics()

    def take_screenshot(self, cam_id: int):
        pixmap = self.cam_labels[cam_id].pixmap()
        if pixmap is None:
            print(f"[WARN] No hay imagen en Cam {cam_id+1}")
            return

        qimage = pixmap.toImage()
        buffer = QBuffer()
        buffer.open(QIODevice.ReadWrite)
        qimage.save(buffer, "PNG")
        img_bytes = bytes(buffer.data())   # importante: convertir a bytes nativos
        buffer.close()

        # Crear worker + hilo
        self.thread = QThread()
        self.worker = UploadWorker(img_bytes, cam_id, position=self.odom_node.position, timestamp=self.odom_node.timestamp)
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.on_report_ready)
        self.worker.error.connect(self.on_report_error)

        # limpiar al terminar
        self.worker.finished.connect(self.thread.quit)
        self.worker.error.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.error.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)

        self.thread.start()
        print(f"[INFO] Enviando screenshot de Cam {cam_id+1} en segundo plano...")

    def on_report_ready(self, filepath: str):
        print(f"[INFO] Reporte guardado en: {filepath}")

    def on_report_error(self, msg: str):
        print(f"[ERROR] Fallo al generar reporte: {msg}")



    def _spin_once(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as e:
            self.topic_status.setText(f"spin_once error: {e}")

    def _refresh_topics(self):
        try:
            topics = self.node.topic_names_and_types()
            key = "|".join(f"{n}:{','.join(t)}" for n, t in sorted(topics))
            if key == self._last_topics_key:
                return
            self._last_topics_key = key
            self.topic_list.clear()
            for name, types in sorted(topics):
                type_str = ", ".join(types) if types else "?"
                self.topic_list.addItem(f"{name}  —  {type_str}")
            self.topic_status.setText(f"{len(topics)} tópicos detectados")
        except Exception as e:
            self.topic_status.setText(f"Error listando tópicos: {e}")


    def publish_cmd_vel(self):
        
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x = self.current_dy * 0.2
        msg.twist.angular.z = -self.current_dx * 1.0
        self.cmd_vel_pub.publish(msg)

        # Actualizar labels con valores en vivo
        self.linear_vel_label.setText(f"Velocidad Lineal: {msg.twist.linear.x:.2f} m/s")
        self.angular_vel_label.setText(f"Velocidad Angular: {msg.twist.angular.z:.2f} rad/s")

    def update_odometry_labels(self):
        if self.odom_node.position is not None:
            x, y, z = self.odom_node.position
            ts = self.odom_node.timestamp
            self.odom_pos_label.setText(f"Posición: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            self.odom_time_label.setText(f"Timestamp: {int(ts)}")



    def handle_joystick_move(self, dx: float, dy: float):
        self.current_dx = dx
        self.current_dy = dy

    def handle_joystick_release(self):
        self.current_dx = 0.0
        self.current_dy = 0.0

    def closeEvent(self, event):
        try:
            self.refresh_timer.stop()
            self.spin_timer.stop()
            self.node.destroy_node()
            rclpy.shutdown()
        finally:
            super().closeEvent(event)

class ImageSubscriber(Node):
    def __init__(self, label: QLabel, topic="/camera/image_raw"):
        super().__init__('image_viewer')
        self.label = label
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(f"[ERROR] cv_bridge: {e}")
            return

        # Convertir a QImage
        h, w, ch = cv_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_image)

        # Mostrar en la GUI
        self.label.setPixmap(pixmap.scaled(
            self.label.width(),
            self.label.height(),
            Qt.KeepAspectRatioByExpanding
        ))


def handle_sigint(*args):
    print("Ctrl+C detectado, cerrando GUI...")
    QApplication.quit()  # Esto cierra la app Qt limpia


def main():
    rclpy.init()
    node = RosUINode()
    odom_node = OdometrySubscriber()
    app = QApplication(sys.argv)
    win = MainWindow(node, odom_node)
    win.show()

    app.setStyleSheet("""
        QMainWindow {
            background-color: #1e1e1e;
        }

        QGroupBox {
            border: 1px solid #444;
            border-radius: 5px;
            margin-top: 12px;
            font-weight: bold;
            color: #f0f0f0;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top center;
            padding: 4px;
        }

        QLabel {
            color: #ccc;
            font-size: 14px;
        }

        QListWidget {
            background-color: #2c2c2c;
            color: #eee;
            border: 1px solid #555;
            font-size: 13px;
            padding: 4px;
        }

        QPushButton {
            background-color: #3c3c3c;
            color: #eee;
            border: 1px solid #666;
            border-radius: 5px;
            padding: 6px 10px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #555;
        }

        QFrame {
            background-color: #222;
            color: #ddd;
        }
    """)

    # Lista de tópicos de cámara
    cam_topics = [
        "/camera/image_raw",
        # "/camera2/image_raw",
        # "/camera3/image_raw",
        # "/camera4/image_raw"
    ]

    # Lista de nodos de cámara
    cam_nodes = []
    for i, topic in enumerate(cam_topics):
        if i < len(win.cam_labels):
            cam_node = ImageSubscriber(win.cam_labels[i], topic)
            cam_nodes.append(cam_node)

    # QTimer para ejecutar spin_once en cada nodo de cámara
    def spin_all():
        for n in cam_nodes + [odom_node]:
            rclpy.spin_once(n, timeout_sec=0.01)
        win.update_odometry_labels()
        win.traj_canvas.update_positions()


    spin_timer = QTimer()
    spin_timer.timeout.connect(spin_all)
    spin_timer.start(10)


    signal.signal(signal.SIGINT, handle_sigint)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
