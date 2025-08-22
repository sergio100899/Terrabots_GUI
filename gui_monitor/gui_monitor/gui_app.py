#!/usr/bin/env python3

from geometry_msgs.msg import TwistStamped

import sys
from typing import List, Tuple
import signal

import rclpy
from rclpy.node import Node

from PySide6.QtCore import Qt, QTimer, QPointF, Signal, QSize
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QListWidget, QLabel, QPushButton, QGroupBox, QSizePolicy, QFrame
)
from PySide6.QtGui import QImage, QPixmap, QPainter, QColor, QPen

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


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
        knob_radius = 20
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#4CAF50"))
        painter.drawEllipse(self.knob_pos, knob_radius, knob_radius)

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
    def __init__(self, node: RosUINode):
        super().__init__()
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.current_dx = 0.0
        self.current_dy = 0.0
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.publish_cmd_vel)
        self.cmd_timer.start(100)  # publica cada 100ms
        self.setWindowTitle("Panel de Control del Robot")
        self.resize(1280, 800)

        root = QHBoxLayout()

        # ─────────────── Lista de tópicos ───────────────
        topics_box = QGroupBox("Tópicos (auto)")
        topics_v = QVBoxLayout()
        self.topic_list = QListWidget()
        self.topic_status = QLabel("—")
        topics_v.addWidget(self.topic_list, 1)
        topics_v.addWidget(self.topic_status)
        topics_box.setLayout(topics_v)
        topics_box.setMinimumWidth(360)
        root.addWidget(topics_box, 1)

        # ─────────────── Controles del robot ───────────────
        controls_box = QGroupBox("Controles del robot")
        controls_v = QVBoxLayout()

        # Joystick virtual
        self.joystick = VirtualJoystick()
        self.joystick.moved.connect(self.handle_joystick_move)
        self.joystick.released.connect(self.handle_joystick_release)
        controls_v.addWidget(self.joystick, alignment=Qt.AlignCenter)

        self.linear_vel_label = QLabel("Velocidad Lineal: 0.00 m/s")
        self.angular_vel_label = QLabel("Velocidad Angular: 0.00 rad/s")
        self.linear_vel_label.setStyleSheet("font-size: 14px; color: #222;")
        self.angular_vel_label.setStyleSheet("font-size: 14px; color: #222;")
        controls_v.addWidget(self.linear_vel_label)
        controls_v.addWidget(self.angular_vel_label)

        # Placeholder para telemetría
        placeholder_controls = QLabel("Telemetría / Estado / Diagnósticos…")
        placeholder_controls.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        placeholder_controls.setAlignment(Qt.AlignCenter)
        placeholder_controls.setStyleSheet("background:#222; color:#eee; font-size:14px;")
        placeholder_controls.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        controls_v.addWidget(placeholder_controls, 1)

        controls_box.setLayout(controls_v)
        root.addWidget(controls_box, 1)

        # Placeholder para telemetría y estado
        placeholder_controls = QLabel("Telemetría / Estado / Diagnósticos…")
        placeholder_controls.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        placeholder_controls.setAlignment(Qt.AlignCenter)
        placeholder_controls.setStyleSheet("background:#222; color:#eee; font-size:14px;")
        placeholder_controls.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        controls_v.addWidget(placeholder_controls, 1)

        controls_box.setLayout(controls_v)
        root.addWidget(controls_box, 1)

        # ─────────────── Cámaras ───────────────
        cameras_box = QGroupBox("Cámaras")
        grid = QGridLayout()
        self.cam_labels = []
        for i in range(4):
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
            grid.addWidget(lbl, i // 2, i % 2)
        cameras_box.setLayout(grid)
        root.addWidget(cameras_box, 1)

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
        msg.twist.linear.x = self.current_dy * 0.2     # escala máxima: 0.2 m/s
        msg.twist.angular.z = self.current_dx * 1.0    # escala máxima: 1.0 rad/s
        self.cmd_vel_pub.publish(msg)

    def publish_cmd_vel(self):
        
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x = self.current_dy * 0.5
        msg.twist.angular.z = self.current_dx * 1.0
        self.cmd_vel_pub.publish(msg)

        # Actualizar labels con valores en vivo
        self.linear_vel_label.setText(f"Velocidad Lineal: {msg.twist.linear.x:.2f} m/s")
        self.angular_vel_label.setText(f"Velocidad Angular: {msg.twist.angular.z:.2f} rad/s")


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
    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()

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
        for n in cam_nodes:
            rclpy.spin_once(n, timeout_sec=0.01)

    spin_timer = QTimer()
    spin_timer.timeout.connect(spin_all)
    spin_timer.start(10)


    signal.signal(signal.SIGINT, handle_sigint)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
