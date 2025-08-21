#!/usr/bin/env python3
import sys
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QListWidget, QLabel, QPushButton, QGroupBox, QSizePolicy, QFrame
)


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
        self.setWindowTitle("Panel de Control del Robot")
        self.resize(1280, 800)

        root = QHBoxLayout()

        # ─────────────── Lista de tópicos ───────────────
        topics_box = QGroupBox("Tópicos (auto)")
        topics_box.setStyleSheet("QGroupBox { font-weight:bold; font-size:14px; color:#333; }")
        topics_v = QVBoxLayout()
        self.topic_list = QListWidget()
        self.topic_list.setStyleSheet("""
            QListWidget {
                background-color: #f0f0f0; border: 1px solid #ccc; padding:5px;
            }
            QListWidget::item:selected {
                background-color: #4CAF50; color: white;
            }
        """)
        self.topic_status = QLabel("—")
        self.topic_status.setStyleSheet("color: gray;")
        topics_v.addWidget(self.topic_list, 1)
        topics_v.addWidget(self.topic_status)
        topics_box.setLayout(topics_v)
        topics_box.setMinimumWidth(360)
        root.addWidget(topics_box, 1)

        # ─────────────── Controles del robot ───────────────
        controls_box = QGroupBox("Controles del robot")
        controls_box.setStyleSheet("QGroupBox { font-weight:bold; font-size:14px; color:#333; }")
        controls_v = QVBoxLayout()

        # Joystick estilo diamond
        joystick_box = QGroupBox("Joystick")
        joystick_box.setStyleSheet("QGroupBox { font-weight:bold; font-size:13px; color:#555; }")
        joystick_layout = QGridLayout()

        self.btn_up = QPushButton("↑")
        self.btn_down = QPushButton("↓")
        self.btn_left = QPushButton("←")
        self.btn_right = QPushButton("→")
        self.btn_stop = QPushButton("⏹")

        for b in (self.btn_up, self.btn_down, self.btn_left, self.btn_right, self.btn_stop):
            b.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50; color: white; font-size: 18px;
                    border-radius: 30px; padding: 10px;
                }
                QPushButton:hover { background-color: #45a049; }
                QPushButton:pressed { background-color: #388E3C; }
            """)
            b.setFixedSize(60, 60)

        joystick_layout.addWidget(self.btn_up, 0, 1)
        joystick_layout.addWidget(self.btn_left, 1, 0)
        joystick_layout.addWidget(self.btn_stop, 1, 1)
        joystick_layout.addWidget(self.btn_right, 1, 2)
        joystick_layout.addWidget(self.btn_down, 2, 1)

        joystick_box.setLayout(joystick_layout)
        controls_v.addWidget(joystick_box)

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
        cameras_box.setStyleSheet("QGroupBox { font-weight:bold; font-size:14px; color:#333; }")
        grid = QGridLayout()
        self.cam_labels = []
        for i in range(4):
            lbl = QLabel(f"Cam {i+1}\n(pendiente)")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setMinimumSize(320, 240)
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

        # Contenedor central
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

    def closeEvent(self, event):
        try:
            self.refresh_timer.stop()
            self.spin_timer.stop()
            self.node.destroy_node()
            rclpy.shutdown()
        finally:
            super().closeEvent(event)


def main():
    rclpy.init()
    node = RosUINode()
    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
