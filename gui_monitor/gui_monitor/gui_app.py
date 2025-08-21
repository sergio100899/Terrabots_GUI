import sys
import rclpy

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QListWidget, QTextEdit, QPushButton, QLineEdit, QLabel
)
from PySide6.QtCore import Qt

from .ros_bridge import RosQtBridge, RosSubscriberNode, RosSpinThread


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 Jazzy GUI Monitor')
        self.resize(1000, 700)

        # Widgets principales
        self.refresh_btn = QPushButton('Refrescar tópicos')
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText('Filtro (regex)')
        self.topic_list = QListWidget()
        self.subscribe_btn = QPushButton('Suscribir')
        self.unsubscribe_btn = QPushButton('Desuscribir')
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.status_label = QLabel('Listo')

        # Layouts
        top_bar = QHBoxLayout()
        top_bar.addWidget(self.refresh_btn)
        top_bar.addWidget(self.filter_edit, 1)
        top_bar.addWidget(self.subscribe_btn)
        top_bar.addWidget(self.unsubscribe_btn)

        left_col = QVBoxLayout()
        left_col.addLayout(top_bar)
        left_col.addWidget(self.topic_list, 2)
        left_col.addWidget(QLabel('Mensajes'))
        left_col.addWidget(self.log_view, 3)

        root = QVBoxLayout()
        root.addLayout(left_col)
        root.addWidget(self.status_label)

        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)

        # Bridge Qt <-> ROS
        self.bridge = RosQtBridge()
        self.bridge.new_message.connect(self._on_new_message)
        self.bridge.topics_updated.connect(self._on_topics_updated)
        self.bridge.status.connect(self.status_label.setText)

        # Arrancar ROS
        rclpy.init()
        self.node = RosSubscriberNode(self.bridge)
        self.spin = RosSpinThread(self.node, self.bridge)
        self.spin.start()

        # Conexiones UI
        self.refresh_btn.clicked.connect(self.node.list_topics)
        self.topic_list.itemDoubleClicked.connect(self._subscribe_selected)
        self.subscribe_btn.clicked.connect(self._subscribe_selected)
        self.unsubscribe_btn.clicked.connect(self._unsubscribe_selected)
        self.filter_edit.textChanged.connect(self._apply_filter)

        # Primera carga
        self.node.list_topics()

        self._all_topics = []

    # Slots
    def _on_new_message(self, topic: str, payload: str):
        self.log_view.append(f"<b>{topic}</b>: {payload}")

    def _on_topics_updated(self, topics: list[str]):
        self._all_topics = topics
        self._apply_filter()
        self.status_label.setText(f"{len(topics)} tópicos encontrados")

    def _apply_filter(self):
        import re
        self.topic_list.clear()
        pattern = self.filter_edit.text().strip()
        if not pattern:
            self.topic_list.addItems(self._all_topics)
            return
        try:
            rx = re.compile(pattern)
            filtered = [t for t in self._all_topics if rx.search(t)]
        except re.error:
            filtered = self._all_topics
        self.topic_list.addItems(filtered)

    def _selected_topic(self) -> str | None:
        item = self.topic_list.currentItem()
        return item.text() if item else None

    def _subscribe_selected(self):
        topic = self._selected_topic()
        if topic:
            self.node.subscribe(topic)

    def _unsubscribe_selected(self):
        topic = self._selected_topic()
        if topic:
            self.node.unsubscribe(topic)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
