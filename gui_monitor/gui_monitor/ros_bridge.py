import threading
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

from PySide6.QtCore import QObject, Signal


class RosQtBridge(QObject):
    """Puente de señales Qt <-> ROS"""
    new_message = Signal(str, str)        # (topic, payload)
    topics_updated = Signal(list)         # lista de (topic_name)
    status = Signal(str)


class RosSubscriberNode(Node):
    def __init__(self, bridge: RosQtBridge):
        super().__init__('ros2_gui_monitor')
        self.bridge = bridge
        self._subs: Dict[str, object] = {}
        self._qos_default = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

    # --- Descubrimiento ---
    def list_topics(self):
        topics_and_types = self.get_topic_names_and_types()
        topics = [name for name, _ in topics_and_types]
        self.bridge.topics_updated.emit(sorted(topics))

    # --- Suscripción dinámica con introspección ---
    def subscribe(self, topic_name: str, qos: Optional[QoSProfile] = None):
        if topic_name in self._subs:
            return
        msg_type_str = self._resolve_msg_type(topic_name)
        if not msg_type_str:
            self.get_logger().warn(f"No se pudo determinar el tipo de mensaje de {topic_name}")
            return
        try:
            msg_cls = get_message(msg_type_str)
        except Exception as e:
            self.get_logger().error(f"Error cargando tipo {msg_type_str}: {e}")
            return
        qos = qos or self._qos_default
        sub = self.create_subscription(msg_cls, topic_name, self._cb_factory(topic_name), qos)
        self._subs[topic_name] = sub
        self.get_logger().info(f"Suscrito a {topic_name} ({msg_type_str})")

    def unsubscribe(self, topic_name: str):
        sub = self._subs.pop(topic_name, None)
        if sub:
            self.destroy_subscription(sub)
            self.get_logger().info(f"Desuscrito de {topic_name}")

    def _resolve_msg_type(self, topic_name: str) -> Optional[str]:
        for name, types in self.get_topic_names_and_types():
            if name == topic_name and types:
                return types[0]
        return None

    def _cb_factory(self, topic_name: str):
        def _cb(msg):
            try:
                payload = self._safe_to_str(msg)
            except Exception:
                payload = repr(msg)
            self.bridge.new_message.emit(topic_name, payload)
        return _cb

    @staticmethod
    def _safe_to_str(msg) -> str:
        text = str(msg)
        if len(text) > 5000:
            text = text[:5000] + '... [truncado]'
        return text


class RosSpinThread:
    """Executor ROS2 en hilo dedicado, seguro para Qt."""
    def __init__(self, node: RosSubscriberNode, bridge: RosQtBridge):
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._spin, name='ros_spin', daemon=True)
        self._bridge = bridge

    def start(self):
        self._thread.start()

    def _spin(self):
        try:
            self._bridge.status.emit('ROS ejecutándose')
            self._executor.spin()
        finally:
            self._bridge.status.emit('ROS detenido')

    def stop(self):
        self._executor.shutdown()
