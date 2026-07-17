"""Publish canonical Panda joint names from Fortress ros2_control state."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


FORTRESS_MIMIC_NAMES = {
    "panda_finger_joint2_mimic": "panda_finger_joint2",
}


def canonical_joint_name(name: str) -> str:
    """Map gz_ros2_control's Humble mimic interface to the URDF joint name."""
    return FORTRESS_MIMIC_NAMES.get(name, name)


def canonicalize_joint_state(message: JointState) -> JointState:
    """Copy one JointState while replacing only the audited Panda alias."""
    canonical = JointState()
    canonical.header = message.header
    canonical.name = [canonical_joint_name(name) for name in message.name]
    if len(set(canonical.name)) != len(canonical.name):
        raise ValueError("canonical joint names are not unique")
    canonical.position = list(message.position)
    canonical.velocity = list(message.velocity)
    canonical.effort = list(message.effort)
    return canonical


class JointStateCanonicalizer(Node):
    """Isolate a Fortress-specific ros2_control naming detail from MoveIt."""

    def __init__(self) -> None:
        super().__init__("panda_joint_state_canonicalizer")
        self.declare_parameter(
            "input_topic", "/joint_state_broadcaster/joint_states"
        )
        self.declare_parameter("output_topic", "/joint_states")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if input_topic == output_topic:
            raise ValueError("joint-state canonicalizer input and output must differ")
        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_state, 10)
        self._mapping_reported = False

    def _on_state(self, message: JointState) -> None:
        try:
            canonical = canonicalize_joint_state(message)
        except ValueError as exc:
            self.get_logger().error(f"dropping invalid joint state: {exc}")
            return
        if not self._mapping_reported and canonical.name != list(message.name):
            self.get_logger().info(
                "mapped panda_finger_joint2_mimic to panda_finger_joint2"
            )
            self._mapping_reported = True
        self._publisher.publish(canonical)


def main(args=None) -> None:
    """Run the canonical joint-state relay."""
    rclpy.init(args=args)
    node = JointStateCanonicalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
