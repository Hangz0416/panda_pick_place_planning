from __future__ import annotations

from pathlib import Path
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from panda_pick_place_interfaces.msg import EpisodeResult as EpisodeResultMsg
from std_msgs.msg import Bool, UInt64

from panda_pick_place_planning.nodes.ros_conversions import episode_from_ros


class EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("evaluator")
        self.declare_parameter("output_path", "results/ros2_episode_results.jsonl")
        self.declare_parameter("truncate_output", False)
        configured_path = str(self.get_parameter("output_path").value).strip()
        self.output_path = (
            Path(configured_path).expanduser()
            if configured_path
            else Path.home()
            / ".ros"
            / "panda_pick_place"
            / "runs"
            / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            / "episodes.jsonl"
        )
        if bool(self.get_parameter("truncate_output").value):
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_path.write_text("", encoding="utf-8")
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(
            EpisodeResultMsg,
            "/episode_result",
            self.on_result,
            qos,
        )
        self.saved_publisher = self.create_publisher(
            UInt64, "/episode_result_saved", qos
        )
        self.create_subscription(
            Bool, "/demo_application_shutdown", self._on_shutdown, 10
        )

    def on_result(self, message: EpisodeResultMsg) -> None:
        try:
            result = episode_from_ros(message)
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"无效 episode 结果: {exc}")
            return
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("a", encoding="utf-8", newline="\n") as file:
            file.write(result.to_json() + "\n")
            file.flush()
            os.fsync(file.fileno())
        self.saved_publisher.publish(UInt64(data=result.episode_id))
        self.get_logger().info(
            f"结果已保存: {self.output_path.resolve()}，success={result.task_success}"
        )

    @staticmethod
    def _on_shutdown(message: Bool) -> None:
        if message.data and rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
