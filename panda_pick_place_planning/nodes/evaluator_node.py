from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from panda_pick_place_planning.core.schemas import EpisodeResult


class EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("evaluator")
        self.declare_parameter("output_path", "results/ros2_episode_results.jsonl")
        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription = self.create_subscription(
            String,
            "/episode_result",
            self.on_result,
            qos,
        )

    def on_result(self, message: String) -> None:
        try:
            result = EpisodeResult.from_json(message.data)
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"无效 episode 结果: {exc}")
            return
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("a", encoding="utf-8", newline="\n") as file:
            file.write(result.to_json() + "\n")
        self.get_logger().info(
            f"结果已保存: {self.output_path.resolve()}，success={result.task_success}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
