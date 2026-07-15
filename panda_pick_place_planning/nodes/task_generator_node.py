from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from panda_pick_place_planning.core.tasks import generate_task


def task_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class TaskGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("task_generator")
        self.declare_parameter("seed", 42)
        self.declare_parameter("planner", "rrt_connect")
        self.declare_parameter("scene", "obstacle")
        self.publisher = self.create_publisher(String, "/manipulation_task", task_qos())
        self.published = False
        self.timer = self.create_timer(1.0, self.publish_task_once)

    def publish_task_once(self) -> None:
        if self.published:
            return
        seed = int(self.get_parameter("seed").value)
        planner = str(self.get_parameter("planner").value)
        scene = str(self.get_parameter("scene").value)
        task = generate_task(0, seed, planner, scene)
        self.publisher.publish(String(data=task.to_json()))
        self.published = True
        self.get_logger().info(
            f"已发布任务: seed={seed}, planner={planner}, scene={scene}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
