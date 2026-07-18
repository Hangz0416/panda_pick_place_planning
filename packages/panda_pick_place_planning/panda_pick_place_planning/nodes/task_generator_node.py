from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from panda_pick_place_interfaces.msg import ManipulationTask

from panda_pick_place_planning.core.schemas import TaskSpec
from panda_pick_place_planning.core.tasks import generate_task
from panda_pick_place_planning.nodes.ros_conversions import task_to_ros


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
        self.declare_parameter("backend", "pybullet")
        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("object_variant", "cube")
        self.declare_parameter("workpiece_count", 1)
        self.declare_parameter("perception_profile", "nominal")
        self.declare_parameter("use_gazebo", False)
        self.declare_parameter("simulator", "")
        self.publisher = self.create_publisher(
            ManipulationTask, "/manipulation_task", task_qos()
        )
        self.published = False
        self.timer = self.create_timer(1.0, self.publish_task_once)

    def publish_task_once(self) -> None:
        if self.published:
            return
        seed = int(self.get_parameter("seed").value)
        planner = str(self.get_parameter("planner").value)
        scene = str(self.get_parameter("scene").value)
        backend = str(self.get_parameter("backend").value)
        grasp_mode = str(self.get_parameter("grasp_mode").value)
        object_variant = str(self.get_parameter("object_variant").value)
        workpiece_count = int(self.get_parameter("workpiece_count").value)
        perception_profile = str(self.get_parameter("perception_profile").value)
        use_gazebo = bool(self.get_parameter("use_gazebo").value)
        simulator = str(self.get_parameter("simulator").value) or (
            "pybullet"
            if backend == "pybullet"
            else "gazebo"
            if use_gazebo
            else "moveit_mock"
        )
        task = generate_task(
            0,
            seed,
            planner,
            scene,
            object_variant=object_variant,
            workpiece_count=workpiece_count,
        )
        # ``to_dict`` deliberately serializes nested WorkpieceSpec objects to
        # dictionaries.  Re-enter through the schema parser so multi-object
        # tasks restore those nested values before ROS message conversion.
        task = TaskSpec.from_dict(
            {
                **task.to_dict(),
                "backend": backend,
                "grasp_mode": grasp_mode,
                "simulator": simulator,
                "perception_profile": perception_profile,
            }
        )
        message = task_to_ros(task)
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)
        self.published = True
        self.get_logger().info(
            f"已发布任务: backend={backend}, simulator={simulator}, "
            f"seed={seed}, planner={planner}, scene={scene}, "
            f"workpieces={workpiece_count}"
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
