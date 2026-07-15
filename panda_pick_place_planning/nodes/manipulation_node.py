from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv
from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec


def reliable_qos(transient_local: bool = False) -> QoSProfile:
    return QoSProfile(
        depth=10,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
        reliability=ReliabilityPolicy.RELIABLE,
    )


class ManipulationNode(Node):
    def __init__(self) -> None:
        super().__init__("manipulation")
        self.declare_parameter("use_gui", True)
        self.env = PyBulletPandaEnv(use_gui=bool(self.get_parameter("use_gui").value))
        self.runner = PickPlaceRunner(self.env)
        self.busy = False
        self.result_publisher = self.create_publisher(
            String,
            "/episode_result",
            reliable_qos(transient_local=True),
        )
        self.task_subscription = self.create_subscription(
            String,
            "/manipulation_task",
            self.on_task,
            reliable_qos(transient_local=True),
        )

    def on_task(self, message: String) -> None:
        if self.busy:
            self.get_logger().warning("上一条任务仍在执行，忽略重复任务。")
            return
        self.busy = True
        task = None
        try:
            task = TaskSpec.from_json(message.data)
            self.get_logger().info(
                f"开始执行: planner={task.planner}, scene={task.scene}, seed={task.task_seed}"
            )
            result = self.runner.run(task)
        except Exception as exc:  # ROS2 边界需要将运行时错误转为可观测结果。
            self.get_logger().error(f"任务执行异常: {exc}")
            if task is None:
                self.busy = False
                return
            result = runtime_error_result(task)
        finally:
            self.busy = False

        self.result_publisher.publish(String(data=result.to_json()))
        self.get_logger().info(
            f"执行结束: success={result.task_success}, failure={result.failure_type or '-'}"
        )

    def destroy_node(self):
        self.env.close()
        return super().destroy_node()


def runtime_error_result(task: TaskSpec) -> EpisodeResult:
    return EpisodeResult(
        episode_id=task.episode_id,
        task_seed=task.task_seed,
        planner=task.planner,
        planner_seed=task.planner_seed,
        scene=task.scene,
        planning_success=False,
        planning_time_sec=0.0,
        explored_nodes=0,
        path_waypoints=0,
        path_length_rad=0.0,
        execution_sim_steps=0,
        task_success=False,
        collision_count=0,
        failure_type="runtime_error",
        object_initial_position=task.object_position,
        object_final_position=task.object_position,
        target_position=task.target_position,
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
