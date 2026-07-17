from __future__ import annotations

import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from panda_pick_place_interfaces.msg import EpisodeResult as EpisodeResultMsg
from panda_pick_place_interfaces.msg import ManipulationTask

from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.planning_regression import (
    PlanningRegressionRunner,
)
from panda_pick_place_planning.core.backends import (
    BackendOperationError,
    MoveItBackend,
    MoveItBackendConfig,
    PyBulletBackend,
)
from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec
from panda_pick_place_planning.core.pybullet_env import SceneConfig
from panda_pick_place_planning.nodes.ros_conversions import episode_to_ros, task_from_ros


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


def default_scene_config_path() -> Path:
    return (
        Path(get_package_share_directory("panda_pick_place_planning"))
        / "config"
        / "scene_defaults.yaml"
    )


class ManipulationNode(Node):
    def __init__(self) -> None:
        super().__init__("manipulation")
        self.callback_group = ReentrantCallbackGroup()
        self.task_lock = threading.Lock()
        self.declare_parameter("backend", "pybullet")
        self.declare_parameter("use_gui", True)
        self.declare_parameter("use_gazebo", False)
        self.declare_parameter("moveit_execute", False)
        self.declare_parameter("moveit_command_gripper", False)
        self.declare_parameter("moveit_collision_monitor_available", False)
        self.declare_parameter("moveit_planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("moveit_planning_time_sec", 3.0)
        self.declare_parameter("moveit_velocity_scaling", 0.2)
        self.declare_parameter("moveit_acceleration_scaling", 0.2)
        self.declare_parameter(
            "scene_config_path",
            str(default_scene_config_path()),
        )
        self.scene_config = SceneConfig.from_yaml(
            Path(str(self.get_parameter("scene_config_path").value)).expanduser()
        )
        self.backend_name = str(self.get_parameter("backend").value)
        self.backend = self._create_backend()
        self.runner = (
            PlanningRegressionRunner(self.backend)
            if bool(getattr(self.backend, "is_planning_only", False))
            else PickPlaceRunner(self.backend)
        )
        self.result_publisher = self.create_publisher(
            EpisodeResultMsg,
            "/episode_result",
            reliable_qos(transient_local=True),
        )
        self.task_subscription = self.create_subscription(
            ManipulationTask,
            "/manipulation_task",
            self.on_task,
            reliable_qos(transient_local=True),
            callback_group=self.callback_group,
        )

    def on_task(self, message: ManipulationTask) -> None:
        if not self.task_lock.acquire(blocking=False):
            self.get_logger().warning("上一条任务仍在执行，忽略重复任务。")
            return
        task = None
        try:
            task = task_from_ros(message)
            if task.backend != self.backend_name:
                raise ValueError(
                    f"task backend '{task.backend}' does not match node backend "
                    f"'{self.backend_name}'"
                )
            backend_simulator = str(
                getattr(self.backend, "simulator_name", self.backend_name)
            )
            if task.simulator and task.simulator != backend_simulator:
                raise ValueError(
                    f"task simulator '{task.simulator}' does not match backend "
                    f"simulator '{backend_simulator}'"
                )
            self.get_logger().info(
                f"开始执行: backend={task.backend}, planner={task.planner}, "
                f"scene={task.scene}, seed={task.task_seed}"
            )
            result = self.runner.run(task)
        except Exception as exc:  # ROS2 边界需要将运行时错误转为可观测结果。
            self.get_logger().error(f"任务执行异常: {exc}")
            if task is None:
                return
            failure_type = (
                exc.failure_type
                if isinstance(exc, BackendOperationError)
                else "runtime_error"
            )
            result = runtime_error_result(
                task,
                self.backend,
                failure_type=failure_type,
                failure_state=str(
                    getattr(getattr(self.runner, "state", None), "value", "")
                    or "runtime_boundary"
                ),
            )
        finally:
            self.task_lock.release()

        result_message = episode_to_ros(result)
        result_message.header.stamp = self.get_clock().now().to_msg()
        self.result_publisher.publish(result_message)
        self.get_logger().info(
            f"执行结束: success={result.task_success}, failure={result.failure_type or '-'}"
        )

    def destroy_node(self):
        self.backend.shutdown()
        return super().destroy_node()

    def _create_backend(self):
        if self.backend_name == "pybullet":
            return PyBulletBackend(
                use_gui=bool(self.get_parameter("use_gui").value),
                config=self.scene_config,
            )
        if self.backend_name == "moveit":
            config = MoveItBackendConfig(
                use_gazebo=bool(self.get_parameter("use_gazebo").value),
                execute_motion=bool(self.get_parameter("moveit_execute").value),
                command_gripper=bool(
                    self.get_parameter("moveit_command_gripper").value
                ),
                collision_monitor_available=bool(
                    self.get_parameter(
                        "moveit_collision_monitor_available"
                    ).value
                ),
                planner_id=str(self.get_parameter("moveit_planner_id").value),
                planning_time_sec=float(
                    self.get_parameter("moveit_planning_time_sec").value
                ),
                velocity_scaling=float(
                    self.get_parameter("moveit_velocity_scaling").value
                ),
                acceleration_scaling=float(
                    self.get_parameter("moveit_acceleration_scaling").value
                ),
            )
            return MoveItBackend(
                self,
                config,
                scene_config=self.scene_config,
                callback_group=self.callback_group,
            )
        raise ValueError("backend must be 'pybullet' or 'moveit'")


def runtime_error_result(
    task: TaskSpec,
    backend=None,
    *,
    failure_type: str = "runtime_error",
    failure_state: str = "runtime_boundary",
) -> EpisodeResult:
    backend_name = str(getattr(backend, "backend_name", task.backend))
    simulator = str(getattr(backend, "simulator_name", backend_name))
    motion = getattr(backend, "last_motion_report", None)
    if motion is None:
        motion = getattr(backend, "reset_motion_report", None)

    object_state_known = bool(
        getattr(backend, "object_state_known", backend_name == "pybullet")
    )
    object_final_position = task.object_position
    object_velocity = None
    try:
        position, velocity = backend.object_state()
        candidate_position = tuple(float(value) for value in position)
        if len(candidate_position) == 3 and all(
            math.isfinite(value) for value in candidate_position
        ):
            object_final_position = candidate_position
        candidate_velocity = tuple(float(value) for value in velocity)
        if len(candidate_velocity) == 3 and all(
            math.isfinite(value) for value in candidate_velocity
        ):
            object_velocity = candidate_velocity
    except (AttributeError, TypeError, ValueError):
        pass

    final_xy_error = (
        math.dist(object_final_position[:2], task.target_position[:2])
        if object_state_known
        else None
    )
    final_z_error = (
        abs(object_final_position[2] - task.target_position[2])
        if object_state_known
        else None
    )
    final_speed = (
        math.sqrt(sum(value * value for value in object_velocity))
        if object_state_known and object_velocity is not None
        else None
    )
    plan = getattr(motion, "plan_result", None)
    trajectory_points = (
        int(motion.trajectory_points)
        if motion is not None and motion.trajectory_points is not None
        else len(getattr(plan, "path", []))
    )
    planning_time = float(plan.planning_time_sec) if plan is not None else 0.0
    if motion is not None and motion.path_length_rad is not None:
        path_length = float(motion.path_length_rad)
    else:
        path = list(getattr(plan, "path", []))
        path_length = sum(
            math.dist(first, second)
            for first, second in zip(path, path[1:])
        )
    collision_count = int(getattr(backend, "collision_count", 0))
    invalid_collision = bool(
        getattr(backend, "invalid_collision_detected", collision_count > 0)
    )
    runtime_metrics = {
        "object_state_known": object_state_known,
        "collision_state_known": bool(
            getattr(backend, "collision_state_known", True)
        ),
        "planning_scene_sync_success": bool(
            getattr(backend, "planning_scene_sync_success", True)
        ),
    }
    if motion is not None:
        runtime_metrics.update(
            {
                "planning_success": bool(motion.planning_success),
                "planning_time_sec": planning_time,
                "execution_time_sec": float(motion.execution_time_sec),
                "path_length_rad": path_length,
                "trajectory_points": trajectory_points,
                "moveit_error_code": int(motion.moveit_error_code),
                "max_joint_tracking_error_rad": (
                    motion.max_joint_tracking_error_rad
                ),
            }
        )
    collision_metrics = getattr(backend, "execution_collision_metrics", {})
    if collision_metrics:
        for key, value in collision_metrics.items():
            output_key = (
                "collision_failure_type" if key == "failure_type" else key
            )
            runtime_metrics[output_key] = value
    physical_metrics = getattr(backend, "physical_grasp_metrics", {})
    if physical_metrics:
        for key, value in physical_metrics.items():
            output_key = (
                "physical_failure_type" if key == "failure_type" else key
            )
            runtime_metrics[output_key] = value

    return EpisodeResult(
        episode_id=task.episode_id,
        task_seed=task.task_seed,
        planner=task.planner,
        planner_seed=task.planner_seed,
        scene=task.scene,
        planning_success=bool(
            motion is not None and motion.planning_success
        ),
        planning_time_sec=planning_time,
        explored_nodes=int(getattr(plan, "explored_nodes", 0)),
        path_waypoints=trajectory_points,
        path_length_rad=path_length,
        execution_sim_steps=int(getattr(backend, "simulation_steps", 0)),
        task_success=False,
        collision_count=collision_count,
        failure_type=failure_type,
        object_initial_position=task.object_position,
        object_final_position=object_final_position,
        target_position=task.target_position,
        backend=backend_name,
        simulator=simulator,
        planner_id=str(getattr(backend, "planner_id", task.planner)),
        grasp_mode=task.grasp_mode,
        execution_time_sec=(
            float(motion.execution_time_sec) if motion is not None else 0.0
        ),
        execution_success=False,
        trajectory_points=trajectory_points,
        max_joint_tracking_error_rad=getattr(
            backend,
            "max_joint_tracking_error_rad",
            None,
        ),
        final_xy_error_m=final_xy_error,
        final_z_error_m=final_z_error,
        final_object_speed_mps=final_speed,
        planning_collision="planning" in failure_type
        and "collision" in failure_type,
        execution_collision=invalid_collision,
        object_attached=bool(
            getattr(backend, "_ever_attached", False)
            or getattr(backend, "_attached", False)
        ),
        object_released=bool(
            getattr(backend, "_ever_released", False)
            or getattr(backend, "_released", False)
        ),
        planning_scene_sync_success=bool(
            getattr(backend, "planning_scene_sync_success", True)
        ),
        failure_state=failure_state,
        stage_metrics={"runtime_boundary": runtime_metrics},
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
