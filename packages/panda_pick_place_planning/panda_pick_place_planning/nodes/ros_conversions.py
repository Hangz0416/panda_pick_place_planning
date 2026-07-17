from __future__ import annotations

import math
from typing import Any

from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec


def _point_values(point: Any) -> tuple[float, float, float]:
    return (float(point.x), float(point.y), float(point.z))


def _set_point(point: Any, values: tuple[float, float, float]) -> None:
    point.x, point.y, point.z = (float(value) for value in values)


def task_to_ros(task: TaskSpec):
    from panda_pick_place_interfaces.msg import ManipulationTask

    message = ManipulationTask()
    message.header.frame_id = "world"
    message.episode_id = task.episode_id
    message.task_seed = task.task_seed
    message.backend = task.backend
    message.simulator = task.simulator or (
        "pybullet" if task.backend == "pybullet" else "moveit_mock"
    )
    message.planner_id = task.planner
    message.planner_seed = task.planner_seed
    message.scene = task.scene
    message.grasp_mode = task.grasp_mode
    _set_point(message.object_position, task.object_position)
    _set_point(message.target_position, task.target_position)
    return message


def task_from_ros(message: Any) -> TaskSpec:
    frame_id = str(message.header.frame_id)
    if frame_id not in ("", "world"):
        raise ValueError(
            f"ManipulationTask frame_id must be empty or 'world', got {frame_id!r}"
        )
    planner_id = str(message.planner_id)
    planner = (
        "rrt_connect"
        if planner_id.startswith(("ompl_", "RRTConnect"))
        else planner_id
    )
    return TaskSpec(
        episode_id=int(message.episode_id),
        task_seed=int(message.task_seed),
        planner=planner,
        planner_seed=int(message.planner_seed),
        scene=str(message.scene),
        object_position=_point_values(message.object_position),
        target_position=_point_values(message.target_position),
        backend=str(message.backend or "pybullet"),
        grasp_mode=str(message.grasp_mode or "assisted"),
        simulator=str(message.simulator),
    )


def episode_to_ros(result: EpisodeResult):
    from panda_pick_place_interfaces.msg import EpisodeResult as EpisodeResultMsg

    message = EpisodeResultMsg()
    message.header.frame_id = "world"
    message.episode_id = result.episode_id
    message.task_seed = result.task_seed
    message.backend = result.backend
    message.simulator = result.simulator
    message.planner_id = result.planner_id or result.planner
    message.planner_seed = result.planner_seed
    message.scene = result.scene
    message.grasp_mode = result.grasp_mode
    message.pose_source = result.pose_source
    message.planning_success = result.planning_success
    message.execution_success = result.execution_success
    message.task_success = result.task_success
    message.attached = result.object_attached
    message.released = result.object_released
    message.planning_collision = result.planning_collision
    message.execution_collision = result.execution_collision
    message.planning_time_sec = result.planning_time_sec
    message.execution_time_sec = result.execution_time_sec
    message.explored_nodes = result.explored_nodes
    message.path_waypoints = result.path_waypoints
    message.path_length_rad = result.path_length_rad
    message.trajectory_points = result.trajectory_points
    message.execution_sim_steps = result.execution_sim_steps
    message.max_joint_tracking_error_rad = _optional_float_to_ros(
        result.max_joint_tracking_error_rad
    )
    message.final_xy_error_m = _optional_float_to_ros(result.final_xy_error_m)
    message.final_z_error_m = _optional_float_to_ros(result.final_z_error_m)
    message.final_object_speed_mps = _optional_float_to_ros(
        result.final_object_speed_mps
    )
    message.perception_success = result.perception_success
    message.perception_latency_sec = _optional_float_to_ros(
        result.perception_latency_sec
    )
    message.object_position_error_m = _optional_float_to_ros(
        result.object_position_error_m
    )
    message.target_position_error_m = _optional_float_to_ros(
        result.target_position_error_m
    )
    message.obstacle_position_error_m = _optional_float_to_ros(
        result.obstacle_position_error_m
    )
    message.minimum_perception_confidence = _optional_float_to_ros(
        result.minimum_perception_confidence
    )
    message.collision_count = result.collision_count
    message.failed_state = result.failure_state
    message.failure_type = result.failure_type
    message.sync_error = "" if result.planning_scene_sync_success else "scene_sync_failed"
    _set_point(message.object_initial_position, result.object_initial_position)
    _set_point(message.object_final_position, result.object_final_position)
    _set_point(message.target_position, result.target_position)
    message.physical_left_finger_contact = _optional_bool_to_ros(
        result.physical_left_finger_contact
    )
    message.physical_right_finger_contact = _optional_bool_to_ros(
        result.physical_right_finger_contact
    )
    message.physical_bilateral_contact = _optional_bool_to_ros(
        result.physical_bilateral_contact
    )
    message.physical_slip = _optional_bool_to_ros(result.physical_slip)
    message.physical_translation_drift_m = _optional_float_to_ros(
        result.physical_translation_drift_m
    )
    message.physical_rotation_drift_rad = _optional_float_to_ros(
        result.physical_rotation_drift_rad
    )
    message.physical_release_stable_time_sec = _optional_float_to_ros(
        result.physical_release_stable_time_sec
    )
    message.last_collision_pair = list(result.last_collision_pair)
    for state_name, metrics in result.stage_metrics.items():
        message.stage_metrics.append(_stage_metric_to_ros(state_name, metrics))
    return message


def episode_from_ros(message: Any) -> EpisodeResult:
    stage_metrics: dict[str, dict[str, Any]] = {}
    for stage_message in message.stage_metrics:
        state_name, metrics = _stage_metric_from_ros(stage_message)
        if not state_name:
            raise ValueError("stage metric name must not be empty")
        if state_name in stage_metrics:
            raise ValueError(f"duplicate stage metric name: {state_name}")
        stage_metrics[state_name] = metrics
    planner_id = str(message.planner_id)
    planner = (
        "rrt_connect"
        if planner_id.startswith(("ompl_", "RRTConnect"))
        else planner_id
    )
    return EpisodeResult(
        episode_id=int(message.episode_id),
        task_seed=int(message.task_seed),
        planner=planner,
        planner_seed=int(message.planner_seed),
        scene=str(message.scene),
        planning_success=bool(message.planning_success),
        planning_time_sec=float(message.planning_time_sec),
        explored_nodes=int(message.explored_nodes),
        path_waypoints=int(message.path_waypoints),
        path_length_rad=float(message.path_length_rad),
        execution_sim_steps=int(message.execution_sim_steps),
        task_success=bool(message.task_success),
        collision_count=int(message.collision_count),
        failure_type=str(message.failure_type),
        object_initial_position=_point_values(message.object_initial_position),
        object_final_position=_point_values(message.object_final_position),
        target_position=_point_values(message.target_position),
        backend=str(message.backend),
        simulator=str(message.simulator),
        planner_id=planner_id,
        grasp_mode=str(message.grasp_mode),
        pose_source=str(message.pose_source or "ground_truth"),
        perception_success=bool(message.perception_success),
        perception_latency_sec=_optional_float_from_ros(
            message.perception_latency_sec
        ),
        object_position_error_m=_optional_float_from_ros(
            message.object_position_error_m
        ),
        target_position_error_m=_optional_float_from_ros(
            message.target_position_error_m
        ),
        obstacle_position_error_m=_optional_float_from_ros(
            message.obstacle_position_error_m
        ),
        minimum_perception_confidence=_optional_float_from_ros(
            message.minimum_perception_confidence
        ),
        execution_time_sec=float(message.execution_time_sec),
        execution_success=bool(message.execution_success),
        trajectory_points=int(message.trajectory_points),
        max_joint_tracking_error_rad=_optional_float_from_ros(
            message.max_joint_tracking_error_rad
        ),
        final_xy_error_m=_optional_float_from_ros(message.final_xy_error_m),
        final_z_error_m=_optional_float_from_ros(message.final_z_error_m),
        final_object_speed_mps=_optional_float_from_ros(
            message.final_object_speed_mps
        ),
        planning_collision=bool(message.planning_collision),
        execution_collision=bool(message.execution_collision),
        object_attached=bool(message.attached),
        object_released=bool(message.released),
        planning_scene_sync_success=not bool(message.sync_error),
        failure_state=str(message.failed_state),
        stage_metrics=stage_metrics,
        physical_left_finger_contact=_optional_bool_from_ros(
            message.physical_left_finger_contact
        ),
        physical_right_finger_contact=_optional_bool_from_ros(
            message.physical_right_finger_contact
        ),
        physical_bilateral_contact=_optional_bool_from_ros(
            message.physical_bilateral_contact
        ),
        physical_slip=_optional_bool_from_ros(message.physical_slip),
        physical_translation_drift_m=_optional_float_from_ros(
            message.physical_translation_drift_m
        ),
        physical_rotation_drift_rad=_optional_float_from_ros(
            message.physical_rotation_drift_rad
        ),
        physical_release_stable_time_sec=_optional_float_from_ros(
            message.physical_release_stable_time_sec
        ),
        last_collision_pair=tuple(str(value) for value in message.last_collision_pair),
    )


def _stage_metric_to_ros(state_name: str, metrics: dict[str, Any]):
    from panda_pick_place_interfaces.msg import StageMetric

    message = StageMetric()
    message.name = str(state_name)
    message.evaluation_mode = str(metrics.get("evaluation_mode", ""))
    message.planning_success_known = _metric_is_known(metrics, "planning_success")
    message.planning_success = bool(metrics.get("planning_success", False))
    message.execution_success_known = _metric_is_known(metrics, "execution_success")
    message.execution_success = bool(metrics.get("execution_success", False))
    message.planning_time_sec = _optional_float_to_ros(
        _metric_float(metrics, "planning_time_sec")
    )
    message.execution_time_sec = _optional_float_to_ros(
        _metric_float(metrics, "execution_time_sec")
    )
    message.path_length_rad = _optional_float_to_ros(
        _metric_float(metrics, "path_length_rad")
    )
    message.path_waypoints_known = _metric_is_known(metrics, "path_waypoints")
    message.path_waypoints = int(metrics.get("path_waypoints", 0))
    message.trajectory_points_known = _metric_is_known(
        metrics, "trajectory_points"
    )
    message.trajectory_points = int(metrics.get("trajectory_points", 0))
    message.failure_type = str(metrics.get("failure_type", ""))
    message.moveit_error_code_known = _metric_is_known(
        metrics, "moveit_error_code"
    )
    message.moveit_error_code = int(metrics.get("moveit_error_code", 0))
    message.max_joint_tracking_error_rad = _optional_float_to_ros(
        _metric_float(metrics, "max_joint_tracking_error_rad")
    )
    return message


def _stage_metric_from_ros(message: Any) -> tuple[str, dict[str, Any]]:
    metrics: dict[str, Any] = {}
    if message.evaluation_mode:
        metrics["evaluation_mode"] = str(message.evaluation_mode)
    if message.planning_success_known:
        metrics["planning_success"] = bool(message.planning_success)
    if message.execution_success_known:
        metrics["execution_success"] = bool(message.execution_success)
    optional_floats = (
        ("planning_time_sec", message.planning_time_sec),
        ("execution_time_sec", message.execution_time_sec),
        ("path_length_rad", message.path_length_rad),
        ("max_joint_tracking_error_rad", message.max_joint_tracking_error_rad),
    )
    for name, value in optional_floats:
        restored = _optional_float_from_ros(value)
        if restored is not None:
            metrics[name] = restored
    if message.path_waypoints_known:
        metrics["path_waypoints"] = int(message.path_waypoints)
    if message.trajectory_points_known:
        metrics["trajectory_points"] = int(message.trajectory_points)
    if message.failure_type:
        metrics["failure_type"] = str(message.failure_type)
    if message.moveit_error_code_known:
        metrics["moveit_error_code"] = int(message.moveit_error_code)
    return str(message.name), metrics


def _metric_is_known(metrics: dict[str, Any], name: str) -> bool:
    return name in metrics and metrics[name] is not None


def _metric_float(metrics: dict[str, Any], name: str) -> float | None:
    if not _metric_is_known(metrics, name):
        return None
    return float(metrics[name])


def _optional_float_to_ros(value: float | None) -> float:
    return math.nan if value is None else float(value)


def _optional_float_from_ros(value: Any) -> float | None:
    parsed = float(value)
    return None if math.isnan(parsed) else parsed


def _optional_bool_to_ros(value: bool | None) -> int:
    if value is None:
        return -1
    return 1 if value else 0


def _optional_bool_from_ros(value: Any) -> bool | None:
    parsed = int(value)
    if parsed == -1:
        return None
    if parsed == 0:
        return False
    if parsed == 1:
        return True
    raise ValueError(f"invalid optional boolean code: {parsed}")
