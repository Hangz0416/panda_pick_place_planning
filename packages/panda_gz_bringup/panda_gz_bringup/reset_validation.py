"""Repeat one typed Gazebo reset and audit the observed post-reset state."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from panda_pick_place_interfaces.msg import SceneObjectTask
from panda_pick_place_interfaces.srv import ResetEpisode
from panda_pick_place_planning.core.tasks import generate_task
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from panda_gz_bringup.reset_contract import (
    RESET_VALIDATION_THRESHOLDS,
    ResetValidationMetrics,
    quaternion_angle_error,
    vector_norm,
)


ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
HOME = (0.0, -0.65, 0.0, -2.2, 0.0, 1.55, 0.78)
OPEN_FINGER = 0.04


def _position_error(actual, expected) -> float:
    return vector_norm(tuple(a - b for a, b in zip(actual, expected)))


def workpiece_ids(workpiece_count: int) -> tuple[str, ...]:
    """Return semantic identities for legacy and multi-workpiece resets."""
    if workpiece_count not in {1, 3}:
        raise ValueError("workpiece_count must be one or three")
    if workpiece_count == 1:
        return ("cube",)
    return tuple(f"workpiece_{index}" for index in range(workpiece_count))


class ResetValidationNode(Node):
    """Collect post-reset state without modifying the reset service schema."""

    def __init__(self, workpiece_count: int = 1) -> None:
        super().__init__("panda_reset_validation")
        # Preserve the legacy single-object identity while the multi-object
        # contract uses the deterministic workpiece_0..2 identities.
        self.workpiece_ids = workpiece_ids(workpiece_count)
        self.joints: dict[str, tuple[float, float]] = {}
        self.workpiece_poses = {}
        self.workpiece_linear = {}
        self.workpiece_angular = {}
        self.target_pose = None
        self.obstacle_pose = None
        self.generations = {
            "joints": 0,
            "target": 0,
            "obstacle": 0,
            **{object_id: 0 for object_id in self.workpiece_ids},
        }
        self.create_subscription(JointState, "/joint_states", self._joints, 10)
        for index, object_id in enumerate(self.workpiece_ids):
            suffix = "" if index == 0 else f"_{index}"
            self.create_subscription(
                Odometry,
                f"/panda_gz/cube{suffix}_odometry",
                lambda message, value=object_id: self._workpiece(value, message),
                10,
            )
        self.create_subscription(
            PoseArray, "/panda_gz/target_pose", self._target, 10
        )
        self.create_subscription(
            PoseArray, "/panda_gz/obstacle_pose", self._obstacle, 10
        )
        self.client = self.create_client(
            ResetEpisode, "/panda_gz/reset_episode"
        )

    def _joints(self, message: JointState) -> None:
        velocities = list(message.velocity) + [0.0] * len(message.name)
        self.joints.update(
            {
                name: (float(position), float(velocities[index]))
                for index, (name, position) in enumerate(
                    zip(message.name, message.position)
                )
            }
        )
        self.generations["joints"] += 1

    def _workpiece(self, object_id: str, message: Odometry) -> None:
        self.workpiece_poses[object_id] = message.pose.pose
        self.workpiece_linear[object_id] = message.twist.twist.linear
        self.workpiece_angular[object_id] = message.twist.twist.angular
        self.generations[object_id] += 1

    def _target(self, message: PoseArray) -> None:
        if message.poses:
            self.target_pose = message.poses[0]
            self.generations["target"] += 1

    def _obstacle(self, message: PoseArray) -> None:
        if message.poses:
            self.obstacle_pose = message.poses[0]
            self.generations["obstacle"] += 1

    def wait_for_fresh_state(self, before: dict[str, int], timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self.generations[name] > before[name] for name in before):
                return True
        return False

    def metrics(self, scene: str, workpieces, target_position) -> ResetValidationMetrics:
        required = set(ARM_JOINTS + FINGER_JOINTS)
        if not required <= self.joints.keys():
            raise RuntimeError("joint state is incomplete")
        if any(
            value is None
            for value in (
                self.target_pose,
                self.obstacle_pose,
            )
        ):
            raise RuntimeError("entity state is incomplete")
        expected = {item.object_id: item for item in workpieces}
        if set(expected) != set(self.workpiece_ids):
            raise RuntimeError("workpiece identity mismatch")
        if any(
            object_id not in self.workpiece_poses
            or object_id not in self.workpiece_linear
            or object_id not in self.workpiece_angular
            for object_id in self.workpiece_ids
        ):
            raise RuntimeError("workpiece state is incomplete")
        arm_error = max(
            abs(self.joints[name][0] - expected)
            for name, expected in zip(ARM_JOINTS, HOME)
        )
        arm_velocity = max(abs(self.joints[name][1]) for name in ARM_JOINTS)
        gripper_error = max(
            abs(self.joints[name][0] - OPEN_FINGER) for name in FINGER_JOINTS
        )
        target = self.target_pose.position
        obstacle = self.obstacle_pose.position
        expected_obstacle = (0.54, 0.0 if scene == "obstacle" else 2.0, 0.14)
        position_errors = []
        orientation_errors = []
        linear_speeds = []
        angular_speeds = []
        for object_id in self.workpiece_ids:
            item = expected[object_id]
            pose = self.workpiece_poses[object_id]
            position_errors.append(
                _position_error(
                    (pose.position.x, pose.position.y, pose.position.z),
                    item.position,
                )
            )
            orientation_errors.append(
                quaternion_angle_error(
                    (
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                    item.orientation_xyzw,
                )
            )
            linear = self.workpiece_linear[object_id]
            angular = self.workpiece_angular[object_id]
            linear_speeds.append(vector_norm((linear.x, linear.y, linear.z)))
            angular_speeds.append(vector_norm((angular.x, angular.y, angular.z)))
        return ResetValidationMetrics(
            joint_position_error_rad=arm_error,
            joint_velocity_rad_s=arm_velocity,
            gripper_position_error_m=gripper_error,
            cube_position_error_m=max(position_errors),
            cube_orientation_error_rad=max(orientation_errors),
            target_position_error_m=_position_error(
                (target.x, target.y, target.z),
                (target_position[0], target_position[1], 0.001),
            ),
            obstacle_position_error_m=_position_error(
                (obstacle.x, obstacle.y, obstacle.z), expected_obstacle
            ),
            cube_linear_speed_mps=max(linear_speeds),
            cube_angular_speed_rad_s=max(angular_speeds),
        )


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--scene", choices=("clear", "obstacle"), default="clear")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--object-variant",
        choices=("cube", "rect_prism"),
        default="rect_prism",
    )
    parser.add_argument("--workpiece-count", type=int, choices=(1, 3), default=3)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/runs/gazebo_assisted/reset_validation.json"),
    )
    parsed = parser.parse_args(args)
    if parsed.repetitions <= 0:
        parser.error("--repetitions must be positive")
    if parsed.seed < 0:
        parser.error("--seed must be non-negative")
    if parsed.workpiece_count == 3 and parsed.object_variant != "rect_prism":
        parser.error("three-workpiece validation requires --object-variant rect_prism")
    return parsed


def main(args=None) -> int:
    cli = parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = ResetValidationNode(cli.workpiece_count)
    records = []
    terminal_error = ""
    try:
        if not node.client.wait_for_service(timeout_sec=120.0):
            terminal_error = "reset_service_timeout"
        task = generate_task(
            0,
            cli.seed,
            "rrt_connect",
            cli.scene,
            object_variant=cli.object_variant,
            workpiece_count=cli.workpiece_count,
        )
        for index in range(cli.repetitions if not terminal_error else 0):
            request = ResetEpisode.Request()
            request.header.frame_id = "world"
            request.episode_id = index
            request.task_seed = task.task_seed
            request.scene = cli.scene
            request.grasp_mode = "assisted"
            request.object_variant = task.object_variant
            request.object_yaw_rad = task.object_yaw_rad
            (
                request.object_position.x,
                request.object_position.y,
                request.object_position.z,
            ) = task.object_position
            (
                request.target_position.x,
                request.target_position.y,
                request.target_position.z,
            ) = task.target_position
            for item in task.workpieces:
                message = SceneObjectTask()
                message.object_id = item.object_id
                message.object_variant = item.object_variant
                (
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ) = item.position
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ) = item.orientation_xyzw
                request.workpieces.append(message)
            before = dict(node.generations)
            future = node.client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=45.0)
            response = future.result() if future.done() else None
            record = {"iteration": index, "service_success": False, "passed": False}
            if response is None:
                record["failure_type"] = "reset_call_timeout"
            elif not response.success:
                record["failure_type"] = response.failure_type or "reset_failed"
                record["message"] = response.message
            elif not node.wait_for_fresh_state(before, 5.0):
                record["failure_type"] = "post_reset_state_timeout"
            else:
                metrics = node.metrics(
                    cli.scene,
                    task.workpieces,
                    task.target_position,
                )
                record.update(asdict(metrics))
                record["service_success"] = True
                record["passed"] = metrics.passed()
                if not record["passed"]:
                    record["failure_type"] = "reset_threshold_exceeded"
            records.append(record)
            if not record["passed"]:
                break
    except Exception as exc:  # CLI boundary persists a classified diagnostic.
        terminal_error = f"reset_validation_error:{type(exc).__name__}:{exc}"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    maxima = {
        field: max(
            (float(record[field]) for record in records if field in record),
            default=None,
        )
        for field in RESET_VALIDATION_THRESHOLDS
    }
    passed = (
        not terminal_error
        and len(records) == cli.repetitions
        and all(record["passed"] for record in records)
    )
    document = {
        "schema_version": 1,
        "scene": cli.scene,
        "task_seed": cli.seed,
        "object_variant": task.object_variant,
        "workpiece_count": task.workpiece_count,
        "object_yaw_rad": task.object_yaw_rad,
        "repetitions_requested": cli.repetitions,
        "repetitions_completed": len(records),
        "passed": passed,
        "terminal_error": terminal_error,
        "thresholds": RESET_VALIDATION_THRESHOLDS,
        "maxima": maxima,
        "iterations": records,
    }
    cli.output.parent.mkdir(parents=True, exist_ok=True)
    cli.output.write_text(
        json.dumps(document, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(document, indent=2, ensure_ascii=False))
    return 0 if passed else 1
