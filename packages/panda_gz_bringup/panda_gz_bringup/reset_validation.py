"""Repeat one typed Gazebo reset and audit the observed post-reset state."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from panda_pick_place_interfaces.srv import ResetEpisode
from panda_pick_place_planning.core.tasks import generate_task
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from panda_gz_bringup.reset_contract import (
    RESET_VALIDATION_THRESHOLDS,
    ResetValidationMetrics,
    vector_norm,
)


ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")
HOME = (0.0, -0.65, 0.0, -2.2, 0.0, 1.55, 0.78)
OPEN_FINGER = 0.04


def _position_error(actual, expected) -> float:
    return vector_norm(tuple(a - b for a, b in zip(actual, expected)))


class ResetValidationNode(Node):
    """Collect post-reset state without modifying the reset service schema."""

    def __init__(self) -> None:
        super().__init__("panda_reset_validation")
        self.joints: dict[str, tuple[float, float]] = {}
        self.cube_pose = None
        self.cube_linear = None
        self.cube_angular = None
        self.target_pose = None
        self.obstacle_pose = None
        self.generations = {"joints": 0, "cube": 0, "target": 0, "obstacle": 0}
        self.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.create_subscription(
            Odometry, "/panda_gz/cube_odometry", self._cube, 10
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

    def _cube(self, message: Odometry) -> None:
        self.cube_pose = message.pose.pose
        self.cube_linear = message.twist.twist.linear
        self.cube_angular = message.twist.twist.angular
        self.generations["cube"] += 1

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

    def metrics(
        self, scene: str, object_position, target_position
    ) -> ResetValidationMetrics:
        required = set(ARM_JOINTS + FINGER_JOINTS)
        if not required <= self.joints.keys():
            raise RuntimeError("joint state is incomplete")
        if any(
            value is None
            for value in (
                self.cube_pose,
                self.cube_linear,
                self.cube_angular,
                self.target_pose,
                self.obstacle_pose,
            )
        ):
            raise RuntimeError("entity state is incomplete")
        arm_error = max(
            abs(self.joints[name][0] - expected)
            for name, expected in zip(ARM_JOINTS, HOME)
        )
        arm_velocity = max(abs(self.joints[name][1]) for name in ARM_JOINTS)
        gripper_error = max(
            abs(self.joints[name][0] - OPEN_FINGER) for name in FINGER_JOINTS
        )
        cube = self.cube_pose.position
        target = self.target_pose.position
        obstacle = self.obstacle_pose.position
        expected_obstacle = (0.54, 0.0 if scene == "obstacle" else 2.0, 0.14)
        return ResetValidationMetrics(
            joint_position_error_rad=arm_error,
            joint_velocity_rad_s=arm_velocity,
            gripper_position_error_m=gripper_error,
            cube_position_error_m=_position_error(
                (cube.x, cube.y, cube.z), object_position
            ),
            target_position_error_m=_position_error(
                (target.x, target.y, target.z),
                (target_position[0], target_position[1], 0.001),
            ),
            obstacle_position_error_m=_position_error(
                (obstacle.x, obstacle.y, obstacle.z), expected_obstacle
            ),
            cube_linear_speed_mps=vector_norm(
                (self.cube_linear.x, self.cube_linear.y, self.cube_linear.z)
            ),
            cube_angular_speed_rad_s=vector_norm(
                (self.cube_angular.x, self.cube_angular.y, self.cube_angular.z)
            ),
        )


def parse_args(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--scene", choices=("clear", "obstacle"), default="clear")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/v2/gazebo_assisted_final/reset_validation.json"),
    )
    parsed = parser.parse_args(args)
    if parsed.repetitions <= 0:
        parser.error("--repetitions must be positive")
    if parsed.seed < 0:
        parser.error("--seed must be non-negative")
    return parsed


def main(args=None) -> int:
    cli = parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args)
    node = ResetValidationNode()
    records = []
    terminal_error = ""
    try:
        if not node.client.wait_for_service(timeout_sec=120.0):
            terminal_error = "reset_service_timeout"
        task = generate_task(0, cli.seed, "rrt_connect", cli.scene)
        for index in range(cli.repetitions if not terminal_error else 0):
            request = ResetEpisode.Request()
            request.header.frame_id = "world"
            request.episode_id = index
            request.task_seed = task.task_seed
            request.scene = cli.scene
            request.grasp_mode = "assisted"
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
                    cli.scene, task.object_position, task.target_position
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
