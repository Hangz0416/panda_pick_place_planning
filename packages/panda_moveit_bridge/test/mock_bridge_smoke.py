#!/usr/bin/env python3
"""Smoke a separately launched panda_moveit_config mock stack."""

from __future__ import annotations

import argparse
import json
from typing import Iterable

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionMatrix,
    CollisionObject,
    MoveItErrorCodes,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene
from panda_pick_place_interfaces.action import PlanExecuteMotion
from panda_pick_place_interfaces.srv import (
    RemoveWorldObject,
    SetAttachedObject,
    UpsertWorldObject,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


REQUIRED_CONTROLLERS = {
    "joint_state_broadcaster",
    "panda_arm_controller",
    "panda_hand_controller",
}

PLANNER_ID = "RRTConnectkConfigDefault"
POSE_END_EFFECTOR_LINK = "panda_grasptarget"
DOWNWARD_ORIENTATION = (0.9238795325, -0.3826834324, 0.0, 0.0)
INVALID_NAMED_TARGET = "__panda_mock_smoke_invalid_target__"
CUBE_TOUCH_LINKS = ("panda_hand", "panda_leftfinger", "panda_rightfinger")
TABLE_DIMENSIONS = (1.5, 1.0, 0.05)
TABLE_POSITION = (0.5, 0.0, -0.025)
PLANNING_ONLY_TARGETS = {
    "pre_grasp": (0.59, 0.22, 0.24),
    "grasp": (0.59, 0.22, 0.075),
    "transfer_hover": (0.59, -0.225, 0.24),
    "place": (0.59, -0.225, 0.09),
}


def wait(node: Node, future, timeout_sec: float, description: str):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"timed out waiting for {description}")
    return future.result()


def box(object_id: str, dimensions: Iterable[float], xyz: Iterable[float]):
    collision_object = CollisionObject()
    collision_object.header.frame_id = "world"
    collision_object.id = object_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in dimensions]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(value) for value in xyz
    )
    pose.orientation.w = 1.0
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [pose]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def acm_pair_allowed(
    matrix: AllowedCollisionMatrix, first: str, second: str
) -> bool:
    """Return true only when both explicit ACM directions allow the pair."""

    indices = {name: index for index, name in enumerate(matrix.entry_names)}
    if first not in indices or second not in indices:
        return False
    first_index = indices[first]
    second_index = indices[second]
    pairs = ((first_index, second_index), (second_index, first_index))
    return all(
        row < len(matrix.entry_values)
        and column < len(matrix.entry_values[row].enabled)
        and matrix.entry_values[row].enabled[column]
        for row, column in pairs
    )


def pose_target(
    xyz: Iterable[float], quaternion_xyzw: Iterable[float]
) -> PoseStamped:
    target = PoseStamped()
    target.header.frame_id = "world"
    target.pose.position.x, target.pose.position.y, target.pose.position.z = (
        float(value) for value in xyz
    )
    (
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w,
    ) = (float(value) for value in quaternion_xyzw)
    return target


class MockBridgeSmoke:
    def __init__(self, timeout_sec: float) -> None:
        self.timeout = timeout_sec
        self.node = rclpy.create_node("panda_moveit_mock_smoke")
        self.controllers = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.upsert = self.node.create_client(
            UpsertWorldObject, "/upsert_world_object"
        )
        self.remove = self.node.create_client(
            RemoveWorldObject, "/remove_world_object"
        )
        self.attach = self.node.create_client(
            SetAttachedObject, "/set_attached_object"
        )
        self.get_scene = self.node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.motion = ActionClient(
            self.node, PlanExecuteMotion, "/plan_execute_motion"
        )

    def wait_for_servers(self) -> None:
        clients = {
            "controller list": self.controllers,
            "world-object upsert": self.upsert,
            "world-object removal": self.remove,
            "attach/detach": self.attach,
            "planning-scene query": self.get_scene,
        }
        for description, client in clients.items():
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise RuntimeError(f"{description} service unavailable")
        if not self.motion.wait_for_server(timeout_sec=self.timeout):
            raise RuntimeError("PlanExecuteMotion action unavailable")

    def active_controllers(self) -> dict[str, str]:
        response = wait(
            self.node,
            self.controllers.call_async(ListControllers.Request()),
            self.timeout,
            "controller list",
        )
        states = {entry.name: entry.state for entry in response.controller}
        if not REQUIRED_CONTROLLERS.issubset(states):
            raise RuntimeError(f"required controller missing: {states}")
        inactive = {
            name: states[name]
            for name in REQUIRED_CONTROLLERS
            if states[name] != "active"
        }
        if inactive:
            raise RuntimeError(f"controller is not active: {inactive}")
        return states

    def upsert_box(
        self,
        object_id: str,
        dimensions: Iterable[float],
        xyz: Iterable[float],
        allowed_links: Iterable[str] = (),
    ) -> None:
        request = UpsertWorldObject.Request()
        request.collision_object = box(object_id, dimensions, xyz)
        request.allowed_collision_links = list(allowed_links)
        response = wait(
            self.node,
            self.upsert.call_async(request),
            self.timeout,
            f"upsert {object_id}",
        )
        if not response.success:
            raise RuntimeError(f"upsert {object_id} failed: {response.message}")

    def remove_object(self, object_id: str) -> None:
        request = RemoveWorldObject.Request()
        request.object_id = object_id
        response = wait(
            self.node,
            self.remove.call_async(request),
            self.timeout,
            f"remove {object_id}",
        )
        if not response.success:
            raise RuntimeError(f"remove {object_id} failed: {response.message}")

    def set_attached(self, attach: bool) -> None:
        request = SetAttachedObject.Request()
        request.object_id = "cube"
        request.link_name = "panda_hand"
        request.touch_links = list(CUBE_TOUCH_LINKS)
        request.attach = attach
        response = wait(
            self.node,
            self.attach.call_async(request),
            self.timeout,
            "attach object" if attach else "detach object",
        )
        if not response.success:
            raise RuntimeError(f"attach={attach} failed: {response.message}")

    def scene_state(
        self,
    ) -> tuple[set[str], set[str], AllowedCollisionMatrix]:
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        response = wait(
            self.node,
            self.get_scene.call_async(request),
            self.timeout,
            "planning scene",
        )
        world = {item.id for item in response.scene.world.collision_objects}
        attached = {
            item.object.id
            for item in response.scene.robot_state.attached_collision_objects
        }
        return world, attached, response.scene.allowed_collision_matrix

    @staticmethod
    def _motion_goal() -> PlanExecuteMotion.Goal:
        goal = PlanExecuteMotion.Goal()
        goal.planner_id = PLANNER_ID
        goal.allowed_planning_time_sec = 3.0
        goal.max_velocity_scaling_factor = 0.2
        goal.max_acceleration_scaling_factor = 0.2
        goal.execute = False
        return goal

    def _send_motion(
        self, goal: PlanExecuteMotion.Goal, description: str
    ) -> PlanExecuteMotion.Result:
        goal_handle = wait(
            self.node,
            self.motion.send_goal_async(goal),
            self.timeout,
            f"{description} goal acceptance",
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"{description} goal rejected by action server")
        wrapped = wait(
            self.node,
            goal_handle.get_result_async(),
            self.timeout + max(3.0, goal.allowed_planning_time_sec),
            description,
        )
        return wrapped.result

    @staticmethod
    def _motion_metrics(result: PlanExecuteMotion.Result) -> dict[str, object]:
        return {
            "moveit_error_code": result.moveit_error_code,
            "planning_time_sec": result.planning_time_sec,
            "path_length_rad": result.path_length_rad,
            "trajectory_points": result.trajectory_points,
        }

    def plan_home(self) -> dict[str, object]:
        goal = self._motion_goal()
        goal.target_type = PlanExecuteMotion.Goal.TARGET_TYPE_NAMED
        goal.named_target = "home"
        result = self._send_motion(goal, "home plan")
        if not result.success:
            raise RuntimeError(
                f"home plan failed: {result.failure_type}/{result.moveit_error_code}"
            )
        return self._motion_metrics(result)

    def plan_pose_from_home(
        self, stage: str, position: Iterable[float]
    ) -> dict[str, object]:
        """Plan one nonzero motion without changing mock-controller state."""

        goal = self._motion_goal()
        goal.target_type = PlanExecuteMotion.Goal.TARGET_TYPE_POSE
        goal.pose_target = pose_target(position, DOWNWARD_ORIENTATION)
        goal.end_effector_link = POSE_END_EFFECTOR_LINK
        result = self._send_motion(goal, f"{stage} planning-only motion")
        if not result.success:
            raise RuntimeError(
                f"{stage} plan failed: "
                f"{result.failure_type}/{result.moveit_error_code}"
            )
        if result.trajectory_points < 2 or result.path_length_rad <= 1.0e-6:
            raise RuntimeError(
                f"{stage} target did not produce a nonzero path: "
                f"points={result.trajectory_points}, "
                f"length={result.path_length_rad}"
            )
        metrics = self._motion_metrics(result)
        metrics["target_position_m"] = [float(value) for value in position]
        return metrics

    def reject_invalid_named_target(self) -> dict[str, object]:
        goal = self._motion_goal()
        goal.target_type = PlanExecuteMotion.Goal.TARGET_TYPE_NAMED
        goal.named_target = INVALID_NAMED_TARGET
        result = self._send_motion(goal, "invalid named target")
        expected_code = MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS
        if result.success:
            raise RuntimeError("invalid named target unexpectedly succeeded")
        if result.failure_type != "target_rejected":
            raise RuntimeError(
                "invalid named target was not classified as target_rejected: "
                f"{result.failure_type}/{result.moveit_error_code}"
            )
        if result.moveit_error_code != expected_code:
            raise RuntimeError(
                "invalid named target returned the wrong MoveIt error code: "
                f"expected={expected_code}, actual={result.moveit_error_code}"
            )
        return {
            "moveit_error_code": result.moveit_error_code,
            "failure_type": result.failure_type,
        }

    def close(self) -> None:
        self.node.destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", choices=("clear", "obstacle"), default="clear")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()
    smoke = MockBridgeSmoke(args.timeout)
    added = []
    cube_attached = False
    try:
        smoke.wait_for_servers()
        controllers = smoke.active_controllers()
        smoke.upsert_box(
            "table", TABLE_DIMENSIONS, TABLE_POSITION, ("panda_link0",)
        )
        added.append("table")
        smoke.upsert_box(
            "cube",
            (0.05, 0.05, 0.05),
            (0.59, 0.22, 0.025),
            CUBE_TOUCH_LINKS,
        )
        added.append("cube")
        if args.scene == "obstacle":
            smoke.upsert_box("obstacle", (0.11, 0.10, 0.28), (0.54, 0.0, 0.14))
            added.append("obstacle")
        world_before, attached_before, acm_before = smoke.scene_state()
        if not set(added).issubset(world_before) or attached_before:
            raise RuntimeError(
                f"unexpected scene: world={world_before}, attached={attached_before}"
            )
        if not acm_pair_allowed(acm_before, "table", "panda_link0"):
            raise RuntimeError(
                "table <-> panda_link0 is not explicitly allowed in the ACM"
            )
        missing_cube_touch_pairs = [
            link
            for link in CUBE_TOUCH_LINKS
            if not acm_pair_allowed(acm_before, "cube", link)
        ]
        if missing_cube_touch_pairs:
            raise RuntimeError(
                "cube touch-link ACM entries are missing for "
                f"{missing_cube_touch_pairs}"
            )

        home = smoke.plan_home()
        independent_pose_plans = {
            stage: smoke.plan_pose_from_home(stage, position)
            for stage, position in PLANNING_ONLY_TARGETS.items()
        }
        invalid_target = smoke.reject_invalid_named_target()
        smoke.set_attached(True)
        cube_attached = True
        world_attached, attached, acm_attached = smoke.scene_state()
        if "cube" not in attached or "cube" in world_attached:
            raise RuntimeError(
                f"attach not synchronized: world={world_attached}, attached={attached}"
            )
        if "cube" in acm_attached.entry_names:
            raise RuntimeError(
                "world-object ACM entries for cube remain after attachment"
            )
        smoke.set_attached(False)
        cube_attached = False
        world_detached, attached_detached, _ = smoke.scene_state()
        if "cube" not in world_detached or "cube" in attached_detached:
            raise RuntimeError(
                f"detach not synchronized: world={world_detached}, attached={attached_detached}"
            )
        print(
            json.dumps(
                {
                    "scene": args.scene,
                    "controllers": controllers,
                    "world_objects": sorted(world_before),
                    "table_base_acm_allowed": True,
                    "cube_touch_acm_allowed": list(CUBE_TOUCH_LINKS),
                    "cube_world_acm_removed_after_attach": True,
                    "attach_detach_synchronized": True,
                    "home": home,
                    "independent_planning_only_from_home": independent_pose_plans,
                    "invalid_named_target": invalid_target,
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        )
    finally:
        try:
            if cube_attached:
                smoke.set_attached(False)
        except RuntimeError:
            pass
        for object_id in reversed(added):
            try:
                smoke.remove_object(object_id)
            except RuntimeError:
                pass
        smoke.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
