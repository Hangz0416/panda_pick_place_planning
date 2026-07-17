from __future__ import annotations

from dataclasses import dataclass, fields
import math
from pathlib import Path
import time
from typing import Any, Mapping, Sequence

from panda_pick_place_planning.core.collision_checker import PandaCollisionChecker
from panda_pick_place_planning.core.kinematics import PandaIKSolver
from panda_pick_place_planning.core.planners.base import interpolate_segment
from panda_pick_place_planning.core.schemas import TaskSpec


ARM_JOINT_INDICES = tuple(range(7))
FINGER_JOINT_INDICES = (9, 10)
END_EFFECTOR_LINK = 11


@dataclass(frozen=True)
class SceneConfig:
    home_joints: tuple[float, ...] = (0.0, -0.65, 0.0, -2.2, 0.0, 1.55, 0.78)
    table_size: tuple[float, float, float] = (1.5, 1.0, 0.05)
    table_position: tuple[float, float, float] = (0.5, 0.0, -0.025)
    cube_half_extent: float = 0.025
    cube_mass: float = 0.05
    target_radius: float = 0.09
    hover_height: float = 0.24
    grasp_height: float = 0.075
    place_height: float = 0.09
    obstacle_position: tuple[float, float, float] = (0.54, 0.0, 0.14)
    obstacle_half_extents: tuple[float, float, float] = (0.055, 0.05, 0.14)
    simulation_hz: int = 240
    motor_steps_per_waypoint: int = 6
    settle_steps: int = 240
    open_gripper_width: float = 0.04
    closed_gripper_width: float = 0.0
    obstacle_clearance_margin: float = 0.01
    carried_object_clearance_margin: float = 0.01
    carried_table_penetration_tolerance: float = 0.002
    success_xy_tolerance_m: float = 0.04
    success_z_tolerance_m: float = 0.015
    success_max_linear_velocity_mps: float = 0.05

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "SceneConfig":
        """Create a config from YAML-compatible values while rejecting typos."""

        field_names = {item.name for item in fields(cls)}
        unknown = set(values) - field_names
        if unknown:
            raise ValueError(f"unknown SceneConfig fields: {sorted(unknown)}")
        tuple_fields = {
            "home_joints",
            "table_size",
            "table_position",
            "obstacle_position",
            "obstacle_half_extents",
        }
        converted = {
            key: tuple(value) if key in tuple_fields else value
            for key, value in values.items()
        }
        return cls(**converted)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "SceneConfig":
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("pyyaml_not_installed") from exc
        with Path(path).open(encoding="utf-8") as config_file:
            values = yaml.safe_load(config_file) or {}
        if not isinstance(values, dict):
            raise ValueError("SceneConfig YAML root must be a mapping")
        return cls.from_mapping(values)


@dataclass(frozen=True)
class ExecutionReport:
    success: bool
    simulation_steps: int
    collision_count: int
    failure_type: str = ""


class PyBulletPandaEnv:
    def __init__(self, use_gui: bool = False, config: SceneConfig | None = None) -> None:
        self.use_gui = use_gui
        self.config = config or SceneConfig()
        self.bullet = None
        self.physics_client_id: int | None = None
        self.robot_id: int | None = None
        self.plane_id: int | None = None
        self.table_id: int | None = None
        self.cube_id: int | None = None
        self.obstacle_id: int | None = None
        self.target_marker_id: int | None = None
        self.joint_limits: list[tuple[float, float]] = []
        self.ik_solver: PandaIKSolver | None = None
        self.collision_checker: PandaCollisionChecker | None = None
        self.grasp_constraint_id: int | None = None
        self.simulation_steps = 0
        self.collision_count = 0
        self.carried_object_collision_count = 0

    def connect(self) -> None:
        if self.physics_client_id is not None:
            return
        try:
            import pybullet as p
            import pybullet_data
        except ImportError as exc:
            raise RuntimeError("pybullet_not_installed") from exc

        self.bullet = p
        mode = p.GUI if self.use_gui else p.DIRECT
        self.physics_client_id = p.connect(mode)
        p.resetSimulation(physicsClientId=self.physics_client_id)
        p.setAdditionalSearchPath(
            pybullet_data.getDataPath(),
            physicsClientId=self.physics_client_id,
        )
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.physics_client_id)
        p.setTimeStep(1.0 / self.config.simulation_hz, physicsClientId=self.physics_client_id)
        self._load_base_scene()

    def close(self) -> None:
        if self.physics_client_id is None or self.bullet is None:
            return
        self.bullet.disconnect(self.physics_client_id)
        self.physics_client_id = None

    def reset(self, task: TaskSpec) -> None:
        self.connect()
        self._require_connected()
        self._remove_grasp_constraint()
        self.simulation_steps = 0
        self.collision_count = 0
        self.carried_object_collision_count = 0
        self._reset_arm(self.config.home_joints)
        self._reset_gripper(self.config.open_gripper_width)
        self.bullet.resetBasePositionAndOrientation(
            self.cube_id,
            task.object_position,
            (0.0, 0.0, 0.0, 1.0),
            physicsClientId=self.physics_client_id,
        )
        self.bullet.resetBaseVelocity(
            self.cube_id,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            physicsClientId=self.physics_client_id,
        )
        self._replace_target_marker(task.target_position)
        self._replace_obstacle(task.scene == "obstacle")
        self._refresh_helpers()
        self.step(self.config.simulation_hz // 2, check_collisions=False)

    def solve_ik(self, position: Sequence[float], orientation: Sequence[float]):
        if self.ik_solver is None:
            raise RuntimeError("environment_not_reset")
        return self.ik_solver.solve(position, orientation, rest_poses=self.current_arm_joints())

    def current_arm_joints(self) -> list[float]:
        self._require_connected()
        return [
            float(
                self.bullet.getJointState(
                    self.robot_id,
                    joint_index,
                    physicsClientId=self.physics_client_id,
                )[0]
            )
            for joint_index in ARM_JOINT_INDICES
        ]

    def current_end_effector_pose(self):
        self._require_connected()
        state = self.bullet.getLinkState(
            self.robot_id,
            END_EFFECTOR_LINK,
            computeForwardKinematics=True,
            physicsClientId=self.physics_client_id,
        )
        return tuple(state[4]), tuple(state[5])

    def downward_orientation(self):
        self._require_connected()
        return self.bullet.getQuaternionFromEuler((math.pi, 0.0, -math.pi / 4.0))

    def execute_joint_path(self, path: Sequence[Sequence[float]]) -> ExecutionReport:
        if len(path) < 2:
            return ExecutionReport(False, 0, 0, "empty_path")
        dense_path: list[list[float]] = []
        for start, goal in zip(path, path[1:]):
            segment = interpolate_segment(start, goal, 0.035)
            dense_path.extend(segment if not dense_path else segment[1:])

        start_steps = self.simulation_steps
        start_collisions = self.collision_count
        start_carried_collisions = self.carried_object_collision_count
        for joints in dense_path[1:]:
            self.bullet.setJointMotorControlArray(
                self.robot_id,
                ARM_JOINT_INDICES,
                self.bullet.POSITION_CONTROL,
                targetPositions=list(joints),
                forces=[87.0] * len(ARM_JOINT_INDICES),
                positionGains=[0.08] * len(ARM_JOINT_INDICES),
                physicsClientId=self.physics_client_id,
            )
            self.step(self.config.motor_steps_per_waypoint, check_collisions=True)
            if self.collision_count > start_collisions:
                failure_type = (
                    "execution_carried_object_collision"
                    if self.carried_object_collision_count > start_carried_collisions
                    else "execution_collision"
                )
                return ExecutionReport(
                    False,
                    self.simulation_steps - start_steps,
                    self.collision_count - start_collisions,
                    failure_type,
                )
        return ExecutionReport(
            True,
            self.simulation_steps - start_steps,
            self.collision_count - start_collisions,
        )

    def open_gripper(self) -> None:
        self.set_gripper(self.config.open_gripper_width)
        self.detach_object()

    def close_gripper_and_attach(self) -> bool:
        self.set_gripper(self.config.closed_gripper_width)
        return self.attach_object()

    def set_gripper(self, width: float) -> None:
        if not 0.0 <= width <= self.config.open_gripper_width:
            raise ValueError("gripper width is outside the configured range")
        self._command_gripper(width)

    def attach_object(self) -> bool:
        ee_position, ee_orientation = self.current_end_effector_pose()
        cube_position, cube_orientation = self.bullet.getBasePositionAndOrientation(
            self.cube_id,
            physicsClientId=self.physics_client_id,
        )
        if math.dist(ee_position, cube_position) > 0.085:
            return False

        inverse_position, inverse_orientation = self.bullet.invertTransform(
            ee_position,
            ee_orientation,
        )
        relative_position, relative_orientation = self.bullet.multiplyTransforms(
            inverse_position,
            inverse_orientation,
            cube_position,
            cube_orientation,
        )
        self.grasp_constraint_id = self.bullet.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=END_EFFECTOR_LINK,
            childBodyUniqueId=self.cube_id,
            childLinkIndex=-1,
            jointType=self.bullet.JOINT_FIXED,
            jointAxis=(0.0, 0.0, 0.0),
            parentFramePosition=relative_position,
            childFramePosition=(0.0, 0.0, 0.0),
            parentFrameOrientation=relative_orientation,
            childFrameOrientation=(0.0, 0.0, 0.0, 1.0),
            physicsClientId=self.physics_client_id,
        )
        if self.collision_checker is not None:
            world_body_ids = [self.table_id]
            clearance_margins: dict[int, float] = {}
            if self.obstacle_id is not None:
                world_body_ids.append(self.obstacle_id)
                clearance_margins[self.obstacle_id] = (
                    self.config.carried_object_clearance_margin
                )
            self.collision_checker.attach_carried_body(
                self.cube_id,
                END_EFFECTOR_LINK,
                relative_position,
                relative_orientation,
                world_body_ids,
                clearance_margins=clearance_margins,
                contact_tolerances={
                    self.table_id: self.config.carried_table_penetration_tolerance,
                },
            )
        return True

    def detach_object(self) -> None:
        self._remove_grasp_constraint()

    def object_state(self):
        self._require_connected()
        position, _ = self.bullet.getBasePositionAndOrientation(
            self.cube_id,
            physicsClientId=self.physics_client_id,
        )
        linear_velocity, _ = self.bullet.getBaseVelocity(
            self.cube_id,
            physicsClientId=self.physics_client_id,
        )
        return tuple(position), tuple(linear_velocity)

    def object_is_stable_in_target(self, target_position: Sequence[float]) -> bool:
        position, velocity = self.object_state()
        horizontal_distance = math.dist(position[:2], target_position[:2])
        height_error = abs(position[2] - target_position[2])
        speed = math.sqrt(sum(component**2 for component in velocity))
        return (
            horizontal_distance <= self.config.success_xy_tolerance_m
            and height_error <= self.config.success_z_tolerance_m
            and speed <= self.config.success_max_linear_velocity_mps
        )

    def step(self, count: int = 1, check_collisions: bool = True) -> None:
        self._require_connected()
        for _ in range(count):
            self.bullet.stepSimulation(physicsClientId=self.physics_client_id)
            self.simulation_steps += 1
            if check_collisions and self.collision_checker is not None:
                collisions = self.collision_checker.current_collisions()
                self.collision_count += int(bool(collisions))
                self.carried_object_collision_count += int(
                    any(collision[0] == "carried" for collision in collisions)
                )
            if self.use_gui:
                time.sleep(1.0 / self.config.simulation_hz)

    def _load_base_scene(self) -> None:
        self._require_connected()
        self.plane_id = self.bullet.loadURDF(
            "plane.urdf",
            physicsClientId=self.physics_client_id,
        )
        table_half_extents = [0.5 * value for value in self.config.table_size]
        table_collision = self.bullet.createCollisionShape(
            self.bullet.GEOM_BOX,
            halfExtents=table_half_extents,
            physicsClientId=self.physics_client_id,
        )
        table_visual = self.bullet.createVisualShape(
            self.bullet.GEOM_BOX,
            halfExtents=table_half_extents,
            rgbaColor=(0.72, 0.58, 0.42, 1.0),
            physicsClientId=self.physics_client_id,
        )
        self.table_id = self.bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=table_collision,
            baseVisualShapeIndex=table_visual,
            basePosition=self.config.table_position,
            physicsClientId=self.physics_client_id,
        )
        flags = (
            self.bullet.URDF_USE_SELF_COLLISION
            | self.bullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
        )
        self.robot_id = self.bullet.loadURDF(
            "franka_panda/panda.urdf",
            useFixedBase=True,
            flags=flags,
            physicsClientId=self.physics_client_id,
        )
        self.joint_limits = [
            (
                float(
                    self.bullet.getJointInfo(
                        self.robot_id,
                        index,
                        physicsClientId=self.physics_client_id,
                    )[8]
                ),
                float(
                    self.bullet.getJointInfo(
                        self.robot_id,
                        index,
                        physicsClientId=self.physics_client_id,
                    )[9]
                ),
            )
            for index in ARM_JOINT_INDICES
        ]
        collision_shape = self.bullet.createCollisionShape(
            self.bullet.GEOM_BOX,
            halfExtents=[self.config.cube_half_extent] * 3,
            physicsClientId=self.physics_client_id,
        )
        visual_shape = self.bullet.createVisualShape(
            self.bullet.GEOM_BOX,
            halfExtents=[self.config.cube_half_extent] * 3,
            rgbaColor=(0.85, 0.12, 0.08, 1.0),
            physicsClientId=self.physics_client_id,
        )
        self.cube_id = self.bullet.createMultiBody(
            baseMass=self.config.cube_mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=(0.55, -0.18, self.config.cube_half_extent),
            physicsClientId=self.physics_client_id,
        )
        self.bullet.changeDynamics(
            self.cube_id,
            -1,
            lateralFriction=0.9,
            spinningFriction=0.02,
            rollingFriction=0.01,
            physicsClientId=self.physics_client_id,
        )

    def _refresh_helpers(self) -> None:
        world_bodies = [self.plane_id, self.table_id]
        if self.obstacle_id is not None:
            world_bodies.append(self.obstacle_id)
        self.ik_solver = PandaIKSolver(
            self.bullet,
            self.physics_client_id,
            self.robot_id,
            ARM_JOINT_INDICES,
            END_EFFECTOR_LINK,
            self.joint_limits,
        )
        self.collision_checker = PandaCollisionChecker(
            self.bullet,
            self.physics_client_id,
            self.robot_id,
            ARM_JOINT_INDICES,
            self.joint_limits,
            world_bodies,
            allowed_world_contacts={
                self.plane_id: {-1, 0},
                self.table_id: {-1, 0},
            },
            world_clearance_margins=(
                {self.obstacle_id: self.config.obstacle_clearance_margin}
                if self.obstacle_id is not None
                else {}
            ),
        )

    def _replace_target_marker(self, target_position: Sequence[float]) -> None:
        if self.target_marker_id is not None:
            self.bullet.removeBody(self.target_marker_id, physicsClientId=self.physics_client_id)
        visual_shape = self.bullet.createVisualShape(
            self.bullet.GEOM_CYLINDER,
            radius=self.config.target_radius,
            length=0.004,
            rgbaColor=(0.08, 0.75, 0.18, 0.45),
            physicsClientId=self.physics_client_id,
        )
        self.target_marker_id = self.bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape,
            basePosition=(target_position[0], target_position[1], 0.002),
            physicsClientId=self.physics_client_id,
        )

    def _replace_obstacle(self, enabled: bool) -> None:
        if self.obstacle_id is not None:
            self.bullet.removeBody(self.obstacle_id, physicsClientId=self.physics_client_id)
            self.obstacle_id = None
        if not enabled:
            return
        collision_shape = self.bullet.createCollisionShape(
            self.bullet.GEOM_BOX,
            halfExtents=self.config.obstacle_half_extents,
            physicsClientId=self.physics_client_id,
        )
        visual_shape = self.bullet.createVisualShape(
            self.bullet.GEOM_BOX,
            halfExtents=self.config.obstacle_half_extents,
            rgbaColor=(0.22, 0.28, 0.38, 0.9),
            physicsClientId=self.physics_client_id,
        )
        self.obstacle_id = self.bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.config.obstacle_position,
            physicsClientId=self.physics_client_id,
        )

    def _command_gripper(self, target_position: float) -> None:
        self.bullet.setJointMotorControlArray(
            self.robot_id,
            FINGER_JOINT_INDICES,
            self.bullet.POSITION_CONTROL,
            targetPositions=[target_position, target_position],
            forces=[30.0, 30.0],
            physicsClientId=self.physics_client_id,
        )
        self.step(90, check_collisions=False)

    def _remove_grasp_constraint(self) -> None:
        if self.collision_checker is not None:
            self.collision_checker.detach_carried_body()
        if self.grasp_constraint_id is None or self.bullet is None:
            return
        self.bullet.removeConstraint(
            self.grasp_constraint_id,
            physicsClientId=self.physics_client_id,
        )
        self.grasp_constraint_id = None

    def shutdown(self) -> None:
        self.close()

    def _reset_arm(self, joints: Sequence[float]) -> None:
        for joint_index, value in zip(ARM_JOINT_INDICES, joints):
            self.bullet.resetJointState(
                self.robot_id,
                joint_index,
                float(value),
                physicsClientId=self.physics_client_id,
            )
        # resetJointState only changes the instantaneous state.  Position
        # controllers keep their target from the previous episode unless it is
        # explicitly replaced, so the settling steps in reset() would otherwise
        # pull the robot away from the home configuration.
        self.bullet.setJointMotorControlArray(
            self.robot_id,
            ARM_JOINT_INDICES,
            self.bullet.POSITION_CONTROL,
            targetPositions=list(joints),
            forces=[87.0] * len(ARM_JOINT_INDICES),
            positionGains=[0.08] * len(ARM_JOINT_INDICES),
            physicsClientId=self.physics_client_id,
        )

    def _reset_gripper(self, value: float) -> None:
        for joint_index in FINGER_JOINT_INDICES:
            self.bullet.resetJointState(
                self.robot_id,
                joint_index,
                value,
                physicsClientId=self.physics_client_id,
            )
        self.bullet.setJointMotorControlArray(
            self.robot_id,
            FINGER_JOINT_INDICES,
            self.bullet.POSITION_CONTROL,
            targetPositions=[value] * len(FINGER_JOINT_INDICES),
            forces=[30.0] * len(FINGER_JOINT_INDICES),
            physicsClientId=self.physics_client_id,
        )

    def _require_connected(self) -> None:
        if self.physics_client_id is None or self.bullet is None:
            raise RuntimeError("environment_not_connected")
