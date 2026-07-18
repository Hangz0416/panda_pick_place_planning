from __future__ import annotations

import math
import random

from panda_pick_place_planning.core.schemas import TaskSpec, WorkpieceSpec


def generate_task(
    episode_id: int,
    task_seed: int,
    planner: str,
    scene: str,
    planner_seed: int | None = None,
    object_variant: str = "cube",
    workpiece_count: int = 1,
) -> TaskSpec:
    if workpiece_count not in {1, 3}:
        raise ValueError("workpiece_count must be one or three")
    if workpiece_count == 3 and object_variant != "rect_prism":
        raise ValueError("three-workpiece tasks require rect_prism objects")
    rng = random.Random(task_seed)
    if scene == "obstacle":
        # Keep the grasp on the home side of the barrier.  Randomly swapping
        # sides makes the initial home-to-pre-grasp motion cross the obstacle,
        # which confounds the transfer-planner comparison.
        x_position = rng.uniform(0.58, 0.61)
        start_y = rng.uniform(0.21, 0.23)
        target_y = rng.uniform(-0.23, -0.22)
    else:
        x_position = rng.uniform(0.52, 0.62)
        start_y = rng.uniform(-0.22, -0.12)
        target_y = rng.uniform(0.12, 0.22)
        if rng.random() < 0.5:
            start_y, target_y = target_y, start_y

    object_yaw = rng.uniform(-math.pi / 2.0, math.pi / 2.0)
    if object_variant == "cube":
        object_yaw = 0.0
    workpieces: tuple[WorkpieceSpec, ...] = ()
    if workpiece_count == 3:
        # X-separated slots make same-colour instance ordering stable without
        # consulting reset truth in the perception path.
        center_x = rng.uniform(0.56, 0.58)
        positions = [
            (center_x + offset, start_y + rng.uniform(-0.012, 0.012), 0.025)
            for offset in (-0.11, 0.0, 0.11)
        ]
        generated = []
        for index, position in enumerate(positions):
            yaw = rng.uniform(-math.pi / 2.0, math.pi / 2.0)
            half_yaw = 0.5 * yaw
            generated.append(
                WorkpieceSpec(
                    object_id=f"workpiece_{index}",
                    object_variant="rect_prism",
                    position=position,
                    orientation_xyzw=(
                        0.0,
                        0.0,
                        math.sin(half_yaw),
                        math.cos(half_yaw),
                    ),
                )
            )
        workpieces = tuple(generated)
        x_position, start_y, _ = workpieces[0].position
        object_yaw = workpieces[0].yaw_rad
    return TaskSpec(
        episode_id=episode_id,
        task_seed=task_seed,
        planner=planner,
        planner_seed=task_seed + 10000 if planner_seed is None else planner_seed,
        scene=scene,
        object_position=(x_position, start_y, 0.025),
        target_position=(x_position, target_y, 0.025),
        object_variant=object_variant,
        object_yaw_rad=object_yaw,
        workpieces=workpieces,
    )


def generate_paired_tasks(
    episodes: int,
    seed: int,
    scenes: list[str],
    planners: list[str],
) -> list[TaskSpec]:
    tasks = []
    episode_id = 0
    for scene_index, scene in enumerate(scenes):
        for local_index in range(episodes):
            task_seed = seed + scene_index * 100000 + local_index
            planner_seed = seed + scene_index * 100000 + local_index + 50000
            for planner in planners:
                tasks.append(
                    generate_task(
                        episode_id=episode_id,
                        task_seed=task_seed,
                        planner=planner,
                        scene=scene,
                        planner_seed=planner_seed,
                    )
                )
                episode_id += 1
    return tasks


def generate_backend_tasks(
    episodes_per_scene: int,
    seed: int,
    scenes: list[str],
    planners: list[str],
    *,
    backend: str,
    simulator: str,
    grasp_mode: str = "assisted",
    object_variant: str = "cube",
    perception_profile: str = "nominal",
    workpiece_count: int = 1,
) -> list[TaskSpec]:
    """Generate deterministic backend tasks with shared geometry per planner.

    ``episodes_per_scene=5`` is the required quick-check shape; setting it to
    50 with both scenes yields the 100-episode MoveIt regression batch.
    """

    if episodes_per_scene <= 0:
        raise ValueError("episodes_per_scene must be positive")
    if not scenes:
        raise ValueError("at least one scene is required")
    if not planners:
        raise ValueError("at least one planner is required")

    tasks: list[TaskSpec] = []
    episode_id = 0
    for scene_index, scene in enumerate(scenes):
        for local_index in range(episodes_per_scene):
            task_seed = seed + scene_index * 100000 + local_index
            planner_seed = task_seed + 50000
            for planner in planners:
                task = generate_task(
                    episode_id=episode_id,
                    task_seed=task_seed,
                    planner=planner,
                    scene=scene,
                    planner_seed=planner_seed,
                    object_variant=object_variant,
                    workpiece_count=workpiece_count,
                )
                tasks.append(
                    TaskSpec.from_dict(
                        {
                            **task.to_dict(),
                            "backend": backend,
                            "simulator": simulator,
                            "grasp_mode": grasp_mode,
                            "perception_profile": perception_profile,
                        }
                    )
                )
                episode_id += 1
    return tasks
