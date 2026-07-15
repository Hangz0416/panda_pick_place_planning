from __future__ import annotations

import random

from panda_pick_place_planning.core.schemas import TaskSpec


def generate_task(
    episode_id: int,
    task_seed: int,
    planner: str,
    scene: str,
    planner_seed: int | None = None,
) -> TaskSpec:
    rng = random.Random(task_seed)
    x_position = rng.uniform(0.52, 0.62)
    if scene == "obstacle":
        start_y = rng.uniform(-0.23, -0.18)
        target_y = rng.uniform(0.18, 0.23)
    else:
        start_y = rng.uniform(-0.22, -0.12)
        target_y = rng.uniform(0.12, 0.22)
    if rng.random() < 0.5:
        start_y, target_y = target_y, start_y

    return TaskSpec(
        episode_id=episode_id,
        task_seed=task_seed,
        planner=planner,
        planner_seed=task_seed + 10000 if planner_seed is None else planner_seed,
        scene=scene,
        object_position=(x_position, start_y, 0.025),
        target_position=(x_position, target_y, 0.025),
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
