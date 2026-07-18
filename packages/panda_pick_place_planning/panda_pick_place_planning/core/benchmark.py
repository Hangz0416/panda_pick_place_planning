from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
from statistics import mean, median
from typing import Iterable

from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv, SceneConfig
from panda_pick_place_planning.core.schemas import EpisodeResult
from panda_pick_place_planning.core.tasks import generate_paired_tasks


def run_benchmark(
    episodes: int,
    seed: int,
    scenes: list[str],
    planners: list[str],
    output_dir: Path,
    use_gui: bool = False,
    config: SceneConfig | None = None,
) -> dict:
    tasks = generate_paired_tasks(episodes, seed, scenes, planners)
    results: list[EpisodeResult] = []
    env = PyBulletPandaEnv(use_gui=use_gui, config=config)
    try:
        runner = PickPlaceRunner(env)
        for task in tasks:
            result = runner.run(task)
            results.append(result)
            print(
                f"episode={result.episode_id} scene={result.scene} planner={result.planner} "
                f"success={result.task_success} failure={result.failure_type or '-'}"
            )
    finally:
        env.close()

    summary = summarize_results(results)
    write_results(output_dir, results, summary)
    return summary


def summarize_results(results: Iterable[EpisodeResult]) -> dict:
    rows = list(results)
    grouped: dict[tuple[str, str], list[EpisodeResult]] = defaultdict(list)
    for result in rows:
        grouped[(result.scene, result.planner)].append(result)

    groups = {}
    for (scene, planner), items in sorted(grouped.items()):
        key = f"{scene}/{planner}"
        groups[key] = _summarize_group(items)

    backend_grouped: dict[
        tuple[str, str, str, str], list[EpisodeResult]
    ] = defaultdict(list)
    for result in rows:
        planner_id = result.planner_id or result.planner
        group_key = (result.backend, result.simulator, result.scene, planner_id)
        backend_grouped[group_key].append(result)
    backend_groups = {
        "/".join(key): _summarize_group(items)
        for key, items in sorted(backend_grouped.items())
    }
    return {
        "schema_version": 5,
        "total_episodes": len(rows),
        "groups": groups,
        "backend_groups": backend_groups,
    }


def _summarize_group(items: list[EpisodeResult]) -> dict:
    planning_times = [item.planning_time_sec for item in items]
    successful_lengths = [
        item.path_length_rad for item in items if item.planning_success
    ]
    scopes = {
        (
            "independent_planning_requests_total"
            if item.simulator == "moveit_mock"
            else "critical_transfer_motion"
        )
        for item in items
    }
    metric_scope = scopes.pop() if len(scopes) == 1 else "mixed_see_backend_groups"
    return {
        "episodes": len(items),
        "top_level_motion_metric_scope": metric_scope,
        "planning_success_rate": _rate(item.planning_success for item in items),
        "execution_success_rate": _rate(item.execution_success for item in items),
        "task_success_rate": _rate(item.task_success for item in items),
        "collision_episode_rate": _rate(item.collision_count > 0 for item in items),
        "avg_planning_time_sec": _mean(planning_times),
        "avg_top_level_planning_time_sec": _mean(planning_times),
        "median_planning_time_sec": _median(planning_times),
        "p95_planning_time_sec": _percentile(planning_times, 0.95),
        "avg_execution_time_sec": _mean(
            item.execution_time_sec for item in items
        ),
        "avg_path_length_rad": _mean(successful_lengths),
        "avg_top_level_path_length_rad": _mean(successful_lengths),
        "median_path_length_rad": _median(successful_lengths),
        "avg_final_xy_error_m": _optional_mean(
            item.final_xy_error_m for item in items
        ),
        "avg_final_z_error_m": _optional_mean(
            item.final_z_error_m for item in items
        ),
        "avg_final_object_speed_mps": _optional_mean(
            item.final_object_speed_mps for item in items
        ),
        "avg_trajectory_points": _mean(
            item.trajectory_points for item in items
        ),
        "avg_max_joint_tracking_error_rad": _optional_mean(
            item.max_joint_tracking_error_rad for item in items
        ),
        "pose_source": (
            items[0].pose_source
            if len({item.pose_source for item in items}) == 1
            else "mixed"
        ),
        "object_variant": (
            items[0].object_variant
            if len({item.object_variant for item in items}) == 1
            else "mixed"
        ),
        "perception_profile": (
            items[0].perception_profile
            if len({item.perception_profile for item in items}) == 1
            else "mixed"
        ),
        "perception_success_rate": _rate(
            item.perception_success for item in items
        ),
        "avg_perception_latency_sec": _optional_mean(
            item.perception_latency_sec for item in items
        ),
        "median_perception_latency_sec": _optional_median(
            item.perception_latency_sec for item in items
        ),
        "p95_perception_latency_sec": _optional_percentile(
            (item.perception_latency_sec for item in items), 0.95
        ),
        "object_position_error_m": _optional_distribution(
            item.object_position_error_m for item in items
        ),
        "target_position_error_m": _optional_distribution(
            item.target_position_error_m for item in items
        ),
        "obstacle_position_error_m": _optional_distribution(
            item.obstacle_position_error_m for item in items
        ),
        "avg_minimum_perception_confidence": _optional_mean(
            item.minimum_perception_confidence for item in items
        ),
        "object_yaw_error_rad": _optional_distribution(
            item.object_yaw_error_rad for item in items
        ),
        "grasp_selection_success_rate": _rate(
            item.grasp_selection_success for item in items
        ),
        "grasp_selection_latency_sec": _optional_distribution(
            item.grasp_selection_latency_sec for item in items
        ),
        "avg_grasp_candidate_count": _mean(
            item.grasp_candidate_count for item in items
        ),
        "selected_grasp_path_length_rad": _optional_distribution(
            item.selected_grasp_path_length_rad for item in items
        ),
        "complete_workpiece_detection_rate": _rate(
            item.detected_workpiece_count
            == (
                3
                if item.object_variant == "rect_prism"
                and item.selected_object_id.startswith("workpiece_")
                else 1
            )
            for item in items
        ),
        "target_selection_success_rate": _rate(
            item.target_selection_success for item in items
        ),
        "target_selection_latency_sec": _optional_distribution(
            item.target_selection_latency_sec for item in items
        ),
        "selected_object_position_error_m": _optional_distribution(
            item.selected_object_position_error_m for item in items
        ),
        "selected_object_yaw_error_rad": _optional_distribution(
            item.selected_object_yaw_error_rad for item in items
        ),
        "selected_object_score": _optional_distribution(
            item.selected_object_score for item in items
        ),
        "avg_total_candidate_count": _mean(
            item.total_candidate_count for item in items
        ),
        "avg_feasible_candidate_count": _mean(
            item.feasible_candidate_count for item in items
        ),
        "max_nonselected_displacement_m": _optional_distribution(
            item.max_nonselected_displacement_m for item in items
        ),
        "failure_counts": dict(
            Counter(item.failure_type or "success" for item in items)
        ),
    }


def write_results(output_dir: Path, results: list[EpisodeResult], summary: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = output_dir / "benchmark_results.jsonl"
    with jsonl_path.open("w", encoding="utf-8", newline="\n") as file:
        for result in results:
            file.write(result.to_json() + "\n")

    csv_path = output_dir / "benchmark_results.csv"
    rows = [result.to_dict() for result in results]
    if rows:
        for row in rows:
            row["stage_metrics"] = json.dumps(
                row["stage_metrics"], ensure_ascii=False, sort_keys=True
            )
        with csv_path.open("w", encoding="utf-8", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    (output_dir / "benchmark_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def _rate(values: Iterable[bool]) -> float:
    items = list(values)
    return round(sum(bool(value) for value in items) / len(items), 4) if items else 0.0


def _mean(values: Iterable[float]) -> float:
    items = list(values)
    return round(mean(items), 6) if items else 0.0


def _optional_mean(values: Iterable[float | None]) -> float | None:
    items = [value for value in values if value is not None]
    return round(mean(items), 6) if items else None


def _optional_median(values: Iterable[float | None]) -> float | None:
    items = [value for value in values if value is not None]
    return round(median(items), 6) if items else None


def _optional_percentile(
    values: Iterable[float | None], probability: float
) -> float | None:
    items = [value for value in values if value is not None]
    return _percentile(items, probability) if items else None


def _optional_distribution(values: Iterable[float | None]) -> dict:
    items = [value for value in values if value is not None]
    return {
        "samples": len(items),
        "mean": round(mean(items), 6) if items else None,
        "median": round(median(items), 6) if items else None,
        "p95": _percentile(items, 0.95) if items else None,
    }


def _median(values: Iterable[float]) -> float:
    items = list(values)
    return round(median(items), 6) if items else 0.0


def _percentile(values: Iterable[float], probability: float) -> float:
    items = sorted(values)
    if not items:
        return 0.0
    index = (len(items) - 1) * probability
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return round(items[lower], 6)
    value = items[lower] * (upper - index) + items[upper] * (index - lower)
    return round(value, 6)


def main() -> None:
    parser = argparse.ArgumentParser(description="运行 Panda 抓取放置规划器对照实验。")
    parser.add_argument("--episodes", type=int, default=10, help="每个场景、每种规划器的任务数。")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--scenes",
        nargs="+",
        choices=["clear", "obstacle"],
        default=["clear", "obstacle"],
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        choices=["linear", "rrt_connect"],
        default=["linear", "rrt_connect"],
    )
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    parser.add_argument(
        "--config",
        type=Path,
        help="可选 SceneConfig YAML；省略时使用代码内同版本默认值。",
    )
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    summary = run_benchmark(
        episodes=args.episodes,
        seed=args.seed,
        scenes=args.scenes,
        planners=args.planners,
        output_dir=args.output_dir,
        use_gui=args.gui,
        config=SceneConfig.from_yaml(args.config) if args.config else None,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
