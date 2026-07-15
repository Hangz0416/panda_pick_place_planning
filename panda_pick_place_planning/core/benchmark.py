from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
from statistics import mean
from typing import Iterable

from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv
from panda_pick_place_planning.core.schemas import EpisodeResult
from panda_pick_place_planning.core.tasks import generate_paired_tasks


def run_benchmark(
    episodes: int,
    seed: int,
    scenes: list[str],
    planners: list[str],
    output_dir: Path,
    use_gui: bool = False,
) -> dict:
    tasks = generate_paired_tasks(episodes, seed, scenes, planners)
    results: list[EpisodeResult] = []
    env = PyBulletPandaEnv(use_gui=use_gui)
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
        groups[key] = {
            "episodes": len(items),
            "planning_success_rate": _rate(item.planning_success for item in items),
            "task_success_rate": _rate(item.task_success for item in items),
            "collision_episode_rate": _rate(item.collision_count > 0 for item in items),
            "avg_planning_time_sec": _mean(item.planning_time_sec for item in items),
            "avg_path_length_rad": _mean(
                item.path_length_rad for item in items if item.planning_success
            ),
            "failure_counts": dict(
                Counter(item.failure_type or "success" for item in items)
            ),
        }
    return {"total_episodes": len(rows), "groups": groups}


def write_results(output_dir: Path, results: list[EpisodeResult], summary: dict) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    jsonl_path = output_dir / "benchmark_results.jsonl"
    with jsonl_path.open("w", encoding="utf-8", newline="\n") as file:
        for result in results:
            file.write(result.to_json() + "\n")

    csv_path = output_dir / "benchmark_results.csv"
    rows = [result.to_dict() for result in results]
    if rows:
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


def main() -> None:
    parser = argparse.ArgumentParser(description="运行 Panda 抓取放置规划器对照实验。")
    parser.add_argument("--episodes", type=int, default=10, help="每个场景、每种规划器的任务数。")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--scenes", nargs="+", choices=["clear", "obstacle"], default=["clear", "obstacle"])
    parser.add_argument("--planners", nargs="+", choices=["linear", "rrt_connect"], default=["linear", "rrt_connect"])
    parser.add_argument("--output-dir", type=Path, default=Path("results"))
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    summary = run_benchmark(
        episodes=args.episodes,
        seed=args.seed,
        scenes=args.scenes,
        planners=args.planners,
        output_dir=args.output_dir,
        use_gui=args.gui,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
