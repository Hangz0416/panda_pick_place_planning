"""Optional two-attempt launcher that retries readiness failures only."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser()
    value.add_argument("--scene", choices=("clear", "obstacle"), default="clear")
    value.add_argument("--seed", type=int, default=42)
    value.add_argument("--pose-source", choices=("ground_truth", "rgbd"), default="rgbd")
    value.add_argument("--perception-profile", default="nominal")
    value.add_argument("--headless", action="store_true")
    value.add_argument("--use-rviz", action="store_true")
    value.add_argument("--keep-open-after-result", action="store_true")
    value.add_argument("--output-path", default="")
    value.add_argument("--max-attempts", type=int, choices=(1, 2), default=2)
    return value


def main(args=None) -> int:
    options = parser().parse_args(args)
    output = Path(options.output_path).expanduser() if options.output_path else (
        Path.home() / ".ros" / "panda_pick_place" / "runs"
        / datetime.now().strftime("%Y%m%d_%H%M%S_%f") / "episodes.jsonl"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    status_path = output.parent / "run_status.json"
    command = [
        "ros2", "launch", "panda_gz_bringup", "panda_visual_pick_place.launch.py",
        f"scene:={options.scene}", f"seed:={options.seed}",
        "object_variant:=rect_prism", "workpiece_count:=3",
        f"pose_source:={options.pose_source}",
        f"perception_profile:={options.perception_profile}",
        f"headless:={'true' if options.headless else 'false'}",
        f"use_rviz:={'true' if options.use_rviz else 'false'}",
        f"keep_open_after_result:={'true' if options.keep_open_after_result else 'false'}",
        f"output_path:={output}",
    ]
    for attempt in range(1, options.max_attempts + 1):
        completed = subprocess.run(command, check=False)
        status = {}
        if status_path.exists():
            try:
                status = json.loads(status_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                status = {}
        if completed.returncode == 0:
            return 0
        if status.get("readiness") != "failed" or attempt >= options.max_attempts:
            if status:
                status["teardown"] = "abnormal_exit"
                status["launch_exit_code"] = completed.returncode
                status_path.write_text(json.dumps(status, indent=2), encoding="utf-8")
            return int(completed.returncode or 1)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
