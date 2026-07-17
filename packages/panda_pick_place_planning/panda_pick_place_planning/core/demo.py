from __future__ import annotations

import argparse
from pathlib import Path

from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv, SceneConfig
from panda_pick_place_planning.core.tasks import generate_task


def main() -> None:
    parser = argparse.ArgumentParser(description="运行单次 Panda 抓取放置可视化演示。")
    parser.add_argument("--planner", choices=["linear", "rrt_connect"], default="rrt_connect")
    parser.add_argument("--scene", choices=["clear", "obstacle"], default="obstacle")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--config",
        type=Path,
        help="可选 SceneConfig YAML；省略时使用代码内同版本默认值。",
    )
    parser.add_argument(
        "--gui",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="启用 PyBullet GUI；在服务器或 CI 中使用 --no-gui。",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="可选单 episode JSONL 输出路径；默认拒绝覆盖已有证据文件。",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="允许 --output 覆盖已有文件。",
    )
    args = parser.parse_args()

    if args.output and args.output.exists() and not args.force:
        parser.error(f"output already exists: {args.output}; pass --force to overwrite")

    task = generate_task(0, args.seed, args.planner, args.scene)
    scene_config = SceneConfig.from_yaml(args.config) if args.config else None
    env = PyBulletPandaEnv(use_gui=args.gui, config=scene_config)
    try:
        result = PickPlaceRunner(env).run(task)
        serialized = result.to_json()
        print(serialized)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(serialized + "\n", encoding="utf-8")
        if args.gui:
            input("演示结束，按 Enter 关闭 PyBullet...")
    finally:
        env.close()


if __name__ == "__main__":
    main()
