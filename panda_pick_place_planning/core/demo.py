from __future__ import annotations

import argparse

from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv
from panda_pick_place_planning.core.tasks import generate_task


def main() -> None:
    parser = argparse.ArgumentParser(description="运行单次 Panda 抓取放置可视化演示。")
    parser.add_argument("--planner", choices=["linear", "rrt_connect"], default="rrt_connect")
    parser.add_argument("--scene", choices=["clear", "obstacle"], default="obstacle")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    task = generate_task(0, args.seed, args.planner, args.scene)
    env = PyBulletPandaEnv(use_gui=True)
    try:
        result = PickPlaceRunner(env).run(task)
        print(result.to_json())
        input("演示结束，按 Enter 关闭 PyBullet...")
    finally:
        env.close()


if __name__ == "__main__":
    main()
