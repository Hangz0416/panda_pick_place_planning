import unittest

from panda_pick_place_planning.core.benchmark import summarize_results
from panda_pick_place_planning.core.schemas import EpisodeResult


def result(planner, success, planning_success=True, failure_type=""):
    return EpisodeResult(
        episode_id=0,
        task_seed=42,
        planner=planner,
        planner_seed=9,
        scene="obstacle",
        planning_success=planning_success,
        planning_time_sec=0.1,
        explored_nodes=10,
        path_waypoints=4,
        path_length_rad=1.2,
        execution_sim_steps=200,
        task_success=success,
        collision_count=0,
        failure_type=failure_type,
        object_initial_position=(0.5, -0.2, 0.025),
        object_final_position=(0.5, 0.2, 0.025),
        target_position=(0.5, 0.2, 0.025),
    )


class BenchmarkSummaryTest(unittest.TestCase):
    def test_groups_results_by_scene_and_planner(self):
        summary = summarize_results(
            [
                result("linear", False, False, "planning_collision"),
                result("linear", True),
                result("rrt_connect", True),
                result("rrt_connect", True),
            ]
        )

        self.assertEqual(summary["total_episodes"], 4)
        self.assertEqual(summary["groups"]["obstacle/linear"]["task_success_rate"], 0.5)
        self.assertEqual(summary["groups"]["obstacle/rrt_connect"]["task_success_rate"], 1.0)
        self.assertEqual(
            summary["groups"]["obstacle/linear"]["failure_counts"]["planning_collision"],
            1,
        )


if __name__ == "__main__":
    unittest.main()
