import unittest

from panda_pick_place_planning.core.benchmark import summarize_results
from panda_pick_place_planning.core.schemas import EpisodeResult


def result(
    planner,
    success,
    planning_success=True,
    failure_type="",
    *,
    final_xy_error_m=None,
    final_z_error_m=None,
    final_object_speed_mps=None,
):
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
        final_xy_error_m=final_xy_error_m,
        final_z_error_m=final_z_error_m,
        final_object_speed_mps=final_object_speed_mps,
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
        self.assertIn("pybullet/pybullet/obstacle/linear", summary["backend_groups"])
        self.assertEqual(
            summary["groups"]["obstacle/rrt_connect"]["median_planning_time_sec"],
            0.1,
        )
        self.assertIsNone(
            summary["groups"]["obstacle/rrt_connect"][
                "avg_max_joint_tracking_error_rad"
            ]
        )
        self.assertEqual(
            summary["groups"]["obstacle/rrt_connect"][
                "top_level_motion_metric_scope"
            ],
            "critical_transfer_motion",
        )
        self.assertIsNone(
            summary["groups"]["obstacle/rrt_connect"]["avg_final_xy_error_m"]
        )

    def test_optional_object_metrics_ignore_unreported_episodes(self):
        summary = summarize_results(
            [
                result("rrt_connect", False),
                result(
                    "rrt_connect",
                    True,
                    final_xy_error_m=0.02,
                    final_z_error_m=0.004,
                    final_object_speed_mps=0.03,
                ),
            ]
        )
        group = summary["groups"]["obstacle/rrt_connect"]

        self.assertEqual(group["avg_final_xy_error_m"], 0.02)
        self.assertEqual(group["avg_final_z_error_m"], 0.004)
        self.assertEqual(group["avg_final_object_speed_mps"], 0.03)


if __name__ == "__main__":
    unittest.main()
