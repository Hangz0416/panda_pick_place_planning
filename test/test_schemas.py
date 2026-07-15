import json
import unittest

from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec


class TaskSpecTest(unittest.TestCase):
    def test_json_round_trip(self):
        task = TaskSpec(
            episode_id=3,
            task_seed=42,
            planner="rrt_connect",
            planner_seed=7,
            scene="obstacle",
            object_position=(0.55, -0.18, 0.025),
            target_position=(0.55, 0.18, 0.025),
        )

        restored = TaskSpec.from_json(task.to_json())

        self.assertEqual(restored, task)
        self.assertEqual(json.loads(task.to_json())["planner"], "rrt_connect")

    def test_rejects_unknown_planner(self):
        with self.assertRaises(ValueError):
            TaskSpec(
                episode_id=0,
                task_seed=0,
                planner="magic",
                planner_seed=0,
                scene="clear",
                object_position=(0.5, 0.0, 0.025),
                target_position=(0.6, 0.0, 0.025),
            )

    def test_episode_result_json_round_trip(self):
        result = EpisodeResult(
            episode_id=1,
            task_seed=2,
            planner="linear",
            planner_seed=3,
            scene="clear",
            planning_success=False,
            planning_time_sec=0.0,
            explored_nodes=0,
            path_waypoints=0,
            path_length_rad=0.0,
            execution_sim_steps=0,
            task_success=False,
            collision_count=0,
            failure_type="runtime_error",
            object_initial_position=(0.5, 0.0, 0.025),
            object_final_position=(0.5, 0.0, 0.025),
            target_position=(0.6, 0.0, 0.025),
        )

        self.assertEqual(EpisodeResult.from_json(result.to_json()), result)


if __name__ == "__main__":
    unittest.main()
