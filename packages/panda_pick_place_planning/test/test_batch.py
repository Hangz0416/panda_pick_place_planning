import unittest
from pathlib import Path

from panda_pick_place_planning.core.batch import BatchSequencer, batch_exit_code
from panda_pick_place_planning.core.schemas import EpisodeResult
from panda_pick_place_planning.core.tasks import generate_backend_tasks


def result_for(task, *, episode_id=None):
    return EpisodeResult(
        episode_id=task.episode_id if episode_id is None else episode_id,
        task_seed=task.task_seed,
        planner=task.planner,
        planner_seed=task.planner_seed,
        scene=task.scene,
        planning_success=True,
        planning_time_sec=0.1,
        explored_nodes=0,
        path_waypoints=5,
        path_length_rad=1.0,
        execution_sim_steps=0,
        task_success=True,
        collision_count=0,
        failure_type="",
        object_initial_position=task.object_position,
        object_final_position=task.target_position,
        target_position=task.target_position,
        backend=task.backend,
        simulator=task.simulator,
        planner_id="RRTConnectkConfigDefault",
    )


class BatchSequencerTest(unittest.TestCase):
    def setUp(self):
        self.tasks = generate_backend_tasks(
            2,
            42,
            ["clear", "obstacle"],
            ["rrt_connect"],
            backend="moveit",
            simulator="moveit_mock",
        )

    def test_tasks_have_deterministic_balanced_geometry(self):
        self.assertEqual(len(self.tasks), 4)
        self.assertEqual([task.scene for task in self.tasks], [
            "clear",
            "clear",
            "obstacle",
            "obstacle",
        ])
        self.assertEqual([task.episode_id for task in self.tasks], [0, 1, 2, 3])
        self.assertTrue(all(task.backend == "moveit" for task in self.tasks))
        self.assertTrue(
            all(task.simulator == "moveit_mock" for task in self.tasks)
        )

    def test_only_matching_result_advances_batch(self):
        sequencer = BatchSequencer(self.tasks)
        first = sequencer.next_task()

        self.assertFalse(sequencer.accept(result_for(first, episode_id=999)))
        self.assertEqual(sequencer.index, 0)
        self.assertIs(sequencer.inflight, first)
        self.assertTrue(sequencer.accept(result_for(first)))
        self.assertEqual(sequencer.index, 1)
        self.assertIsNone(sequencer.inflight)

    def test_terminal_status_has_shell_visible_exit_code(self):
        self.assertEqual(batch_exit_code("complete"), 0)
        self.assertNotEqual(batch_exit_code("timeout"), 0)
        with self.assertRaises(ValueError):
            batch_exit_code("running")

    def test_five_run_quick_check_has_ten_episodes(self):
        tasks = generate_backend_tasks(
            5,
            42,
            ["clear", "obstacle"],
            ["rrt_connect"],
            backend="moveit",
            simulator="gazebo",
        )

        self.assertEqual(len(tasks), 10)
        self.assertEqual(len({task.task_seed for task in tasks}), 10)

    def test_formal_batch_has_fifty_clear_and_fifty_obstacle(self):
        tasks = generate_backend_tasks(
            50,
            42,
            ["clear", "obstacle"],
            ["rrt_connect"],
            backend="moveit",
            simulator="moveit_mock",
        )

        self.assertEqual(len(tasks), 100)
        self.assertEqual(sum(task.scene == "clear" for task in tasks), 50)
        self.assertEqual(sum(task.scene == "obstacle" for task in tasks), 50)

    def test_package_help_documents_quick_and_formal_commands(self):
        readme = (Path(__file__).parents[1] / "README.md").read_text(
            encoding="utf-8"
        )

        self.assertIn("episodes_per_scene:=5", readme)
        self.assertIn("episodes_per_scene:=50", readme)
        self.assertIn("not Gazebo physics execution", readme)
        batch_launch = (
            Path(__file__).parents[1] / "launch" / "batch_benchmark.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("OnProcessExit", batch_launch)
        self.assertIn("Shutdown", batch_launch)


if __name__ == "__main__":
    unittest.main()
