import unittest
import math

from panda_pick_place_planning.core.tasks import (
    generate_paired_tasks,
    generate_task,
)


class TaskGenerationTest(unittest.TestCase):
    def test_generates_three_deterministic_separated_workpieces(self):
        options = {
            "object_variant": "rect_prism",
            "workpiece_count": 3,
        }
        first = generate_task(0, 42, "rrt_connect", "obstacle", **options)
        second = generate_task(0, 42, "rrt_connect", "obstacle", **options)

        self.assertEqual(first, second)
        self.assertEqual(first.workpiece_count, 3)
        self.assertEqual(
            [item.object_id for item in first.workpieces],
            ["workpiece_0", "workpiece_1", "workpiece_2"],
        )
        positions = [item.position for item in first.workpieces]
        self.assertGreaterEqual(positions[1][0] - positions[0][0], 0.09)
        self.assertGreaterEqual(positions[2][0] - positions[1][0], 0.09)

    def test_same_seed_generates_same_geometry(self):
        linear = generate_task(0, 42, "linear", "obstacle", planner_seed=99)
        rrt = generate_task(1, 42, "rrt_connect", "obstacle", planner_seed=99)

        self.assertEqual(linear.object_position, rrt.object_position)
        self.assertEqual(linear.target_position, rrt.target_position)
        self.assertEqual(linear.planner_seed, rrt.planner_seed)

    def test_paired_tasks_are_balanced(self):
        tasks = generate_paired_tasks(
            episodes=3,
            seed=7,
            scenes=["clear", "obstacle"],
            planners=["linear", "rrt_connect"],
        )

        self.assertEqual(len(tasks), 12)
        self.assertEqual(sum(task.planner == "linear" for task in tasks), 6)
        self.assertEqual(sum(task.scene == "obstacle" for task in tasks), 6)

    def test_task_crosses_between_table_sides(self):
        task = generate_task(0, 12, "linear", "obstacle")

        self.assertLess(task.object_position[1] * task.target_position[1], 0.0)
        self.assertGreater(task.object_position[1], 0.0)
        self.assertLess(task.target_position[1], 0.0)

    def test_rect_prism_yaw_is_seeded_and_in_symmetric_interval(self):
        first = generate_task(
            0, 42, "rrt_connect", "clear", object_variant="rect_prism"
        )
        second = generate_task(
            1, 42, "rrt_connect", "clear", object_variant="rect_prism"
        )

        self.assertEqual(first.object_yaw_rad, second.object_yaw_rad)
        self.assertGreaterEqual(first.object_yaw_rad, -math.pi / 2.0)
        self.assertLess(first.object_yaw_rad, math.pi / 2.0)


if __name__ == "__main__":
    unittest.main()
