import unittest

from panda_pick_place_planning.core.tasks import generate_paired_tasks, generate_task


class TaskGenerationTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
