import unittest

from panda_pick_place_planning.core.planners.base import PlanningRequest
from panda_pick_place_planning.core.planners.linear import LinearPlanner


class LinearPlannerTest(unittest.TestCase):
    def test_returns_path_with_exact_endpoints(self):
        request = PlanningRequest(
            start=[0.0, 0.0],
            goal=[1.0, 0.0],
            joint_limits=[(-2.0, 2.0), (-2.0, 2.0)],
            state_validator=lambda _: True,
            collision_resolution=0.2,
        )

        result = LinearPlanner().plan(request)

        self.assertTrue(result.success)
        self.assertEqual(result.path[0], request.start)
        self.assertEqual(result.path[-1], request.goal)

    def test_rejects_segment_crossing_obstacle(self):
        def valid(config):
            x_value, y_value = config
            return not (0.4 <= x_value <= 0.6 and abs(y_value) <= 0.2)

        request = PlanningRequest(
            start=[0.0, 0.0],
            goal=[1.0, 0.0],
            joint_limits=[(-1.0, 2.0), (-1.0, 1.0)],
            state_validator=valid,
            collision_resolution=0.05,
        )

        result = LinearPlanner().plan(request)

        self.assertFalse(result.success)
        self.assertEqual(result.failure_type, "planning_collision")


if __name__ == "__main__":
    unittest.main()
