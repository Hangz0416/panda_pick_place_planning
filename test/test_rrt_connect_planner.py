import unittest

from panda_pick_place_planning.core.planners.base import PlanningRequest, edge_is_valid
from panda_pick_place_planning.core.planners.rrt_connect import RRTConnectPlanner


def obstacle_free(config):
    x_value, y_value = config
    return not (0.4 <= x_value <= 0.6 and -0.35 <= y_value <= 0.35)


class RRTConnectPlannerTest(unittest.TestCase):
    def make_request(self, seed=7):
        return PlanningRequest(
            start=[0.0, 0.0],
            goal=[1.0, 0.0],
            joint_limits=[(-0.2, 1.2), (-1.0, 1.0)],
            state_validator=obstacle_free,
            collision_resolution=0.03,
            step_size=0.15,
            timeout_sec=2.0,
            max_iterations=3000,
            goal_bias=0.15,
            seed=seed,
        )

    def test_routes_around_obstacle(self):
        request = self.make_request()

        result = RRTConnectPlanner().plan(request)

        self.assertTrue(result.success, result.failure_type)
        self.assertEqual(result.path[0], request.start)
        self.assertEqual(result.path[-1], request.goal)
        for start, goal in zip(result.path, result.path[1:]):
            self.assertTrue(
                edge_is_valid(start, goal, request.collision_resolution, request.state_validator)
            )

    def test_same_seed_is_reproducible(self):
        request = self.make_request(seed=19)

        first = RRTConnectPlanner().plan(request)
        second = RRTConnectPlanner().plan(request)

        self.assertTrue(first.success)
        self.assertEqual(first.path, second.path)
        self.assertEqual(first.explored_nodes, second.explored_nodes)

    def test_uses_direct_edge_when_no_obstacle_exists(self):
        request = PlanningRequest(
            start=[0.0, 0.0],
            goal=[1.0, 0.0],
            joint_limits=[(-1.0, 2.0), (-1.0, 1.0)],
            state_validator=lambda _: True,
            collision_resolution=0.05,
            seed=5,
        )

        result = RRTConnectPlanner().plan(request)

        self.assertTrue(result.success)
        self.assertEqual(result.path, [request.start, request.goal])
        self.assertEqual(result.explored_nodes, 2)


if __name__ == "__main__":
    unittest.main()
