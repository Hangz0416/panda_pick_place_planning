from pathlib import Path
import unittest

from panda_pick_place_planning.core.backends import (
    BackendMotionReport,
    ManipulationBackend,
    PyBulletBackend,
)
from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.pybullet_env import SceneConfig
from panda_pick_place_planning.core.schemas import (
    PlanResult,
    SceneEstimate,
    SceneObjectEstimate,
    TaskSpec,
)


class FakeBackend:
    hover_height = 0.24
    grasp_height = 0.075
    place_height = 0.09
    open_gripper_width = 0.04
    closed_gripper_width = 0.0

    def __init__(self):
        self.simulation_steps = 0
        self.collision_count = 0
        self.gripper_commands = []
        self.attached = False
        self.released = False
        self.target_position = (0.6, 0.2, 0.025)
        self.motion_targets = []
        self.applied_estimate = None

    def reset(self, task):
        self.target_position = task.target_position

    def move_to_pose(
        self,
        position,
        orientation,
        *,
        planner="linear",
        seed=0,
        timeout_sec=1.0,
    ):
        self.motion_targets.append(tuple(position))
        self.simulation_steps += 3
        plan = PlanResult(
            True,
            path=[[0.0], [1.0]],
            planning_time_sec=0.01,
            explored_nodes=2,
        )
        return BackendMotionReport(True, True, 3, 0, plan_result=plan)

    def move_home(self):
        return self.move_to_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def set_gripper(self, width):
        self.gripper_commands.append(width)

    def attach_object(self):
        self.attached = True
        return True

    def detach_object(self):
        self.released = True

    def wait_for_object_settle(self):
        self.simulation_steps += 2

    def object_state(self):
        position = self.target_position if self.released else (0.5, -0.2, 0.025)
        return position, (0.0, 0.0, 0.0)

    def object_is_stable_in_target(self, target_position):
        return self.released and target_position == self.target_position

    def downward_orientation(self):
        return (0.0, 1.0, 0.0, 0.0)

    def apply_scene_estimate(self, estimate, scene):
        self.applied_estimate = (estimate, scene)
        return True

    def shutdown(self):
        return None


class BackendContractTest(unittest.TestCase):
    def test_runner_uses_runtime_backend_contract(self):
        backend = FakeBackend()
        self.assertIsInstance(backend, ManipulationBackend)
        task = TaskSpec(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="clear",
            object_position=(0.5, -0.2, 0.025),
            target_position=(0.6, 0.2, 0.025),
        )

        result = PickPlaceRunner(backend).run(task)

        self.assertTrue(result.task_success)
        self.assertTrue(result.object_attached)
        self.assertTrue(result.object_released)
        self.assertEqual(result.backend, "pybullet")
        self.assertEqual(result.planner_id, "rrt_connect")
        self.assertIn("transfer", result.stage_metrics)
        self.assertIn(backend.open_gripper_width, backend.gripper_commands)
        self.assertIn(backend.closed_gripper_width, backend.gripper_commands)

    def test_installed_yaml_matches_scene_defaults(self):
        config_path = Path(__file__).parents[1] / "config" / "scene_defaults.yaml"

        loaded = SceneConfig.from_yaml(config_path)

        self.assertEqual(loaded, SceneConfig())

    def test_pybullet_backend_uses_injected_scene_config(self):
        config = SceneConfig(hover_height=0.31)

        backend = PyBulletBackend(use_gui=False, config=config)

        self.assertIs(backend.env.config, config)
        self.assertEqual(backend.hover_height, 0.31)

    def test_runner_does_not_claim_success_without_collision_observation(self):
        backend = FakeBackend()
        backend.backend_name = "moveit"
        backend.simulator_name = "gazebo"
        backend.planner_id = "RRTConnectkConfigDefault"
        backend.collision_state_known = False
        task = TaskSpec(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="clear",
            object_position=(0.5, -0.2, 0.025),
            target_position=(0.6, 0.2, 0.025),
            backend="moveit",
        )

        result = PickPlaceRunner(backend).run(task)

        self.assertFalse(result.task_success)
        self.assertTrue(result.execution_success)
        self.assertEqual(result.failure_type, "collision_state_unavailable")
        self.assertEqual(result.simulator, "gazebo")

    def test_rgbd_control_targets_do_not_leak_task_truth(self):
        class OffsetVisualProvider:
            pose_source = "rgbd"

            @staticmethod
            def estimate(task):
                return SceneEstimate(
                    episode_id=task.episode_id,
                    pose_source="rgbd",
                    success=True,
                    latency_sec=0.02,
                    estimates={
                        "cube": SceneObjectEstimate(
                            "cube", (0.52, -0.18, 0.025), 0.9, 200
                        ),
                        "target": SceneObjectEstimate(
                            "target", (0.58, 0.18, 0.025), 0.8, 500
                        ),
                    },
                )

        backend = FakeBackend()
        task = TaskSpec(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="clear",
            object_position=(0.5, -0.2, 0.025),
            target_position=(0.6, 0.2, 0.025),
        )

        result = PickPlaceRunner(backend, OffsetVisualProvider()).run(task)

        self.assertEqual(backend.motion_targets[0][:2], (0.52, -0.18))
        self.assertEqual(backend.motion_targets[3][:2], (0.58, 0.18))
        self.assertEqual(backend.applied_estimate[1], "clear")
        self.assertEqual(result.target_position, task.target_position)
        self.assertAlmostEqual(result.object_position_error_m, 0.02 * 2 ** 0.5)
        self.assertAlmostEqual(result.target_position_error_m, 0.02 * 2 ** 0.5)
        self.assertEqual(result.minimum_perception_confidence, 0.8)
        self.assertEqual(result.pose_source, "rgbd")

    def test_perception_failure_stops_before_motion(self):
        class FailedProvider:
            pose_source = "rgbd"

            @staticmethod
            def estimate(task):
                return SceneEstimate(
                    episode_id=task.episode_id,
                    pose_source="rgbd",
                    success=False,
                    failure_type="cube_not_detected",
                    latency_sec=0.1,
                )

        backend = FakeBackend()
        task = TaskSpec(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="clear",
            object_position=(0.5, -0.2, 0.025),
            target_position=(0.6, 0.2, 0.025),
        )

        result = PickPlaceRunner(backend, FailedProvider()).run(task)

        self.assertFalse(result.task_success)
        self.assertEqual(result.failure_state, "perception")
        self.assertEqual(result.failure_type, "cube_not_detected")
        self.assertFalse(result.perception_success)
        self.assertEqual(backend.motion_targets, [])


if __name__ == "__main__":
    unittest.main()
