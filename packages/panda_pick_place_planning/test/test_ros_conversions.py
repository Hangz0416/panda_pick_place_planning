import unittest
import math

from panda_pick_place_planning.core.schemas import EpisodeResult, TaskSpec
from panda_pick_place_planning.nodes.ros_conversions import (
    episode_from_ros,
    episode_to_ros,
    task_from_ros,
    task_to_ros,
)


class RosConversionsTest(unittest.TestCase):
    def test_unavailable_tracking_error_uses_nan_on_ros_boundary(self):
        result = EpisodeResult(
            episode_id=1,
            task_seed=42,
            planner="rrt_connect",
            planner_seed=10042,
            scene="clear",
            planning_success=True,
            planning_time_sec=0.1,
            explored_nodes=0,
            path_waypoints=2,
            path_length_rad=0.2,
            execution_sim_steps=0,
            task_success=False,
            collision_count=0,
            failure_type="not_executed",
            object_initial_position=(0.5, 0.2, 0.025),
            object_final_position=(0.5, 0.2, 0.025),
            target_position=(0.5, -0.2, 0.025),
            max_joint_tracking_error_rad=None,
        )

        message = episode_to_ros(result)
        restored = episode_from_ros(message)

        self.assertTrue(math.isnan(message.max_joint_tracking_error_rad))
        self.assertTrue(math.isnan(message.final_xy_error_m))
        self.assertTrue(math.isnan(message.final_z_error_m))
        self.assertTrue(math.isnan(message.final_object_speed_mps))
        self.assertEqual(message.physical_left_finger_contact, -1)
        self.assertEqual(message.physical_right_finger_contact, -1)
        self.assertEqual(message.physical_bilateral_contact, -1)
        self.assertEqual(message.physical_slip, -1)
        self.assertTrue(math.isnan(message.physical_translation_drift_m))
        self.assertIsNone(restored.max_joint_tracking_error_rad)
        self.assertIsNone(restored.final_xy_error_m)
        self.assertIsNone(restored.final_z_error_m)
        self.assertIsNone(restored.final_object_speed_mps)
        self.assertIsNone(restored.physical_left_finger_contact)
        self.assertIsNone(restored.physical_right_finger_contact)
        self.assertIsNone(restored.physical_bilateral_contact)

    def test_task_preserves_world_frame_and_simulator(self):
        task = TaskSpec(
            episode_id=1,
            task_seed=42,
            planner="rrt_connect",
            planner_seed=10042,
            scene="clear",
            object_position=(0.58, 0.2, 0.025),
            target_position=(0.58, -0.2, 0.025),
            backend="moveit",
            simulator="moveit_mock",
        )

        message = task_to_ros(task)
        restored = task_from_ros(message)

        self.assertEqual(message.header.frame_id, "world")
        self.assertEqual(message.simulator, "moveit_mock")
        self.assertEqual(restored, task)

    def test_task_rejects_non_world_frame(self):
        task = TaskSpec(
            episode_id=1,
            task_seed=42,
            planner="rrt_connect",
            planner_seed=10042,
            scene="clear",
            object_position=(0.58, 0.2, 0.025),
            target_position=(0.58, -0.2, 0.025),
            simulator="pybullet",
        )
        message = task_to_ros(task)
        message.header.frame_id = "panda_link0"

        with self.assertRaisesRegex(ValueError, "empty or 'world'"):
            task_from_ros(message)

    def test_task_accepts_empty_frame_as_world_contract(self):
        task = TaskSpec(
            episode_id=1,
            task_seed=42,
            planner="rrt_connect",
            planner_seed=10042,
            scene="clear",
            object_position=(0.58, 0.2, 0.025),
            target_position=(0.58, -0.2, 0.025),
            simulator="pybullet",
        )
        message = task_to_ros(task)
        message.header.frame_id = ""

        self.assertEqual(task_from_ros(message), task)

    def test_episode_preserves_execution_and_collision_metrics(self):
        result = EpisodeResult(
            episode_id=1,
            task_seed=42,
            planner="rrt_connect",
            planner_seed=10042,
            scene="obstacle",
            planning_success=True,
            planning_time_sec=0.2,
            explored_nodes=0,
            path_waypoints=8,
            path_length_rad=1.2,
            execution_sim_steps=0,
            task_success=False,
            collision_count=1,
            failure_type="execution_collision",
            object_initial_position=(0.58, 0.2, 0.025),
            object_final_position=(0.57, -0.18, 0.028),
            target_position=(0.58, -0.2, 0.025),
            backend="moveit",
            simulator="gazebo",
            planner_id="RRTConnectkConfigDefault",
            execution_time_sec=1.5,
            execution_success=False,
            trajectory_points=17,
            max_joint_tracking_error_rad=0.03,
            final_xy_error_m=0.022,
            final_z_error_m=0.003,
            final_object_speed_mps=0.07,
            object_attached=True,
            object_released=True,
            planning_collision=False,
            execution_collision=True,
            planning_scene_sync_success=False,
            failure_state="transfer",
            pose_source="rgbd",
            perception_success=True,
            perception_latency_sec=0.025,
            object_position_error_m=0.004,
            target_position_error_m=0.006,
            obstacle_position_error_m=0.009,
            minimum_perception_confidence=0.82,
            stage_metrics={
                "transfer": {
                    "evaluation_mode": "full_episode",
                    "planning_success": True,
                    "execution_success": False,
                    "planning_time_sec": 0.2,
                    "execution_time_sec": 1.5,
                    "path_length_rad": 1.2,
                    "path_waypoints": 17,
                    "trajectory_points": 19,
                    "failure_type": "controller_aborted",
                    "moveit_error_code": -4,
                    "max_joint_tracking_error_rad": 0.03,
                    "left_finger_contact": True,
                    "right_finger_contact": True,
                    "slipping": False,
                    "max_relative_translation_drift_m": 0.004,
                    "max_relative_rotation_drift_rad": 0.02,
                    "release_stable_time_sec": 0.15,
                    "collision_pair": ["panda_link4", "obstacle"],
                }
            },
        )

        message = episode_to_ros(result)
        restored = episode_from_ros(message)

        self.assertEqual(message.header.frame_id, "world")
        self.assertFalse(message.execution_success)
        self.assertTrue(message.execution_collision)
        self.assertEqual(message.trajectory_points, 17)
        self.assertAlmostEqual(message.final_object_speed_mps, 0.07)
        self.assertEqual(message.pose_source, "rgbd")
        self.assertTrue(message.perception_success)
        self.assertAlmostEqual(message.perception_latency_sec, 0.025)
        self.assertAlmostEqual(message.object_position_error_m, 0.004)
        self.assertEqual(len(message.stage_metrics), 1)
        self.assertTrue(message.stage_metrics[0].planning_success_known)
        self.assertTrue(message.stage_metrics[0].planning_success)
        self.assertTrue(message.stage_metrics[0].execution_success_known)
        self.assertFalse(message.stage_metrics[0].execution_success)
        self.assertEqual(message.stage_metrics[0].trajectory_points, 19)
        self.assertEqual(message.stage_metrics[0].moveit_error_code, -4)
        self.assertEqual(message.physical_left_finger_contact, 1)
        self.assertEqual(message.physical_right_finger_contact, 1)
        self.assertEqual(message.physical_bilateral_contact, 1)
        self.assertEqual(message.physical_slip, 0)
        self.assertAlmostEqual(message.physical_translation_drift_m, 0.004)
        self.assertAlmostEqual(message.physical_rotation_drift_rad, 0.02)
        self.assertAlmostEqual(message.physical_release_stable_time_sec, 0.15)
        self.assertEqual(
            list(message.last_collision_pair), ["panda_link4", "obstacle"]
        )
        self.assertFalse(restored.execution_success)
        self.assertTrue(restored.execution_collision)
        self.assertEqual(restored.trajectory_points, 17)
        self.assertAlmostEqual(restored.final_object_speed_mps, 0.07)
        self.assertEqual(restored.pose_source, "rgbd")
        self.assertAlmostEqual(restored.target_position_error_m, 0.006)
        self.assertAlmostEqual(restored.minimum_perception_confidence, 0.82)
        self.assertFalse(restored.planning_scene_sync_success)
        self.assertEqual(
            restored.stage_metrics["transfer"],
            {
                "evaluation_mode": "full_episode",
                "planning_success": True,
                "execution_success": False,
                "planning_time_sec": 0.2,
                "execution_time_sec": 1.5,
                "path_length_rad": 1.2,
                "max_joint_tracking_error_rad": 0.03,
                "path_waypoints": 17,
                "trajectory_points": 19,
                "failure_type": "controller_aborted",
                "moveit_error_code": -4,
            },
        )
        self.assertTrue(restored.physical_left_finger_contact)
        self.assertTrue(restored.physical_right_finger_contact)
        self.assertTrue(restored.physical_bilateral_contact)
        self.assertFalse(restored.physical_slip)
        self.assertAlmostEqual(restored.physical_translation_drift_m, 0.004)
        self.assertAlmostEqual(restored.physical_rotation_drift_rad, 0.02)
        self.assertAlmostEqual(restored.physical_release_stable_time_sec, 0.15)
        self.assertEqual(
            restored.last_collision_pair, ("panda_link4", "obstacle")
        )


if __name__ == "__main__":
    unittest.main()
