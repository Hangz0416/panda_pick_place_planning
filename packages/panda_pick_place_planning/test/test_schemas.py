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

    def test_reads_v1_episode_result_with_v2_defaults(self):
        payload = {
            "episode_id": 1,
            "task_seed": 2,
            "planner": "linear",
            "planner_seed": 3,
            "scene": "clear",
            "planning_success": True,
            "planning_time_sec": 0.01,
            "explored_nodes": 2,
            "path_waypoints": 2,
            "path_length_rad": 0.3,
            "execution_sim_steps": 20,
            "task_success": True,
            "collision_count": 0,
            "failure_type": "",
            "object_initial_position": [0.5, 0.1, 0.025],
            "object_final_position": [0.5, -0.1, 0.025],
            "target_position": [0.5, -0.1, 0.025],
        }

        restored = EpisodeResult.from_dict(payload)

        self.assertEqual(restored.schema_version, 1)
        self.assertEqual(restored.backend, "pybullet")
        self.assertEqual(restored.planner_id, "linear")
        self.assertIsNone(restored.final_xy_error_m)
        self.assertIsNone(restored.final_z_error_m)
        self.assertIsNone(restored.final_object_speed_mps)
        self.assertEqual(restored.pose_source, "ground_truth")
        self.assertTrue(restored.perception_success)
        self.assertIsNone(restored.perception_latency_sec)
        self.assertEqual(restored.object_variant, "cube")
        self.assertEqual(restored.perception_profile, "nominal")
        self.assertFalse(restored.grasp_selection_success)
        self.assertEqual(restored.grasp_candidate_count, 0)
        self.assertIsNone(restored.object_yaw_error_rad)

    def test_unavailable_metrics_serialize_as_json_null(self):
        result = EpisodeResult(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="clear",
            planning_success=True,
            planning_time_sec=0.01,
            explored_nodes=2,
            path_waypoints=2,
            path_length_rad=0.3,
            execution_sim_steps=0,
            task_success=False,
            collision_count=0,
            failure_type="not_executed",
            object_initial_position=(0.5, 0.1, 0.025),
            object_final_position=(0.5, 0.1, 0.025),
            target_position=(0.5, -0.1, 0.025),
            simulator="moveit_mock",
        )

        payload = json.loads(result.to_json())

        self.assertIsNone(payload["final_xy_error_m"])
        self.assertIsNone(payload["final_z_error_m"])
        self.assertIsNone(payload["final_object_speed_mps"])
        self.assertEqual(payload["schema_version"], 4)
        self.assertIsNone(payload["perception_latency_sec"])

    def test_schema_v3_preserves_visual_metrics(self):
        payload = {
            "episode_id": 1,
            "task_seed": 2,
            "planner": "rrt_connect",
            "planner_seed": 3,
            "scene": "obstacle",
            "planning_success": True,
            "planning_time_sec": 0.1,
            "explored_nodes": 1,
            "path_waypoints": 2,
            "path_length_rad": 0.3,
            "execution_sim_steps": 10,
            "task_success": True,
            "collision_count": 0,
            "failure_type": "",
            "object_initial_position": [0.5, 0.1, 0.025],
            "object_final_position": [0.5, -0.1, 0.025],
            "target_position": [0.5, -0.1, 0.025],
            "schema_version": 3,
            "pose_source": "rgbd",
            "perception_success": True,
            "perception_latency_sec": 0.025,
            "object_position_error_m": 0.004,
            "target_position_error_m": 0.006,
            "obstacle_position_error_m": 0.009,
            "minimum_perception_confidence": 0.82,
        }

        restored = EpisodeResult.from_dict(payload)

        self.assertEqual(restored.pose_source, "rgbd")
        self.assertAlmostEqual(restored.perception_latency_sec, 0.025)
        self.assertAlmostEqual(restored.obstacle_position_error_m, 0.009)
        self.assertAlmostEqual(restored.minimum_perception_confidence, 0.82)
        self.assertFalse(restored.grasp_selection_success)

    def test_schema_v4_preserves_yaw_and_grasp_selection_metrics(self):
        base = EpisodeResult(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="obstacle",
            planning_success=True,
            planning_time_sec=0.1,
            explored_nodes=3,
            path_waypoints=9,
            path_length_rad=0.7,
            execution_sim_steps=100,
            task_success=True,
            collision_count=0,
            failure_type="",
            object_initial_position=(0.5, 0.1, 0.025),
            object_final_position=(0.5, -0.1, 0.025),
            target_position=(0.5, -0.1, 0.025),
            pose_source="rgbd",
            object_variant="rect_prism",
            perception_profile="combined",
            object_yaw_error_rad=0.04,
            grasp_selection_success=True,
            grasp_selection_latency_sec=0.08,
            grasp_candidate_count=2,
            selected_grasp_yaw_rad=-0.3,
            selected_grasp_path_length_rad=1.2,
        )

        restored = EpisodeResult.from_json(base.to_json())

        self.assertEqual(restored.schema_version, 4)
        self.assertEqual(restored.object_variant, "rect_prism")
        self.assertEqual(restored.perception_profile, "combined")
        self.assertTrue(restored.grasp_selection_success)
        self.assertAlmostEqual(restored.object_yaw_error_rad, 0.04)
        self.assertEqual(restored.grasp_candidate_count, 2)
        self.assertAlmostEqual(restored.selected_grasp_yaw_rad, -0.3)

    def test_promotes_physical_and_collision_metrics_from_stages(self):
        result = EpisodeResult(
            episode_id=1,
            task_seed=2,
            planner="rrt_connect",
            planner_seed=3,
            scene="obstacle",
            planning_success=True,
            planning_time_sec=0.01,
            explored_nodes=2,
            path_waypoints=2,
            path_length_rad=0.3,
            execution_sim_steps=100,
            task_success=False,
            collision_count=1,
            failure_type="robot_environment_collision",
            object_initial_position=(0.5, 0.1, 0.025),
            object_final_position=(0.5, -0.1, 0.025),
            target_position=(0.5, -0.1, 0.025),
            stage_metrics={
                "grasp": {
                    "left_finger_contact": True,
                    "right_finger_contact": True,
                    "slipping": False,
                    "max_relative_translation_drift_m": 0.003,
                    "max_relative_rotation_drift_rad": 0.04,
                },
                "place": {
                    "release_stable_time_sec": 0.2,
                    "collision_pair": ["panda_hand", "obstacle"],
                },
            },
        )

        self.assertTrue(result.physical_left_finger_contact)
        self.assertTrue(result.physical_right_finger_contact)
        self.assertTrue(result.physical_bilateral_contact)
        self.assertFalse(result.physical_slip)
        self.assertAlmostEqual(result.physical_translation_drift_m, 0.003)
        self.assertAlmostEqual(result.physical_rotation_drift_rad, 0.04)
        self.assertAlmostEqual(result.physical_release_stable_time_sec, 0.2)
        self.assertEqual(result.last_collision_pair, ("panda_hand", "obstacle"))


if __name__ == "__main__":
    unittest.main()
