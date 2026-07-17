import unittest
import math

try:
    import pybullet  # noqa: F401

    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False

from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv
from panda_pick_place_planning.core.pick_place_fsm import PickPlaceRunner
from panda_pick_place_planning.core.tasks import generate_task


@unittest.skipUnless(PYBULLET_AVAILABLE, "PyBullet 未安装，跳过仿真冒烟测试。")
class PyBulletSmokeTest(unittest.TestCase):
    def test_table_collision_uses_shared_scene_geometry(self):
        env = PyBulletPandaEnv(use_gui=False)
        try:
            env.connect()
            lower, upper = env.bullet.getAABB(
                env.table_id,
                physicsClientId=env.physics_client_id,
            )

            expected_lower = tuple(
                center - 0.5 * size
                for center, size in zip(
                    env.config.table_position,
                    env.config.table_size,
                )
            )
            expected_upper = tuple(
                center + 0.5 * size
                for center, size in zip(
                    env.config.table_position,
                    env.config.table_size,
                )
            )
            for actual, expected in zip(lower, expected_lower):
                self.assertAlmostEqual(actual, expected)
            for actual, expected in zip(upper, expected_upper):
                self.assertAlmostEqual(actual, expected)
        finally:
            env.close()

    def test_clear_scene_reset_and_pregrasp_ik(self):
        task = generate_task(0, 42, "linear", "clear")
        env = PyBulletPandaEnv(use_gui=False)
        try:
            env.reset(task)
            orientation = env.downward_orientation()
            target = (
                task.object_position[0],
                task.object_position[1],
                env.config.hover_height,
            )

            solution = env.solve_ik(target, orientation)

            self.assertTrue(solution.success, solution.failure_type)
            self.assertIsNotNone(env.collision_checker)
            current_joints = env.current_arm_joints()
            is_valid = env.collision_checker.is_valid(current_joints)
            collisions = env.collision_checker.current_collisions()
            self.assertTrue(is_valid, f"unexpected home collisions: {collisions}")
        finally:
            env.close()

    def test_reset_replaces_previous_episode_motor_targets(self):
        first_task = generate_task(0, 42, "linear", "clear")
        second_task = generate_task(1, 43, "linear", "clear")
        env = PyBulletPandaEnv(use_gui=False)
        try:
            first_result = PickPlaceRunner(env).run(first_task)
            self.assertTrue(first_result.task_success, first_result.failure_type)

            env.reset(second_task)

            reset_error = math.dist(env.current_arm_joints(), env.config.home_joints)
            self.assertLess(reset_error, 0.02)
        finally:
            env.close()

    def test_carried_cube_pose_is_checked_against_obstacle(self):
        task = generate_task(0, 42, "rrt_connect", "obstacle")
        env = PyBulletPandaEnv(use_gui=False)
        try:
            env.reset(task)
            checker = env.collision_checker
            self.assertIsNotNone(checker)
            ee_position, ee_orientation = env.current_end_effector_pose()
            inverse_position, inverse_orientation = env.bullet.invertTransform(
                ee_position,
                ee_orientation,
            )
            relative_position, relative_orientation = env.bullet.multiplyTransforms(
                inverse_position,
                inverse_orientation,
                env.config.obstacle_position,
                (0.0, 0.0, 0.0, 1.0),
            )
            checker.attach_carried_body(
                env.cube_id,
                11,
                relative_position,
                relative_orientation,
                [env.obstacle_id],
                clearance_margins={
                    env.obstacle_id: env.config.carried_object_clearance_margin,
                },
            )

            self.assertFalse(checker.is_valid(env.current_arm_joints()))
            self.assertEqual(
                checker.last_validation_failure,
                "carried_object_collision",
            )
        finally:
            env.close()

    def test_execution_reports_carried_cube_collision_separately(self):
        task = generate_task(0, 42, "rrt_connect", "obstacle")
        env = PyBulletPandaEnv(use_gui=False)
        try:
            env.reset(task)
            ee_position, ee_orientation = env.current_end_effector_pose()
            env.bullet.resetBasePositionAndOrientation(
                env.cube_id,
                ee_position,
                ee_orientation,
                physicsClientId=env.physics_client_id,
            )
            self.assertTrue(env.attach_object())
            env.bullet.resetBasePositionAndOrientation(
                env.obstacle_id,
                ee_position,
                (0.0, 0.0, 0.0, 1.0),
                physicsClientId=env.physics_client_id,
            )
            joints = env.current_arm_joints()

            report = env.execute_joint_path([joints, joints])

            self.assertFalse(report.success)
            self.assertEqual(
                report.failure_type,
                "execution_carried_object_collision",
            )
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
