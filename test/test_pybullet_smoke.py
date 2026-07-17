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


if __name__ == "__main__":
    unittest.main()
