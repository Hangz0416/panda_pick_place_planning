import unittest

try:
    import pybullet  # noqa: F401

    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False

from panda_pick_place_planning.core.pybullet_env import PyBulletPandaEnv
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
            self.assertTrue(env.collision_checker.is_valid(env.current_arm_joints()))
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
