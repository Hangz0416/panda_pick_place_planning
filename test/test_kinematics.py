import unittest

from panda_pick_place_planning.core.kinematics import PandaIKSolver


def joint_info(joint_type, lower=-1.0, upper=1.0, parent=-1):
    info = [0] * 17
    info[2] = joint_type
    info[8] = lower
    info[9] = upper
    info[16] = parent
    return tuple(info)


class FakeBullet:
    JOINT_FIXED = 4

    def __init__(self):
        self.states = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.02}
        self.last_ik_kwargs = {}

    def getNumJoints(self, robot_id, physicsClientId):
        return 4

    def getJointInfo(self, robot_id, joint_index, physicsClientId):
        infos = {
            0: joint_info(0, -2.0, 2.0),
            1: joint_info(0, -2.0, 2.0, parent=0),
            2: joint_info(self.JOINT_FIXED, parent=1),
            3: joint_info(1, 0.0, 0.04, parent=2),
        }
        return infos[joint_index]

    def getJointState(self, robot_id, joint_index, physicsClientId):
        return (self.states[joint_index], 0.0, 0.0, 0.0)

    def resetJointState(self, robot_id, joint_index, value, physicsClientId):
        self.states[joint_index] = value

    def calculateInverseKinematics(self, *args, **kwargs):
        self.last_ik_kwargs = kwargs
        return (0.1, 0.2, 0.02)

    def getLinkState(self, *args, **kwargs):
        return (None, None, None, None, (0.5, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))


class PandaIKSolverTest(unittest.TestCase):
    def test_ik_parameters_cover_every_movable_joint(self):
        bullet = FakeBullet()
        solver = PandaIKSolver(
            bullet=bullet,
            physics_client_id=0,
            robot_id=1,
            arm_joint_indices=[0, 1],
            end_effector_link=2,
            joint_limits=[(-2.0, 2.0), (-2.0, 2.0)],
        )

        solution = solver.solve(
            target_position=(0.5, 0.0, 0.3),
            target_orientation=(0.0, 0.0, 0.0, 1.0),
            rest_poses=[0.0, 0.0],
        )

        self.assertTrue(solution.success)
        self.assertEqual(solution.joints, [0.1, 0.2])
        self.assertEqual(len(bullet.last_ik_kwargs["jointDamping"]), 3)
        self.assertEqual(len(bullet.last_ik_kwargs["lowerLimits"]), 3)
        self.assertEqual(len(bullet.last_ik_kwargs["restPoses"]), 3)


if __name__ == "__main__":
    unittest.main()
