import unittest

from panda_pick_place_planning.core.collision_checker import (
    PandaCollisionChecker,
)


def joint_info(parent_link):
    info = [0] * 17
    info[16] = parent_link
    return tuple(info)


def contact(first_link, second_link):
    values = [0] * 5
    values[3] = first_link
    values[4] = second_link
    return tuple(values)


class FakeBullet:
    def __init__(self):
        self.states = {joint_index: 0.0 for joint_index in range(12)}
        self.contacts = {
            (1, 1): [
                contact(0, 1),
                contact(8, 10),
                contact(1, 4),
            ],
            (1, 2): [contact(-1, -1), contact(0, -1)],
            (1, 3): [contact(3, -1)],
        }

    def getNumJoints(self, robot_id, physicsClientId):
        return 12

    def getJointInfo(self, robot_id, joint_index, physicsClientId):
        return joint_info(joint_index - 1)

    def getContactPoints(self, bodyA, bodyB, physicsClientId):
        return self.contacts.get((bodyA, bodyB), [])

    def getJointState(self, robot_id, joint_index, physicsClientId):
        return (self.states[joint_index], 0.0, 0.0, 0.0)

    def resetJointState(
        self,
        robot_id,
        joint_index,
        value,
        physicsClientId,
    ):
        self.states[joint_index] = value

    def performCollisionDetection(self, physicsClientId):
        return None


class PandaCollisionCheckerTest(unittest.TestCase):
    def test_filters_expected_contacts_and_keeps_real_collisions(self):
        checker = PandaCollisionChecker(
            bullet=FakeBullet(),
            physics_client_id=0,
            robot_id=1,
            arm_joint_indices=[0, 1],
            joint_limits=[(-1.0, 1.0), (-1.0, 1.0)],
            world_body_ids=[2, 3],
            allowed_world_contacts={2: {-1, 0}},
        )

        self.assertEqual(
            checker.current_collisions(),
            [
                ("self", 1, 1, 4),
                ("world", 3, 3, -1),
            ],
        )
        self.assertFalse(checker.is_valid([0.0, 0.0]))

    def test_accepts_only_expected_home_contacts(self):
        bullet = FakeBullet()
        bullet.contacts[(1, 1)] = [contact(0, 1), contact(8, 10)]
        bullet.contacts[(1, 3)] = []
        checker = PandaCollisionChecker(
            bullet=bullet,
            physics_client_id=0,
            robot_id=1,
            arm_joint_indices=[0, 1],
            joint_limits=[(-1.0, 1.0), (-1.0, 1.0)],
            world_body_ids=[2, 3],
            allowed_world_contacts={2: {-1, 0}},
        )

        self.assertTrue(checker.is_valid([0.0, 0.0]))


if __name__ == "__main__":
    unittest.main()
