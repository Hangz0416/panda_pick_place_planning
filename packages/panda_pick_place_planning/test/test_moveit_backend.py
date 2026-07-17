from types import SimpleNamespace
import math
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, PoseArray
from panda_pick_place_interfaces.action import PlanExecuteMotion
from panda_pick_place_interfaces.srv import GetPhysicalGraspState
from panda_pick_place_interfaces.srv import GetExecutionCollisionState

from panda_pick_place_planning.core.backends import (
    BackendMotionReport,
    ManipulationBackend,
    MoveItBackend,
    MoveItBackendConfig,
    MoveItClients,
)
from panda_pick_place_planning.core.planning_regression import (
    PlanningRegressionRunner,
)
from panda_pick_place_planning.core.schemas import PlanResult, TaskSpec
from panda_pick_place_planning.nodes.manipulation_node import runtime_error_result


class ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


class DelayedFuture:
    def __init__(self):
        self.value = None
        self.callbacks = []
        self.completed = False

    def done(self):
        return self.completed

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def complete(self, value):
        self.value = value
        self.completed = True
        for callback in self.callbacks:
            callback(self)


class FakeGoalHandle:
    def __init__(self, result, accepted=True):
        self.accepted = accepted
        self._result = result
        self.cancelled = False

    def get_result_async(self):
        wrapped = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=self._result,
        )
        return ImmediateFuture(wrapped)

    def cancel_goal_async(self):
        self.cancelled = True
        return ImmediateFuture(None)


class FakeActionClient:
    def __init__(self, result_factory):
        self.result_factory = result_factory
        self.goals = []
        self.available = True

    def wait_for_server(self, timeout_sec):
        del timeout_sec
        return self.available

    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        if feedback_callback is not None:
            feedback_callback(SimpleNamespace(feedback=SimpleNamespace(phase="planning")))
        return ImmediateFuture(FakeGoalHandle(self.result_factory()))


class FakeServiceClient:
    def __init__(self, successes=True):
        if isinstance(successes, bool):
            successes = [successes]
        self.successes = list(successes)
        self.requests = []
        self.available = True

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        index = min(len(self.requests) - 1, len(self.successes) - 1)
        response = SimpleNamespace(
            success=self.successes[index], message="fake"
        )
        if hasattr(request, "object_position"):
            response.applied_object_position = SimpleNamespace(
                x=request.object_position.x,
                y=request.object_position.y,
                z=request.object_position.z,
            )
            response.applied_target_position = SimpleNamespace(
                x=request.target_position.x,
                y=request.target_position.y,
                z=request.target_position.z,
            )
        return ImmediateFuture(
            response
        )


class MismatchedResetClient(FakeServiceClient):
    def call_async(self, request):
        future = super().call_async(request)
        future.value.applied_object_position.x += 0.1
        return future


class FakePhysicalServiceClient:
    def __init__(self, responses, available=True):
        self.responses = list(responses)
        self.requests = []
        self.available = available

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        response = self.responses.pop(0) if len(self.responses) > 1 else self.responses[0]
        return ImmediateFuture(response)


def motion_result():
    result = PlanExecuteMotion.Result()
    result.success = True
    result.moveit_error_code = 1
    result.planning_time_sec = 0.125
    result.path_length_rad = 0.75
    result.trajectory_points = 9
    result.execution_time_sec = 0.4
    return result


def gripper_result():
    return SimpleNamespace(reached_goal=True, stalled=False)


def physical_state(
    *,
    holding=False,
    released=False,
    slipping=False,
    grasped=None,
    position=(0.59, 0.22, 0.025),
    speed=0.0,
    stable_time=0.0,
):
    response = GetPhysicalGraspState.Response()
    response.success = True
    response.episode_id = 1
    response.grasped = holding if grasped is None else grasped
    response.holding_object = holding
    response.released = released
    response.left_finger_contact = holding
    response.right_finger_contact = holding
    response.slipping = slipping
    response.object_linear_speed_mps = speed
    response.release_stable_time_sec = stable_time
    response.object_pose.position.x = position[0]
    response.object_pose.position.y = position[1]
    response.object_pose.position.z = position[2]
    response.object_pose.orientation.w = 1.0
    return response


def collision_state(
    *,
    available=True,
    invalid=False,
    count=0,
    failure_type="",
    episode_id=1,
):
    response = GetExecutionCollisionState.Response()
    response.success = True
    response.episode_id = episode_id
    response.available = available
    response.invalid_collision = invalid
    response.collision_count = count
    response.last_failure_type = failure_type
    response.last_collision1 = "panda_link4" if invalid else ""
    response.last_collision2 = "obstacle" if invalid else ""
    return response


def pose_message(stamp_sec, position):
    message = PoseArray()
    if stamp_sec is not None:
        whole_seconds = int(stamp_sec)
        message.header.stamp.sec = whole_seconds
        message.header.stamp.nanosec = int(
            round((stamp_sec - whole_seconds) * 1_000_000_000)
        )
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    message.poses = [pose]
    return message


def task(scene="obstacle", grasp_mode="assisted"):
    return TaskSpec(
        episode_id=1,
        task_seed=42,
        planner="rrt_connect",
        planner_seed=10042,
        scene=scene,
        object_position=(0.59, 0.22, 0.025),
        target_position=(0.59, -0.225, 0.025),
        backend="moveit",
        grasp_mode=grasp_mode,
    )


def clients(
    *,
    attached=True,
    reset=True,
    gazebo_attach=True,
    gazebo_detach=True,
    gripper=None,
    physical_grasp=None,
    execution_collision=None,
):
    if execution_collision is None:
        execution_collision = FakePhysicalServiceClient([collision_state()])
    return MoveItClients(
        motion=FakeActionClient(motion_result),
        upsert=FakeServiceClient(True),
        remove=FakeServiceClient(True),
        attached=FakeServiceClient(attached),
        reset=FakeServiceClient(reset),
        gazebo_attach=FakeServiceClient(gazebo_attach),
        gazebo_detach=FakeServiceClient(gazebo_detach),
        gripper=gripper,
        physical_grasp=physical_grasp,
        execution_collision=execution_collision,
    )


class MoveItBackendTest(unittest.TestCase):
    def test_default_gripper_action_matches_shared_controller_contract(self):
        self.assertEqual(
            MoveItBackendConfig().gripper_action_name,
            "/panda_hand_controller/gripper_cmd",
        )

    def test_pose_target_and_attachment_use_distinct_links(self):
        fake_clients = clients()
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                execute_motion=False,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )

        backend.reset(task(scene="clear"))
        backend.move_to_pose(
            (0.5, 0.0, 0.2),
            (1.0, 0.0, 0.0, 0.0),
            planner="rrt_connect",
        )
        self.assertEqual(
            fake_clients.motion.goals[-1].end_effector_link,
            "panda_grasptarget",
        )

        self.assertTrue(backend.attach_object())
        self.assertEqual(fake_clients.attached.requests[-1].link_name, "panda_hand")

    def test_mock_backend_reports_independent_planning_without_fake_task(self):
        fake_clients = clients()
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                execute_motion=False,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )
        self.assertIsInstance(backend, ManipulationBackend)

        result = PlanningRegressionRunner(backend).run(task())

        self.assertTrue(result.planning_success)
        self.assertFalse(result.task_success)
        self.assertFalse(result.execution_success)
        self.assertEqual(result.failure_type, "not_executed")
        self.assertEqual(result.object_initial_position, result.object_final_position)
        self.assertFalse(result.object_attached)
        self.assertFalse(result.object_released)
        self.assertEqual(result.backend, "moveit")
        self.assertEqual(result.simulator, "moveit_mock")
        self.assertEqual(result.planner_id, "RRTConnectkConfigDefault")
        self.assertEqual(result.path_waypoints, 36)
        self.assertAlmostEqual(result.path_length_rad, 3.0)
        self.assertIsNone(result.final_xy_error_m)
        self.assertIsNone(result.final_z_error_m)
        self.assertIsNone(result.final_object_speed_mps)
        self.assertEqual(
            result.stage_metrics["transfer_from_current"]["evaluation_mode"],
            "independent_planning_only",
        )
        self.assertEqual(fake_clients.motion.goals[0].target_type, 1)
        self.assertEqual(fake_clients.motion.goals[0].named_target, "home")
        transfer_goal = fake_clients.motion.goals[2]
        self.assertEqual(transfer_goal.target_type, 2)
        self.assertEqual(transfer_goal.planner_id, "RRTConnectkConfigDefault")
        self.assertFalse(transfer_goal.execute)
        self.assertEqual(len(fake_clients.motion.goals), 4)
        self.assertFalse(any(request.attach for request in fake_clients.attached.requests))
        object_ids = [
            request.collision_object.id for request in fake_clients.upsert.requests
        ]
        self.assertIn("table", object_ids)
        self.assertIn("cube", object_ids)
        self.assertIn("obstacle", object_ids)
        table_request = next(
            request
            for request in fake_clients.upsert.requests
            if request.collision_object.id == "table"
        )
        self.assertEqual(
            list(table_request.allowed_collision_links), ["panda_link0"]
        )
        self.assertEqual(
            list(table_request.collision_object.primitives[0].dimensions),
            [1.5, 1.0, 0.05],
        )
        table_pose = table_request.collision_object.primitive_poses[0]
        self.assertEqual(
            (table_pose.position.x, table_pose.position.y, table_pose.position.z),
            (0.5, 0.0, -0.025),
        )
        cube_request = next(
            request
            for request in fake_clients.upsert.requests
            if request.collision_object.id == "cube"
        )
        self.assertEqual(
            list(cube_request.allowed_collision_links),
            ["panda_hand", "panda_leftfinger", "panda_rightfinger"],
        )

    def test_reset_cleanup_failures_are_required_and_classified(self):
        cases = (
            (
                "detach",
                clients(attached=False),
                "planning_scene_reset_detach_failed",
            ),
            (
                "cube_remove",
                clients(),
                "planning_scene_reset_cube_remove_failed",
            ),
            (
                "obstacle_remove",
                clients(),
                "planning_scene_reset_obstacle_remove_failed",
            ),
        )
        cases[1][1].remove = FakeServiceClient(False)
        cases[2][1].remove = FakeServiceClient([True, False])

        for label, fake_clients, failure_type in cases:
            with self.subTest(label=label):
                backend = MoveItBackend(
                    object(),
                    MoveItBackendConfig(
                        execute_motion=False,
                        command_gripper=False,
                        service_timeout_sec=0.05,
                    ),
                    clients=fake_clients,
                )

                with self.assertRaisesRegex(RuntimeError, failure_type):
                    backend.reset(task(scene="clear"))
                self.assertFalse(backend.planning_scene_sync_success)

        self.assertEqual(
            [request.object_id for request in cases[2][1].remove.requests],
            ["cube", "obstacle"],
        )

    def test_assisted_attach_rolls_back_gazebo_if_scene_sync_fails(self):
        fake_clients = clients(attached=[True, False])
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )
        backend.reset(task(scene="clear"))
        reset_request = fake_clients.reset.requests[0]
        self.assertEqual(reset_request.header.frame_id, "world")
        self.assertEqual(reset_request.task_seed, 42)
        self.assertEqual(reset_request.scene, "clear")

        attached = backend.attach_object()

        self.assertFalse(attached)
        self.assertFalse(backend.planning_scene_sync_success)
        self.assertEqual(len(fake_clients.gazebo_attach.requests), 1)
        self.assertEqual(len(fake_clients.gazebo_detach.requests), 1)

    def test_assisted_grasp_holds_clearance_until_gazebo_attach(self):
        gripper = FakeActionClient(gripper_result)
        fake_clients = clients(gripper=gripper)
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=True,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )
        backend.reset(task(scene="clear"))

        backend.set_gripper(backend.closed_gripper_width)
        self.assertAlmostEqual(gripper.goals[-1].command.position, 0.03)
        self.assertTrue(backend.attach_object())
        self.assertAlmostEqual(gripper.goals[-1].command.position, 0.0)

    def test_physics_mode_does_not_fake_moveit_attachment(self):
        fake_clients = clients()
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                command_gripper=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )
        backend.reset(task(scene="clear", grasp_mode="physics"))

        self.assertFalse(backend.attach_object())
        self.assertEqual(
            backend.last_failure_type,
            "physical_grasp_state_unavailable",
        )
        # The only attachment request is reset-time cleanup (attach=False).
        self.assertEqual(len(fake_clients.attached.requests), 1)
        self.assertFalse(fake_clients.attached.requests[0].attach)

    def test_physics_attach_requires_monitor_hold_then_only_attaches_moveit(self):
        monitor = FakePhysicalServiceClient(
            [
                physical_state(),
                physical_state(),
                physical_state(holding=True),
                physical_state(released=True, grasped=True),
                physical_state(released=True, grasped=True),
                physical_state(
                    released=True,
                    grasped=True,
                    position=(0.59, -0.225, 0.025),
                    stable_time=0.2,
                ),
            ]
        )
        fake_clients = clients(physical_grasp=monitor)
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                collision_monitor_available=True,
                service_timeout_sec=0.05,
                settle_timeout_sec=0.1,
            ),
            clients=fake_clients,
        )
        physics_task = task(scene="clear", grasp_mode="physics")
        backend.reset(physics_task)

        self.assertTrue(backend.attach_object())
        self.assertEqual(len(fake_clients.gazebo_attach.requests), 0)
        self.assertTrue(fake_clients.attached.requests[-1].attach)
        backend.detach_object()
        backend.wait_for_object_settle()
        self.assertEqual(len(fake_clients.gazebo_detach.requests), 0)
        self.assertTrue(backend.object_released_confirmed)
        self.assertEqual(backend.object_state()[0], physics_task.target_position)
        self.assertTrue(backend.grasp_metrics()["released"])
        commands = [request.command for request in monitor.requests]
        self.assertEqual(commands[0], GetPhysicalGraspState.Request.COMMAND_RESET)
        self.assertGreaterEqual(
            commands.count(GetPhysicalGraspState.Request.COMMAND_QUERY),
            5,
        )

    def test_physics_slip_is_a_typed_grasp_failure(self):
        monitor = FakePhysicalServiceClient(
            [
                physical_state(),
                physical_state(),
                physical_state(holding=True, slipping=True),
            ]
        )
        fake_clients = clients(physical_grasp=monitor)
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )
        backend.reset(task(scene="clear", grasp_mode="physics"))

        self.assertFalse(backend.attach_object())
        self.assertEqual(backend.last_failure_type, "physical_grasp_slip")
        self.assertFalse(fake_clients.attached.requests[-1].attach)

    def test_physics_grasp_loss_after_motion_is_latched(self):
        monitor = FakePhysicalServiceClient(
            [
                physical_state(),
                physical_state(),
                physical_state(holding=True),
                physical_state(released=True, grasped=True),
                physical_state(
                    released=True,
                    grasped=True,
                    stable_time=0.2,
                ),
            ]
        )
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=clients(physical_grasp=monitor),
        )
        backend.reset(task(scene="clear", grasp_mode="physics"))
        self.assertTrue(backend.attach_object())

        report = backend.move_to_pose(
            (0.5, 0.0, 0.2),
            (1.0, 0.0, 0.0, 0.0),
        )
        backend._wait_for_physical_release()

        self.assertFalse(report.success)
        self.assertEqual(report.failure_type, "grasp_lost")
        self.assertEqual(backend.physical_grasp_failure_type, "grasp_lost")
        self.assertEqual(backend.last_failure_type, "grasp_lost")
        self.assertEqual(
            backend.grasp_metrics()["latched_failure_type"],
            "grasp_lost",
        )

    def test_physics_slip_after_motion_is_latched(self):
        monitor = FakePhysicalServiceClient(
            [
                physical_state(),
                physical_state(),
                physical_state(holding=True),
                physical_state(holding=True, slipping=True),
            ]
        )
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=clients(physical_grasp=monitor),
        )
        backend.reset(task(scene="clear", grasp_mode="physics"))
        self.assertTrue(backend.attach_object())

        report = backend.move_to_pose(
            (0.5, 0.0, 0.2),
            (1.0, 0.0, 0.0, 0.0),
        )

        self.assertFalse(report.success)
        self.assertEqual(report.failure_type, "physical_grasp_slip")
        self.assertEqual(
            backend.physical_grasp_failure_type,
            "physical_grasp_slip",
        )

    def test_gazebo_requires_execution(self):
        with self.assertRaisesRegex(ValueError, "cannot disable"):
            MoveItBackendConfig(use_gazebo=True, execute_motion=False)

    def test_collision_monitor_service_drives_execution_failure(self):
        collision_monitor = FakePhysicalServiceClient(
            [
                collision_state(),
                collision_state(),
                collision_state(
                    invalid=True,
                    count=1,
                    failure_type="robot_environment_collision",
                ),
            ]
        )
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=clients(execution_collision=collision_monitor),
        )
        backend.reset(task(scene="clear"))

        report = backend.move_to_pose(
            (0.5, 0.0, 0.2),
            (1.0, 0.0, 0.0, 0.0),
        )

        self.assertFalse(report.success)
        self.assertEqual(report.failure_type, "robot_environment_collision")
        self.assertEqual(report.collision_count, 1)
        self.assertTrue(backend.collision_state_known)
        self.assertTrue(backend.invalid_collision_detected)
        self.assertEqual(backend.collision_count, 1)
        self.assertEqual(
            collision_monitor.requests[0].command,
            GetExecutionCollisionState.Request.COMMAND_RESET,
        )
        self.assertEqual(collision_monitor.requests[0].episode_id, 1)

    def test_deprecated_boolean_cannot_fake_collision_availability(self):
        collision_monitor = FakePhysicalServiceClient(
            [collision_state(available=False)]
        )
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                collision_monitor_available=True,
                service_timeout_sec=0.05,
            ),
            clients=clients(execution_collision=collision_monitor),
        )

        backend.reset(task(scene="clear"))

        self.assertFalse(backend.collision_state_known)
        self.assertFalse(backend.refresh_execution_state())

    def test_reset_rejects_applied_pose_mismatch(self):
        fake_clients = clients()
        fake_clients.reset = MismatchedResetClient(True)
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=fake_clients,
        )

        with self.assertRaisesRegex(RuntimeError, "reset_response_mismatch"):
            backend.reset(task(scene="clear"))
        self.assertFalse(backend.planning_scene_sync_success)

    def test_future_wait_allows_another_executor_thread_to_complete_callback(self):
        future = DelayedFuture()

        worker = threading.Thread(
            target=lambda: (time.sleep(0.01), future.complete("done"))
        )
        worker.start()
        result = MoveItBackend._wait_future(future, 0.2)
        worker.join()

        self.assertEqual(result, "done")

    def test_controller_error_accumulates_per_motion_and_episode(self):
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(command_gripper=False),
            clients=clients(),
        )
        self.assertIsNone(backend.max_joint_tracking_error_rad)
        backend._begin_motion_tracking(enabled=True)
        state = JointTrajectoryControllerState()
        state.error.positions = [0.01, -0.03, float("nan")]
        backend._on_controller_state(state)
        report = backend._finish_motion_tracking(
            BackendMotionReport(success=True, planning_success=True)
        )

        self.assertAlmostEqual(report.max_joint_tracking_error_rad, 0.03)
        self.assertAlmostEqual(backend.max_joint_tracking_error_rad, 0.03)
        state.error.positions = [0.005, -0.01]
        backend._on_controller_state(state)
        self.assertAlmostEqual(backend.max_joint_tracking_error_rad, 0.03)

    def test_missing_controller_state_stays_unavailable(self):
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(command_gripper=False),
            clients=clients(),
        )
        backend._begin_motion_tracking(enabled=True)
        report = backend._finish_motion_tracking(
            BackendMotionReport(success=True, planning_success=True)
        )

        self.assertIsNone(report.max_joint_tracking_error_rad)
        self.assertIsNone(backend.max_joint_tracking_error_rad)

    def test_cube_velocity_uses_sim_stamp_and_rejects_unstamped_samples(self):
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
            ),
            clients=clients(),
        )
        backend._on_cube_pose(pose_message(10.0, (0.5, 0.0, 0.025)))
        self.assertFalse(backend.object_state_known)
        self.assertTrue(math.isnan(backend.object_state()[1][0]))

        backend._on_cube_pose(pose_message(12.0, (0.6, 0.0, 0.025)))
        self.assertTrue(backend.object_state_known)
        self.assertAlmostEqual(backend.object_state()[1][0], 0.05)

        backend._on_cube_pose(pose_message(None, (0.7, 0.0, 0.025)))
        self.assertFalse(backend.object_state_known)
        self.assertTrue(math.isnan(backend.object_state()[1][0]))
        self.assertFalse(
            backend.object_is_stable_in_target((0.7, 0.0, 0.025))
        )

        backend._on_cube_pose(pose_message(11.0, (0.8, 0.0, 0.025)))
        self.assertFalse(backend.object_state_known)
        self.assertIsNotNone(backend._last_object_sample)
        backend._on_cube_pose(pose_message(10.0, (0.9, 0.0, 0.025)))
        self.assertFalse(backend.object_state_known)
        self.assertIsNone(backend._last_object_sample)
        backend._on_cube_pose(pose_message(13.0, (1.0, 0.0, 0.025)))
        self.assertFalse(backend.object_state_known)
        backend._on_cube_pose(pose_message(14.0, (1.1, 0.0, 0.025)))
        self.assertTrue(backend.object_state_known)
        self.assertAlmostEqual(backend.object_state()[1][0], 0.1)

    def test_gazebo_reset_clears_pre_teleport_velocity_sample(self):
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
                verify_gazebo_reset_pose=False,
                service_timeout_sec=0.05,
            ),
            clients=clients(),
        )
        backend._on_cube_pose(pose_message(1.0, (0.5, 0.0, 0.025)))
        backend._on_cube_pose(pose_message(2.0, (0.6, 0.0, 0.025)))
        self.assertTrue(backend.object_state_known)

        backend.reset(task(scene="clear"))

        self.assertIsNone(backend._last_object_sample)
        self.assertFalse(backend._object_velocity_known)
        self.assertTrue(math.isnan(backend.object_state()[1][0]))

    def test_runtime_boundary_preserves_known_backend_metrics(self):
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(command_gripper=False),
            clients=clients(),
        )
        backend._object_position = (0.59, -0.225, 0.025)
        backend._object_velocity = (0.03, 0.0, 0.0)
        backend._object_velocity_known = True
        backend.object_state_known = True
        backend.simulation_steps = 18
        backend.collision_count = 2
        backend.invalid_collision_detected = True
        backend.planning_scene_sync_success = False
        backend._ever_attached = True
        backend._ever_released = True
        backend.execution_collision_metrics = {
            "available": True,
            "collision_count": 2,
            "collision_pair": ["panda_link4", "obstacle"],
            "failure_type": "robot_environment_collision",
        }
        backend.physical_grasp_metrics = {
            "holding_object": False,
            "left_finger_contact": True,
            "right_finger_contact": True,
            "slipping": True,
            "max_relative_translation_drift_m": 0.012,
            "max_relative_rotation_drift_rad": 0.08,
            "release_stable_time_sec": 0.4,
            "failure_type": "physical_grasp_slip",
        }
        backend.last_motion_report = BackendMotionReport(
            success=False,
            planning_success=True,
            plan_result=PlanResult(
                True,
                planning_time_sec=0.12,
                explored_nodes=4,
            ),
            execution_time_sec=0.8,
            path_length_rad=0.7,
            trajectory_points=11,
            moveit_error_code=-4,
            max_joint_tracking_error_rad=0.02,
        )
        with backend._tracking_lock:
            backend._episode_tracking_max = 0.02

        result = runtime_error_result(
            task(scene="clear"),
            backend,
            failure_type="robot_environment_collision",
            failure_state="transfer",
        )

        self.assertTrue(result.planning_success)
        self.assertAlmostEqual(result.planning_time_sec, 0.12)
        self.assertAlmostEqual(result.path_length_rad, 0.7)
        self.assertEqual(result.trajectory_points, 11)
        self.assertEqual(result.execution_sim_steps, 18)
        self.assertEqual(result.collision_count, 2)
        self.assertTrue(result.execution_collision)
        self.assertTrue(result.object_attached)
        self.assertTrue(result.object_released)
        self.assertAlmostEqual(result.final_object_speed_mps, 0.03)
        self.assertAlmostEqual(result.max_joint_tracking_error_rad, 0.02)
        self.assertFalse(result.planning_scene_sync_success)
        self.assertEqual(result.failure_state, "transfer")
        runtime_metrics = result.stage_metrics["runtime_boundary"]
        self.assertEqual(
            runtime_metrics["collision_failure_type"],
            "robot_environment_collision",
        )
        self.assertEqual(
            runtime_metrics["physical_failure_type"],
            "physical_grasp_slip",
        )
        self.assertTrue(result.physical_left_finger_contact)
        self.assertTrue(result.physical_right_finger_contact)
        self.assertTrue(result.physical_bilateral_contact)
        self.assertTrue(result.physical_slip)
        self.assertAlmostEqual(result.physical_translation_drift_m, 0.012)
        self.assertAlmostEqual(result.physical_rotation_drift_rad, 0.08)
        self.assertAlmostEqual(result.physical_release_stable_time_sec, 0.4)
        self.assertEqual(
            result.last_collision_pair,
            ("panda_link4", "obstacle"),
        )

    def test_runtime_boundary_keeps_unknown_object_metrics_unavailable(self):
        backend = MoveItBackend(
            object(),
            MoveItBackendConfig(
                use_gazebo=True,
                execute_motion=True,
                command_gripper=False,
            ),
            clients=clients(),
        )

        result = runtime_error_result(task(scene="clear"), backend)

        self.assertIsNone(result.final_xy_error_m)
        self.assertIsNone(result.final_z_error_m)
        self.assertIsNone(result.final_object_speed_mps)


if __name__ == "__main__":
    unittest.main()
