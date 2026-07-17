from panda_pick_place_interfaces.msg import SceneObjectEstimate as EstimateMsg
from panda_pick_place_interfaces.srv import EstimateScene

from panda_pick_place_planning.core.scene_pose_provider import (
    GroundTruthPoseProvider,
    RGBDScenePoseProvider,
)
from panda_pick_place_planning.core.schemas import TaskSpec


class ImmediateFuture:
    def __init__(self, value):
        self.value = value

    def done(self):
        return True

    def result(self):
        return self.value


class FakeClient:
    def __init__(self, response=None, available=True):
        self.response = response
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        assert timeout_sec > 0.0
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return ImmediateFuture(self.response)


def task(scene="obstacle"):
    return TaskSpec(
        episode_id=7,
        task_seed=42,
        planner="rrt_connect",
        planner_seed=10042,
        scene=scene,
        object_position=(0.57, -0.17, 0.025),
        target_position=(0.57, 0.17, 0.025),
        backend="moveit",
        simulator="gazebo",
    )


def estimate_message(object_id, position, confidence=0.9):
    message = EstimateMsg()
    message.object_id = object_id
    message.pose.pose.position.x = position[0]
    message.pose.pose.position.y = position[1]
    message.pose.pose.position.z = position[2]
    message.pose.pose.orientation.w = 1.0
    message.confidence = confidence
    message.valid_pixel_count = 100
    return message


def test_ground_truth_provider_preserves_reset_truth_as_oracle_baseline():
    provider = GroundTruthPoseProvider((0.54, 0.0, 0.14))

    result = provider.estimate(task())

    assert result.success
    assert result.require("cube").position == task().object_position
    assert result.require("target").position == task().target_position
    assert result.require("obstacle").position == (0.54, 0.0, 0.14)


def test_rgbd_provider_requests_scene_specific_required_objects():
    response = EstimateScene.Response()
    response.success = True
    response.latency_sec = 0.03
    response.estimates = [
        estimate_message("cube", (0.58, -0.16, 0.025)),
        estimate_message("target", (0.56, 0.18, 0.025)),
        estimate_message("obstacle", (0.53, 0.01, 0.14)),
    ]
    client = FakeClient(response)

    result = RGBDScenePoseProvider(client).estimate(task())

    assert result.success
    assert result.pose_source == "rgbd"
    assert result.require("cube").position == (0.58, -0.16, 0.025)
    assert client.requests[0].episode_id == 7
    assert list(client.requests[0].required_object_ids) == [
        "cube",
        "target",
        "obstacle",
    ]


def test_rgbd_provider_fails_closed_when_service_is_unavailable():
    result = RGBDScenePoseProvider(FakeClient(available=False), 0.01).estimate(
        task("clear")
    )

    assert not result.success
    assert result.failure_type == "perception_timeout"
