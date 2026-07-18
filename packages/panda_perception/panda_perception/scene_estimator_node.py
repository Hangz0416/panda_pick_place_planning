"""ROS 2 service node exposing synchronized RGB-D scene localization."""

from __future__ import annotations

from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
from panda_pick_place_interfaces.msg import SceneObjectEstimate
from panda_pick_place_interfaces.srv import EstimateScene
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from panda_perception.config import load_config
from panda_perception.detector import DetectionError, HSVDepthDetector


def default_config_path() -> Path:
    return (
        Path(get_package_share_directory("panda_perception"))
        / "config"
        / "perception.yaml"
    )


class SceneEstimatorNode(Node):
    """Wait for a post-request frame and estimate the required scene objects."""

    def __init__(self) -> None:
        super().__init__("panda_scene_estimator")
        group = ReentrantCallbackGroup()
        self.declare_parameter("config_path", str(default_config_path()))
        self.declare_parameter("use_debug_overlay", True)
        self.declare_parameter("perception_profile", "nominal")
        self.declare_parameter(
            "color_topic", "/panda_perception/color/image_raw"
        )
        self.declare_parameter(
            "depth_topic", "/panda_perception/depth/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/panda_perception/camera_info"
        )

        self.config = load_config(str(self.get_parameter("config_path").value))
        self.detector = HSVDepthDetector(self.config)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._condition = threading.Condition()
        self._generation = 0
        self._color_seen = False
        self._depth_seen = False
        self._latest_pair = None
        self._camera_info = None

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        self.color_subscriber = Subscriber(
            self,
            Image,
            color_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=group,
        )
        self.depth_subscriber = Subscriber(
            self,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=group,
        )
        self.color_subscriber.registerCallback(self._mark_color)
        self.depth_subscriber.registerCallback(self._mark_depth)
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            queue_size=10,
            slop=self.config.sync_slop_sec,
        )
        self.synchronizer.registerCallback(self._on_images)
        self.info_subscription = self.create_subscription(
            CameraInfo,
            info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
            callback_group=group,
        )
        self.debug_publisher = self.create_publisher(
            Image, "/panda_perception/debug_overlay", 10
        )
        self.estimate_publisher = self.create_publisher(
            PoseArray, "/panda_perception/estimates", 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/panda_perception/object_markers", 10
        )
        self.service = self.create_service(
            EstimateScene,
            "/panda_perception/estimate_scene",
            self._estimate_scene,
            callback_group=group,
        )

    def _mark_color(self, _message: Image) -> None:
        self._color_seen = True

    def _mark_depth(self, _message: Image) -> None:
        self._depth_seen = True

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._condition:
            self._camera_info = message
            self._condition.notify_all()

    def _on_images(self, color: Image, depth: Image) -> None:
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().warning(f"RGB-D conversion failed: {exc}")
            return
        depth_m = np.asarray(depth_image, dtype=np.float32)
        if str(depth.encoding).upper() in {"16UC1", "MONO16"}:
            depth_m *= 0.001
        with self._condition:
            self._latest_pair = (color_bgr, depth_m, color.header)
            self._generation += 1
            self._condition.notify_all()

    def _estimate_scene(self, request, response):
        started_at = time.perf_counter()
        timeout = float(request.timeout_sec)
        if timeout <= 0.0:
            return self._failure(response, "perception_timeout", started_at)
        with self._condition:
            requested_generation = self._generation
            deadline = time.monotonic() + timeout
            while self._generation <= requested_generation:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    failure = self._missing_stream_failure()
                    return self._failure(response, failure, started_at)
                self._condition.wait(timeout=min(0.05, remaining))
            pair = self._latest_pair
            camera_info = self._camera_info
        if pair is None:
            return self._failure(response, "rgb_unavailable", started_at)
        if camera_info is None:
            return self._failure(response, "camera_info_unavailable", started_at)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.config.camera.world_frame,
                self.config.camera.optical_frame,
                Time(),
            )
        except TransformException:
            return self._failure(response, "tf_unavailable", started_at)

        intrinsics = (
            camera_info.k[0],
            camera_info.k[4],
            camera_info.k[2],
            camera_info.k[5],
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        try:
            detections = self.detector.detect_scene(
                pair[0],
                pair[1],
                intrinsics,
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
                list(request.required_object_ids),
                noise_seed=(int(request.episode_id) * 1_000_003) ^ int(request.task_seed),
                object_variant=str(request.object_variant or "cube"),
                perception_profile=str(
                    self.get_parameter("perception_profile").value
                ),
                expected_workpiece_count=int(request.expected_workpiece_count or 1),
            )
        except DetectionError as exc:
            return self._failure(response, exc.failure_type, started_at)
        except (TypeError, ValueError):
            failure = (
                "invalid_orientation"
                if str(request.object_variant) == "rect_prism"
                else "invalid_depth"
            )
            return self._failure(response, failure, started_at)

        response.success = True
        response.failure_type = ""
        response.latency_sec = time.perf_counter() - started_at
        response.observed_workpiece_count = sum(
            1
            for object_id in detections
            if object_id == "cube" or object_id.startswith("workpiece_")
        )
        stamp = self.get_clock().now().to_msg()
        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.config.camera.world_frame
        for object_id, detection in detections.items():
            message = SceneObjectEstimate()
            message.header.stamp = stamp
            message.header.frame_id = self.config.camera.world_frame
            message.object_id = object_id
            message.object_class = (
                "workpiece"
                if object_id == "cube" or object_id.startswith("workpiece_")
                else object_id
            )
            message.position_valid = detection.position_valid
            message.orientation_valid = detection.orientation_valid
            message.failure_type = detection.failure_type
            message.pose.pose.position.x = detection.position[0]
            message.pose.pose.position.y = detection.position[1]
            message.pose.pose.position.z = detection.position[2]
            orientation = detection.orientation_xyzw
            message.pose.pose.orientation.x = orientation[0]
            message.pose.pose.orientation.y = orientation[1]
            message.pose.pose.orientation.z = orientation[2]
            message.pose.pose.orientation.w = orientation[3]
            variance = max(1.0e-6, (1.0 - detection.confidence) * 1.0e-4)
            for index in (0, 7, 14):
                message.pose.covariance[index] = variance
            message.pose.covariance[35] = detection.yaw_variance
            message.confidence = detection.confidence
            message.valid_pixel_count = detection.valid_pixel_count
            response.estimates.append(message)
            pose = Pose()
            pose.position = message.pose.pose.position
            pose.orientation = message.pose.pose.orientation
            pose_array.poses.append(pose)
        self.estimate_publisher.publish(pose_array)
        self._publish_markers(detections, stamp)
        if bool(self.get_parameter("use_debug_overlay").value):
            self._publish_debug(pair[0], detections, stamp)
        return response

    def _publish_debug(self, color_bgr, detections, stamp) -> None:
        overlay = np.asarray(color_bgr).copy()
        colors = {
            "cube": (0, 0, 255),
            "target": (0, 255, 0),
            "obstacle": (255, 0, 0),
        }
        for object_id, detection in detections.items():
            color = colors.get(object_id, (0, 0, 255))
            contours, _ = cv2.findContours(
                detection.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(overlay, contours, -1, color, 2)
            u, v = detection.centroid_uv
            label = (
                f"{object_id} ({detection.position[0]:.3f},"
                f" {detection.position[1]:.3f}, {detection.position[2]:.3f})"
            )
            if detection.yaw_rad is not None:
                label += (
                    f" yaw={np.degrees(detection.yaw_rad):.1f}deg"
                    f" orient={detection.orientation_confidence:.2f}"
                )
                if detection.axis_uv is not None:
                    cv2.line(
                        overlay,
                        detection.axis_uv[0],
                        detection.axis_uv[1],
                        color,
                        2,
                        cv2.LINE_AA,
                    )
            if not detection.orientation_valid:
                label += f" {detection.failure_type}"
            cv2.circle(overlay, (u, v), 4, color, -1)
            cv2.putText(
                overlay,
                label,
                (u + 6, v - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        message.header.stamp = stamp
        message.header.frame_id = self.config.camera.optical_frame
        self.debug_publisher.publish(message)

    def _publish_markers(self, detections, stamp) -> None:
        markers = MarkerArray()
        for index, (object_id, detection) in enumerate(sorted(detections.items())):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.config.camera.world_frame
            marker.ns = "scene_objects"
            marker.id = index
            marker.type = Marker.ARROW if detection.orientation_valid else Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = detection.position[0]
            marker.pose.position.y = detection.position[1]
            marker.pose.position.z = detection.position[2] + 0.04
            marker.pose.orientation.x = detection.orientation_xyzw[0]
            marker.pose.orientation.y = detection.orientation_xyzw[1]
            marker.pose.orientation.z = detection.orientation_xyzw[2]
            marker.pose.orientation.w = detection.orientation_xyzw[3]
            marker.scale.x = 0.09
            marker.scale.y = 0.012
            marker.scale.z = 0.012
            marker.color.a = 0.9
            if object_id == "target":
                marker.color.g = 1.0
            elif object_id == "obstacle":
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
            marker.text = (
                f"{object_id} yaw={np.degrees(detection.yaw_rad):.1f} "
                f"conf={detection.confidence:.2f}"
                if detection.yaw_rad is not None
                else f"{object_id} conf={detection.confidence:.2f}"
            )
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def _missing_stream_failure(self) -> str:
        if not self._color_seen:
            return "rgb_unavailable"
        if not self._depth_seen:
            return "depth_unavailable"
        return "perception_timeout"

    @staticmethod
    def _failure(response, failure_type: str, started_at: float):
        response.success = False
        response.failure_type = failure_type
        response.latency_sec = time.perf_counter() - started_at
        response.observed_workpiece_count = 0
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneEstimatorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
