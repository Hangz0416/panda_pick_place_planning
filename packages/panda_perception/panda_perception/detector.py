"""Deterministic HSV segmentation and depth-based position estimation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

from panda_perception.config import ObjectConfig, PerceptionConfig
from panda_perception.geometry import deproject_pixels, robust_center, transform_points


@dataclass(frozen=True)
class Detection:
    object_id: str
    position: tuple[float, float, float]
    confidence: float
    valid_pixel_count: int
    mask: np.ndarray
    centroid_uv: tuple[int, int]
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    yaw_rad: float | None = None
    yaw_variance: float = 0.0
    orientation_confidence: float = 1.0
    axis_uv: tuple[tuple[int, int], tuple[int, int]] | None = None


class DetectionError(RuntimeError):
    """Known perception failure carrying a stable failure classification."""

    def __init__(self, failure_type: str) -> None:
        self.failure_type = failure_type
        super().__init__(failure_type)


class HSVDepthDetector:
    """CPU-only detector designed as a replaceable perception implementation."""

    def __init__(self, config: PerceptionConfig) -> None:
        self.config = config

    def detect_scene(
        self,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        intrinsics: Sequence[float],
        translation: Sequence[float],
        quaternion_xyzw: Sequence[float],
        required_object_ids: Sequence[str],
        *,
        noise_seed: int = 0,
        object_variant: str = "cube",
        perception_profile: str = "nominal",
    ) -> dict[str, Detection]:
        color = np.asarray(color_bgr)
        depth = np.asarray(depth_m, dtype=np.float32)
        if color.ndim != 3 or color.shape[2] != 3:
            raise DetectionError("rgb_unavailable")
        if depth.shape != color.shape[:2]:
            raise DetectionError("depth_unavailable")
        if object_variant not in {"cube", "rect_prism"}:
            raise DetectionError("unsupported_object_variant")
        try:
            profile = self.config.perturbation_profiles[perception_profile]
        except KeyError as exc:
            raise DetectionError("unsupported_perception_profile") from exc
        rng = np.random.default_rng(int(noise_seed))
        processed_color = np.clip(
            color.astype(np.float32) * profile.rgb_scale,
            0.0,
            255.0,
        )
        if profile.rgb_noise_stddev > 0.0:
            processed_color += rng.normal(
                0.0, profile.rgb_noise_stddev, processed_color.shape
            )
        processed_color = np.clip(processed_color, 0.0, 255.0).astype(np.uint8)
        noisy_depth = depth.copy()
        if profile.depth_dropout_rate > 0.0:
            dropout = rng.random(noisy_depth.shape) < profile.depth_dropout_rate
            noisy_depth[dropout] = np.nan
        sigma = self.config.camera.depth_noise_stddev_m
        if sigma > 0.0:
            valid = np.isfinite(noisy_depth) & (noisy_depth > 0.0)
            noisy_depth[valid] += rng.normal(0.0, sigma, int(np.count_nonzero(valid)))
        hsv = cv2.cvtColor(processed_color, cv2.COLOR_BGR2HSV)
        detections = {}
        for object_id in required_object_ids:
            object_config = self.config.objects.get(object_id)
            if object_config is None:
                raise DetectionError("low_confidence")
            detections[object_id] = self._detect_one(
                hsv,
                noisy_depth,
                intrinsics,
                translation,
                quaternion_xyzw,
                object_config,
                require_orientation=(
                    object_id == "cube" and object_variant == "rect_prism"
                ),
            )
        return detections

    def _detect_one(
        self,
        hsv: np.ndarray,
        depth: np.ndarray,
        intrinsics: Sequence[float],
        translation: Sequence[float],
        quaternion_xyzw: Sequence[float],
        config: ObjectConfig,
        require_orientation: bool,
    ) -> Detection:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in config.hsv_ranges:
            mask |= cv2.inRange(
                hsv,
                np.asarray(lower, dtype=np.uint8),
                np.asarray(upper, dtype=np.uint8),
            )
        kernel = np.ones(
            (self.config.morphology_kernel_px,) * 2, dtype=np.uint8
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        component, count, centroid = _largest_component(mask)
        if count < config.min_pixels:
            raise DetectionError(f"{config.object_id}_not_detected")
        confidence = min(1.0, count / float(config.expected_pixels))
        if confidence < self.config.minimum_confidence:
            raise DetectionError("low_confidence")
        rows, columns = np.nonzero(component)
        depths = depth[rows, columns]
        camera = self.config.camera
        valid = (
            np.isfinite(depths)
            & (depths >= camera.near_clip_m)
            & (depths <= camera.far_clip_m)
        )
        if int(np.count_nonzero(valid)) < config.min_pixels:
            raise DetectionError("invalid_depth")
        pixels_uv = np.column_stack((columns[valid], rows[valid]))
        points = deproject_pixels(pixels_uv, depths[valid], intrinsics)
        world_points = transform_points(points, translation, quaternion_xyzw)
        world = robust_center(world_points, self.config.mad_scale)
        yaw = None
        yaw_variance = 0.0
        orientation_confidence = 1.0
        orientation = (0.0, 0.0, 0.0, 1.0)
        axis_uv = None
        if require_orientation:
            yaw, ratio = _principal_yaw(world_points[:, :2])
            threshold = self.config.orientation_min_eigenvalue_ratio
            if ratio < threshold:
                raise DetectionError("orientation_ambiguous")
            orientation_confidence = min(1.0, (ratio - 1.0) / (threshold - 1.0))
            confidence = min(confidence, orientation_confidence)
            if not np.isfinite(yaw):
                raise DetectionError("invalid_orientation")
            half_yaw = 0.5 * yaw
            orientation = (0.0, 0.0, float(np.sin(half_yaw)), float(np.cos(half_yaw)))
            yaw_variance = float(
                np.clip((1.0 / ratio) / max(1, len(world_points)), 1.0e-6, np.deg2rad(10.0) ** 2)
            )
            axis_uv = _principal_axis_uv(pixels_uv, centroid)
        position = (world[0], world[1], config.output_center_z_m)
        return Detection(
            object_id=config.object_id,
            position=position,
            confidence=confidence,
            valid_pixel_count=len(points),
            mask=component,
            centroid_uv=centroid,
            orientation_xyzw=orientation,
            yaw_rad=yaw,
            yaw_variance=yaw_variance,
            orientation_confidence=orientation_confidence,
            axis_uv=axis_uv,
        )


def _largest_component(mask: np.ndarray) -> tuple[np.ndarray, int, tuple[int, int]]:
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return np.zeros_like(mask), 0, (0, 0)
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[index, cv2.CC_STAT_AREA])
    component = np.where(labels == index, 255, 0).astype(np.uint8)
    centroid = tuple(int(round(value)) for value in centroids[index])
    return component, area, centroid


def normalize_symmetric_yaw(yaw_rad: float) -> float:
    """Normalize an unoriented rectangular axis to [-pi/2, pi/2)."""
    return (float(yaw_rad) + np.pi / 2.0) % np.pi - np.pi / 2.0


def _principal_yaw(points_xy: np.ndarray) -> tuple[float, float]:
    values = np.asarray(points_xy, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 3:
        raise DetectionError("invalid_orientation")
    centered = values - np.mean(values, axis=0)
    covariance = centered.T @ centered / max(1, len(centered) - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1.0e-12:
        raise DetectionError("invalid_orientation")
    ratio = float(eigenvalues[-1] / max(eigenvalues[-2], 1.0e-12))
    major = eigenvectors[:, -1]
    yaw = normalize_symmetric_yaw(np.arctan2(major[1], major[0]))
    return yaw, ratio


def _principal_axis_uv(
    pixels_uv: np.ndarray, centroid_uv: tuple[int, int]
) -> tuple[tuple[int, int], tuple[int, int]]:
    values = np.asarray(pixels_uv, dtype=np.float64)
    centered = values - np.mean(values, axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    direction = axes[0]
    extent = max(20.0, float(np.percentile(np.abs(centered @ direction), 90)))
    center = np.asarray(centroid_uv, dtype=np.float64)
    first = tuple(int(round(value)) for value in center - extent * direction)
    second = tuple(int(round(value)) for value in center + extent * direction)
    return first, second
