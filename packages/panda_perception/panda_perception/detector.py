"""Deterministic HSV segmentation and depth-based position estimation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

from panda_perception.config import ObjectConfig, PerceptionConfig
from panda_perception.geometry import deproject_pixels, robust_center, transform_point


@dataclass(frozen=True)
class Detection:
    object_id: str
    position: tuple[float, float, float]
    confidence: float
    valid_pixel_count: int
    mask: np.ndarray
    centroid_uv: tuple[int, int]


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
    ) -> dict[str, Detection]:
        color = np.asarray(color_bgr)
        depth = np.asarray(depth_m, dtype=np.float32)
        if color.ndim != 3 or color.shape[2] != 3:
            raise DetectionError("rgb_unavailable")
        if depth.shape != color.shape[:2]:
            raise DetectionError("depth_unavailable")
        noisy_depth = depth.copy()
        sigma = self.config.camera.depth_noise_stddev_m
        if sigma > 0.0:
            rng = np.random.default_rng(int(noise_seed))
            valid = np.isfinite(noisy_depth) & (noisy_depth > 0.0)
            noisy_depth[valid] += rng.normal(0.0, sigma, int(np.count_nonzero(valid)))
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
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
        camera_center = robust_center(points, self.config.mad_scale)
        world = transform_point(camera_center, translation, quaternion_xyzw)
        position = (world[0], world[1], config.output_center_z_m)
        return Detection(
            object_id=config.object_id,
            position=position,
            confidence=confidence,
            valid_pixel_count=len(points),
            mask=component,
            centroid_uv=centroid,
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
