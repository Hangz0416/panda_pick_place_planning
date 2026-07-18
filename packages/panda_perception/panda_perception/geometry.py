"""Small camera and rigid-transform helpers kept independent of ROS."""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np


def deproject_pixels(
    pixels_uv: np.ndarray,
    depths_m: np.ndarray,
    intrinsics: Sequence[float],
) -> np.ndarray:
    """Deproject image pixels into a ROS optical frame (+Z forward)."""
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    depths = np.asarray(depths_m, dtype=np.float64)
    x = (pixels[:, 0] - cx) * depths / fx
    y = (pixels[:, 1] - cy) * depths / fy
    return np.column_stack((x, y, depths))


def robust_center(points: np.ndarray, mad_scale: float = 3.5) -> np.ndarray:
    """Return a coordinate-wise median after radial MAD rejection."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) == 0:
        raise ValueError("points must be a non-empty Nx3 array")
    center = np.median(values, axis=0)
    distances = np.linalg.norm(values - center, axis=1)
    median_distance = float(np.median(distances))
    mad = float(np.median(np.abs(distances - median_distance)))
    if mad > 1.0e-12:
        inliers = values[distances <= median_distance + mad_scale * mad]
        if len(inliers):
            center = np.median(inliers, axis=0)
    return center


def transform_point(
    point: Sequence[float],
    translation: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float]:
    """Apply a parent-from-child transform to a 3-D point."""
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    output = rotation @ np.asarray(point, dtype=np.float64)
    output += np.asarray(translation, dtype=np.float64)
    return tuple(float(value) for value in output)


def transform_points(
    points: np.ndarray,
    translation: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Apply one parent-from-child transform to an Nx3 point array."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points must be an Nx3 array")
    origin = np.asarray(transform_point((0.0, 0.0, 0.0), translation, quaternion_xyzw))
    axes = np.asarray(
        [
            transform_point(axis, translation, quaternion_xyzw)
            for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        ]
    )
    rotation = (axes - origin).T
    return values @ rotation.T + origin
