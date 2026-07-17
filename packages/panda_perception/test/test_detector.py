from pathlib import Path

import cv2
import numpy as np
import pytest

from panda_perception.config import load_config
from panda_perception.detector import DetectionError, HSVDepthDetector


def detector() -> HSVDepthDetector:
    path = Path(__file__).parents[1] / "config" / "perception.yaml"
    return HSVDepthDetector(load_config(path))


def synthetic_scene():
    color = np.zeros((480, 640, 3), dtype=np.uint8)
    color[:] = (60, 90, 130)
    cv2.rectangle(color, (290, 190), (310, 210), (0, 0, 255), -1)
    cv2.circle(color, (400, 300), 28, (0, 255, 0), -1)
    cv2.rectangle(color, (180, 215), (220, 265), (255, 0, 0), -1)
    depth = np.full((480, 640), 1.0, dtype=np.float32)
    return color, depth


def test_detects_cube_target_and_obstacle_with_deterministic_depth_noise():
    color, depth = synthetic_scene()

    result = detector().detect_scene(
        color,
        depth,
        (500.0, 500.0, 320.0, 240.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        ["cube", "target", "obstacle"],
        noise_seed=42,
    )

    assert set(result) == {"cube", "target", "obstacle"}
    assert result["cube"].valid_pixel_count >= 80
    assert result["target"].confidence == 1.0
    assert abs(result["cube"].position[0] + 0.04) < 0.005
    assert abs(result["cube"].position[1] + 0.08) < 0.005
    assert result["cube"].position[2] == 0.025
    assert result["obstacle"].position[2] == 0.14


def test_same_episode_seed_produces_same_estimate():
    color, depth = synthetic_scene()
    arguments = (
        color,
        depth,
        (500.0, 500.0, 320.0, 240.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        ["cube"],
    )

    first = detector().detect_scene(*arguments, noise_seed=7)
    second = detector().detect_scene(*arguments, noise_seed=7)

    assert first["cube"].position == second["cube"].position


def test_invalid_depth_is_classified():
    color, depth = synthetic_scene()
    depth[:] = np.nan

    with pytest.raises(DetectionError, match="invalid_depth"):
        detector().detect_scene(
            color,
            depth,
            (500.0, 500.0, 320.0, 240.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            ["cube"],
        )


def test_missing_required_object_is_classified():
    color, depth = synthetic_scene()
    color[:] = 0

    with pytest.raises(DetectionError, match="cube_not_detected"):
        detector().detect_scene(
            color,
            depth,
            (500.0, 500.0, 320.0, 240.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            ["cube"],
        )
