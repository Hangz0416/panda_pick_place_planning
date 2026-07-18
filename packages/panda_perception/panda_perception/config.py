"""Typed loading for the shared RGB-D camera and segmentation parameters."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class CameraConfig:
    model_name: str
    link_name: str
    sensor_name: str
    optical_frame: str
    world_frame: str
    position: tuple[float, float, float]
    rpy: tuple[float, float, float]
    optical_rpy_from_link: tuple[float, float, float]
    width: int
    height: int
    update_rate_hz: float
    horizontal_fov_rad: float
    near_clip_m: float
    far_clip_m: float
    depth_noise_stddev_m: float


@dataclass(frozen=True)
class ObjectConfig:
    object_id: str
    hsv_ranges: tuple[tuple[tuple[int, int, int], tuple[int, int, int]], ...]
    min_pixels: int
    expected_pixels: int
    output_center_z_m: float


@dataclass(frozen=True)
class PerturbationConfig:
    rgb_scale: float
    rgb_noise_stddev: float
    depth_dropout_rate: float


@dataclass(frozen=True)
class PerceptionConfig:
    camera: CameraConfig
    sync_slop_sec: float
    morphology_kernel_px: int
    mad_scale: float
    minimum_confidence: float
    orientation_min_eigenvalue_ratio: float
    perturbation_profiles: dict[str, PerturbationConfig]
    objects: dict[str, ObjectConfig]


def load_config(path: str | Path) -> PerceptionConfig:
    """Load and validate the one source of camera and detector parameters."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    camera_raw = dict(raw["camera"])
    camera = CameraConfig(
        model_name=str(camera_raw["model_name"]),
        link_name=str(camera_raw["link_name"]),
        sensor_name=str(camera_raw["sensor_name"]),
        optical_frame=str(camera_raw["optical_frame"]),
        world_frame=str(camera_raw["world_frame"]),
        position=_triple(camera_raw["position"]),
        rpy=_triple(camera_raw["rpy"]),
        optical_rpy_from_link=_triple(camera_raw["optical_rpy_from_link"]),
        width=int(camera_raw["width"]),
        height=int(camera_raw["height"]),
        update_rate_hz=float(camera_raw["update_rate_hz"]),
        horizontal_fov_rad=float(camera_raw["horizontal_fov_rad"]),
        near_clip_m=float(camera_raw["near_clip_m"]),
        far_clip_m=float(camera_raw["far_clip_m"]),
        depth_noise_stddev_m=float(camera_raw["depth_noise_stddev_m"]),
    )
    processing = dict(raw["processing"])
    objects = {
        str(object_id): _object_config(str(object_id), values)
        for object_id, values in processing["objects"].items()
    }
    profiles = {
        str(name): PerturbationConfig(
            rgb_scale=float(values["rgb_scale"]),
            rgb_noise_stddev=float(values["rgb_noise_stddev"]),
            depth_dropout_rate=float(values["depth_dropout_rate"]),
        )
        for name, values in processing["perturbation_profiles"].items()
    }
    config = PerceptionConfig(
        camera=camera,
        sync_slop_sec=float(processing["sync_slop_sec"]),
        morphology_kernel_px=int(processing["morphology_kernel_px"]),
        mad_scale=float(processing["mad_scale"]),
        minimum_confidence=float(processing["minimum_confidence"]),
        orientation_min_eigenvalue_ratio=float(
            processing["orientation_min_eigenvalue_ratio"]
        ),
        perturbation_profiles=profiles,
        objects=objects,
    )
    _validate(config)
    return config


def _object_config(object_id: str, raw: dict[str, Any]) -> ObjectConfig:
    ranges = tuple(
        (
            tuple(int(value) for value in lower),
            tuple(int(value) for value in upper),
        )
        for lower, upper in raw["hsv_ranges"]
    )
    return ObjectConfig(
        object_id=object_id,
        hsv_ranges=ranges,
        min_pixels=int(raw["min_pixels"]),
        expected_pixels=int(raw["expected_pixels"]),
        output_center_z_m=float(raw["output_center_z_m"]),
    )


def _triple(values: Any) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError("expected three values")
    return result


def _validate(config: PerceptionConfig) -> None:
    camera = config.camera
    if camera.width <= 0 or camera.height <= 0 or camera.update_rate_hz <= 0.0:
        raise ValueError("camera dimensions and rate must be positive")
    if not 0.0 < camera.near_clip_m < camera.far_clip_m:
        raise ValueError("camera clip range is invalid")
    if camera.depth_noise_stddev_m < 0.0:
        raise ValueError("depth noise must be non-negative")
    if config.morphology_kernel_px <= 0 or config.morphology_kernel_px % 2 == 0:
        raise ValueError("morphology kernel must be a positive odd number")
    if not 0.0 <= config.minimum_confidence <= 1.0:
        raise ValueError("minimum confidence must be in [0, 1]")
    if config.orientation_min_eigenvalue_ratio <= 1.0:
        raise ValueError("orientation eigenvalue ratio must be greater than one")
    expected_profiles = {
        "nominal", "low_light", "rgb_noise", "depth_dropout", "combined"
    }
    if set(config.perturbation_profiles) != expected_profiles:
        raise ValueError("perception perturbation profiles are incomplete")
    for profile in config.perturbation_profiles.values():
        if not 0.0 < profile.rgb_scale <= 1.0:
            raise ValueError("rgb_scale must be in (0, 1]")
        if profile.rgb_noise_stddev < 0.0:
            raise ValueError("rgb noise must be non-negative")
        if not 0.0 <= profile.depth_dropout_rate < 1.0:
            raise ValueError("depth dropout rate must be in [0, 1)")
    if set(config.objects) != {"cube", "target", "obstacle"}:
        raise ValueError("config must define cube, target, and obstacle")
