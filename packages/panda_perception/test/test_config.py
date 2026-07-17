from pathlib import Path

from panda_perception.config import load_config


def config_path() -> Path:
    return Path(__file__).parents[1] / "config" / "perception.yaml"


def test_shared_config_has_fixed_overhead_rgbd_contract():
    config = load_config(config_path())

    assert config.camera.width == 640
    assert config.camera.height == 480
    assert config.camera.update_rate_hz == 20.0
    assert config.camera.optical_frame == "overhead_camera_optical_frame"
    assert set(config.objects) == {"cube", "target", "obstacle"}


def test_depth_noise_is_fixed_and_lightweight():
    config = load_config(config_path())

    assert config.camera.depth_noise_stddev_m == 0.002
