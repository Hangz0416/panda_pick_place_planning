import math

import numpy as np

from panda_perception.geometry import deproject_pixels, robust_center, transform_point


def test_deproject_uses_ros_optical_axes():
    points = deproject_pixels(
        np.array([[320.0, 240.0], [420.0, 290.0]]),
        np.array([1.0, 2.0]),
        (500.0, 500.0, 320.0, 240.0),
    )

    np.testing.assert_allclose(points[0], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(points[1], [0.4, 0.2, 2.0])


def test_robust_center_rejects_depth_outlier():
    points = np.array(
        [[0.0, 0.0, 1.0], [0.01, 0.0, 1.0], [-0.01, 0.0, 1.0], [5.0, 5.0, 8.0]]
    )

    center = robust_center(points)

    np.testing.assert_allclose(center, [0.0, 0.0, 1.0], atol=0.01)


def test_transform_point_applies_quaternion_and_translation():
    half = math.sqrt(0.5)

    output = transform_point((1.0, 0.0, 0.0), (1.0, 2.0, 3.0), (0, 0, half, half))

    np.testing.assert_allclose(output, [1.0, 3.0, 3.0], atol=1.0e-9)
