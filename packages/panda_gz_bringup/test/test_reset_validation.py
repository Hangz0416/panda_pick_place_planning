from dataclasses import replace

from panda_gz_bringup.reset_contract import (
    RESET_VALIDATION_THRESHOLDS,
    ResetValidationMetrics,
)
from panda_gz_bringup.reset_validation import parse_args, workpiece_ids


def _passing_metrics() -> ResetValidationMetrics:
    return ResetValidationMetrics(
        **{
            field: limit
            for field, limit in RESET_VALIDATION_THRESHOLDS.items()
        }
    )


def test_threshold_boundary_passes_and_excess_fails():
    metrics = _passing_metrics()
    assert metrics.passed()
    assert not replace(
        metrics,
        cube_linear_speed_mps=(
            RESET_VALIDATION_THRESHOLDS["cube_linear_speed_mps"] + 1.0e-9
        ),
    ).passed()


def test_reset_validation_defaults_to_three_rectangular_workpieces():
    options = parse_args([])

    assert options.object_variant == "rect_prism"
    assert options.workpiece_count == 3


def test_reset_validation_preserves_legacy_and_multi_object_identities():
    assert workpiece_ids(1) == ("cube",)
    assert workpiece_ids(3) == (
        "workpiece_0",
        "workpiece_1",
        "workpiece_2",
    )
