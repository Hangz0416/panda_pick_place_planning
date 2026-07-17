from dataclasses import replace

from panda_gz_bringup.reset_contract import (
    RESET_VALIDATION_THRESHOLDS,
    ResetValidationMetrics,
)


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
