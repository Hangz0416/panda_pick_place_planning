"""Dependency-light, provenance-preserving benchmark report generation.

The report generator consumes either per-episode JSONL written by the evaluator
or aggregate JSON written by :mod:`benchmark`.  It deliberately keeps missing
metrics as ``None``/``N/A`` instead of converting them to zero.  SVG output is
rendered with the Python standard library so headless ROS workspaces do not need
matplotlib, pandas, or a browser runtime.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import hashlib
from html import escape
import json
import math
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable, Sequence


REPORT_SCHEMA_VERSION = 1
PLANNING_ONLY = "planning_only"
FULL_EPISODE = "full_episode"

_SERIES_COLORS = (
    "#0072B2",  # blue
    "#E69F00",  # orange
    "#009E73",  # green
    "#CC79A7",  # purple
    "#D55E00",  # vermillion
    "#56B4E9",  # sky blue
)
_FAILURE_COLORS = (
    "#D55E00",
    "#CC79A7",
    "#E69F00",
    "#0072B2",
    "#009E73",
    "#56B4E9",
    "#8C6D31",
    "#6B6ECF",
    "#E377C2",
    "#17BECF",
    "#BCBD22",
    "#7F7F7F",
)


@dataclass
class ReportGroup:
    """Normalized aggregate for one comparable backend/scene/planner group."""

    dataset: str
    group_key: str
    display_label: str
    evaluation_mode: str
    episodes: int
    planning_success_rate: float | None
    execution_success_rate: float | None
    task_success_rate: float | None
    planning_time_mean_sec: float | None
    planning_time_median_sec: float | None
    planning_time_p95_sec: float | None
    path_length_mean_rad: float | None
    path_length_median_rad: float | None
    tracking_error_mean_rad: float | None
    final_xy_error_mean_m: float | None
    final_z_error_mean_m: float | None
    pose_source: str
    perception_success_rate: float | None
    perception_latency_mean_sec: float | None
    perception_latency_median_sec: float | None
    perception_latency_p95_sec: float | None
    object_position_error_mean_m: float | None
    object_position_error_p95_m: float | None
    target_position_error_mean_m: float | None
    target_position_error_p95_m: float | None
    obstacle_position_error_mean_m: float | None
    obstacle_position_error_p95_m: float | None
    failure_counts: dict[str, int] = field(default_factory=dict)
    metric_sample_counts: dict[str, int | None] = field(default_factory=dict)
    metric_origins: dict[str, str] = field(default_factory=dict)


@dataclass
class LoadedDataset:
    """A source file plus normalized groups and optional episode coverage."""

    label: str
    path: Path
    source_kind: str
    sha256: str
    groups: list[ReportGroup]
    evaluation_modes: list[str]
    episode_count: int
    coverage_keys: set[tuple[str, int]] | None
    batch_status: str | None = None
    warnings: list[str] = field(default_factory=list)

    def manifest_source(self) -> dict[str, Any]:
        return {
            "label": self.label,
            # Preserve the caller's relative path so committed manifests stay
            # portable across workspaces.  The content hash is the immutable
            # provenance key; an absolute build-machine path adds no evidence.
            "path": str(self.path),
            "source_kind": self.source_kind,
            "sha256": self.sha256,
            "episode_count": self.episode_count,
            "evaluation_modes": self.evaluation_modes,
            "batch_status": self.batch_status,
            "warnings": self.warnings,
        }


def load_dataset(input_spec: str) -> LoadedDataset:
    """Load ``[LABEL=]PATH`` containing EpisodeResult JSONL or summary JSON."""

    label, path = _split_input_spec(input_spec)
    if not path.is_file():
        raise ValueError(f"input file does not exist: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()

    if path.suffix.lower() == ".jsonl":
        records = _read_jsonl(path)
        return _dataset_from_records(label, path, digest, records)

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {path}: {exc}") from exc

    if isinstance(payload, list):
        return _dataset_from_records(label, path, digest, payload)
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object, array, or JSONL records: {path}")
    if "groups" in payload or "backend_groups" in payload:
        return _dataset_from_summary(label, path, digest, payload)
    if "scene" in payload and "planning_success" in payload:
        return _dataset_from_records(label, path, digest, [payload])
    raise ValueError(f"JSON is neither an EpisodeResult nor a benchmark summary: {path}")


def validate_paired_coverage(datasets: Sequence[LoadedDataset]) -> dict[str, Any]:
    """Compare unique ``(scene, task_seed)`` coverage across raw datasets.

    Aggregate summary JSON cannot be validated because it does not contain task
    seeds.  Repeated keys (for example one task evaluated by two planners) are
    intentionally collapsed; the check validates task coverage, not row counts.
    """

    if len(datasets) < 2:
        return {
            "available": False,
            "paired": False,
            "reason": "at least two inputs are required",
        }
    unavailable = [item.label for item in datasets if item.coverage_keys is None]
    if unavailable:
        return {
            "available": False,
            "paired": False,
            "reason": "summary JSON does not retain per-episode task seeds",
            "unavailable_inputs": unavailable,
        }

    coverages = {
        item.label: set(item.coverage_keys or set())
        for item in datasets
    }
    union = set().union(*coverages.values())
    intersection = set.intersection(*coverages.values()) if coverages else set()
    missing = {
        label: [_coverage_key_json(key) for key in sorted(union - keys)]
        for label, keys in coverages.items()
    }
    paired = all(not values for values in missing.values())
    return {
        "available": True,
        "paired": paired,
        "union_task_count": len(union),
        "paired_task_count": len(intersection),
        "coverage_counts": {
            label: len(keys) for label, keys in coverages.items()
        },
        "missing_by_input": missing,
    }


def generate_report(
    datasets: Sequence[LoadedDataset],
    output_dir: Path,
    title: str = "Panda pick-place benchmark",
    *,
    force: bool = False,
    paired_coverage: dict[str, Any] | None = None,
) -> Path:
    """Write mode-scoped SVG charts and a machine-readable report manifest."""

    if not datasets:
        raise ValueError("at least one dataset is required")
    labels = [dataset.label for dataset in datasets]
    if len(labels) != len(set(labels)):
        raise ValueError("input labels must be unique")

    groups = [group for dataset in datasets for group in dataset.groups]
    if not groups:
        raise ValueError("the inputs contain no reportable groups")

    modes = [
        mode
        for mode in (PLANNING_ONLY, FULL_EPISODE)
        if any(group.evaluation_mode == mode for group in groups)
    ]
    chart_specs: list[tuple[Path, str, str, list[ReportGroup]]] = []
    for mode in modes:
        scoped = [group for group in groups if group.evaluation_mode == mode]
        mode_title = "Planning-only" if mode == PLANNING_ONLY else "Full episode"
        chart_specs.extend(
            [
                (
                    output_dir / f"{mode}_success_rates.svg",
                    "rates",
                    f"{title} — {mode_title} success rates",
                    scoped,
                ),
                (
                    output_dir / f"{mode}_planning_time.svg",
                    "planning_time",
                    f"{title} — {mode_title} planning time",
                    scoped,
                ),
                (
                    output_dir / f"{mode}_path_length.svg",
                    "path_length",
                    f"{title} — {mode_title} joint-path length",
                    scoped,
                ),
                (
                    output_dir / f"{mode}_failure_counts.svg",
                    "failures",
                    f"{title} — {mode_title} failure counts",
                    scoped,
                ),
            ]
        )
        if mode == FULL_EPISODE:
            chart_specs.extend(
                [
                    (
                        output_dir / "full_episode_place_error.svg",
                        "place_error",
                        f"{title} — Full episode placement error",
                        scoped,
                    ),
                    (
                        output_dir / "full_episode_tracking_error.svg",
                        "tracking_error",
                        f"{title} — Full episode joint tracking error",
                        scoped,
                    ),
                ]
            )
            if any(group.perception_success_rate is not None for group in scoped):
                chart_specs.extend(
                    [
                        (
                            output_dir / "full_episode_perception.svg",
                            "perception",
                            f"{title} — Full episode RGB-D perception",
                            scoped,
                        ),
                        (
                            output_dir / "full_episode_perception_error.svg",
                            "perception_error",
                            f"{title} — Full episode RGB-D position error",
                            scoped,
                        ),
                        (
                            output_dir / "full_episode_perception_latency.svg",
                            "perception_latency",
                            f"{title} — Full episode RGB-D perception latency",
                            scoped,
                        ),
                    ]
                )

    manifest_path = output_dir / "report_manifest.json"
    targets = [path for path, _, _, _ in chart_specs] + [manifest_path]
    existing = [path for path in targets if path.exists()]
    if existing and not force:
        names = ", ".join(path.name for path in existing)
        raise FileExistsError(
            f"refusing to overwrite existing report files ({names}); pass --force"
        )
    output_dir.mkdir(parents=True, exist_ok=True)

    chart_manifest = []
    for path, chart_kind, chart_title, scoped in chart_specs:
        _write_chart(path, chart_kind, chart_title, scoped)
        chart_manifest.append(
            {
                "path": path.name,
                "evaluation_mode": scoped[0].evaluation_mode,
                "metric": chart_kind,
            }
        )

    warnings = [
        f"{dataset.label}: {warning}"
        for dataset in datasets
        for warning in dataset.warnings
    ]
    incomplete = [
        dataset.label
        for dataset in datasets
        if dataset.batch_status not in (None, "complete")
    ]
    if incomplete:
        warnings.append(
            "incomplete batch status reported by: " + ", ".join(incomplete)
        )

    manifest = {
        "report_schema_version": REPORT_SCHEMA_VERSION,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "title": title,
        "sources": [dataset.manifest_source() for dataset in datasets],
        "evaluation_modes": modes,
        "semantic_notes": {
            PLANNING_ONLY: (
                "Only planning_success is a success-rate outcome; execution and "
                "task success are not applicable because no trajectory or grasp ran."
            ),
            FULL_EPISODE: (
                "Planning, execution, and task rates are shown when reported; "
                "missing metrics remain null/N/A."
            ),
            "path_length": (
                "Mean and median use planning-success rows with a reported "
                "top-level joint path length."
            ),
            "placement_error": (
                "Uses reported final errors, or derives absolute XY/Z error from "
                "recorded final and target positions in raw full-episode JSONL."
            ),
            "perception": (
                "Detection rate, service latency, and position error are reported "
                "only for rows that persist RGB-D perception observations."
            ),
        },
        "groups": [asdict(group) for group in groups],
        "charts": chart_manifest,
        "paired_coverage": paired_coverage,
        "warnings": warnings,
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return manifest_path


def _split_input_spec(input_spec: str) -> tuple[str, Path]:
    if "=" in input_spec:
        candidate_label, candidate_path = input_spec.split("=", 1)
        if candidate_label.strip() and candidate_path.strip():
            return candidate_label.strip(), Path(candidate_path).expanduser()
    path = Path(input_spec).expanduser()
    label = path.stem.replace("_summary", "").replace("_results", "")
    return label or "dataset", path


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    f"invalid JSONL record in {path} at line {line_number}: {exc}"
                ) from exc
            if not isinstance(payload, dict):
                raise ValueError(
                    f"expected an object in {path} at line {line_number}"
                )
            records.append(payload)
    if not records:
        raise ValueError(f"input contains no EpisodeResult records: {path}")
    return records


def _dataset_from_records(
    label: str,
    path: Path,
    digest: str,
    records: Sequence[dict[str, Any]],
) -> LoadedDataset:
    if not records:
        raise ValueError(f"input contains no EpisodeResult records: {path}")
    normalized: list[dict[str, Any]] = []
    coverage: set[tuple[str, int]] = set()
    warnings: list[str] = []
    for index, record in enumerate(records, start=1):
        if not isinstance(record, dict):
            raise ValueError(f"record {index} in {path} is not a JSON object")
        if "scene" not in record or "planning_success" not in record:
            raise ValueError(
                f"record {index} in {path} lacks scene/planning_success"
            )
        scene = str(record["scene"])
        if "task_seed" not in record:
            raise ValueError(f"record {index} in {path} lacks task_seed")
        try:
            task_seed = int(record["task_seed"])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"record {index} in {path} has invalid task_seed"
            ) from exc
        coverage.add((scene, task_seed))
        normalized.append(_normalize_record(record, path, index))

    grouped: dict[tuple[str, ...], list[dict[str, Any]]] = defaultdict(list)
    for record in normalized:
        key = (
            record["evaluation_mode"],
            record["backend"],
            record["simulator"],
            record["scene"],
            record["planner_id"],
            record["grasp_mode"],
            record["pose_source"],
        )
        grouped[key].append(record)

    groups = [
        _summarize_records(label, key, rows)
        for key, rows in sorted(grouped.items())
    ]
    modes = sorted({group.evaluation_mode for group in groups})
    if len(modes) > 1:
        warnings.append(
            "one raw file contains planning-only and full-episode rows; charts are separated"
        )
    return LoadedDataset(
        label=label,
        path=path,
        source_kind="episode_jsonl" if path.suffix.lower() == ".jsonl" else "episode_json",
        sha256=digest,
        groups=groups,
        evaluation_modes=modes,
        episode_count=len(normalized),
        coverage_keys=coverage,
        warnings=warnings,
    )


def _normalize_record(
    record: dict[str, Any], path: Path, index: int
) -> dict[str, Any]:
    planning_success = record["planning_success"]
    if not isinstance(planning_success, bool):
        raise ValueError(
            f"record {index} in {path} has non-boolean planning_success"
        )
    simulator = str(record.get("simulator", record.get("backend", "pybullet")))
    mode = PLANNING_ONLY if simulator == "moveit_mock" else FULL_EPISODE
    normalized = {
        "evaluation_mode": mode,
        "backend": str(record.get("backend", "pybullet")),
        "simulator": simulator,
        "scene": str(record["scene"]),
        "planner_id": str(record.get("planner_id") or record.get("planner", "unknown")),
        "grasp_mode": str(record.get("grasp_mode", "assisted")),
        "pose_source": str(record.get("pose_source", "")),
        "planning_success": planning_success,
        "planning_time_sec": _record_number(record, "planning_time_sec", path, index),
        "path_length_rad": _record_number(record, "path_length_rad", path, index),
        "execution_success": _record_bool(record, "execution_success", path, index),
        "task_success": _record_bool(record, "task_success", path, index),
        "tracking_error_rad": _record_number(
            record, "max_joint_tracking_error_rad", path, index
        ),
        "failure_type": str(record.get("failure_type") or ""),
        "perception_success": _record_bool(
            record, "perception_success", path, index
        ),
        "perception_latency_sec": _record_number(
            record, "perception_latency_sec", path, index
        ),
        "object_position_error_m": _record_number(
            record, "object_position_error_m", path, index
        ),
        "target_position_error_m": _record_number(
            record, "target_position_error_m", path, index
        ),
        "obstacle_position_error_m": _record_number(
            record, "obstacle_position_error_m", path, index
        ),
    }
    xy_error = _record_number(record, "final_xy_error_m", path, index)
    z_error = _record_number(record, "final_z_error_m", path, index)
    error_origin = "reported" if xy_error is not None or z_error is not None else "unavailable"
    if mode == FULL_EPISODE and (xy_error is None or z_error is None):
        derived = _derive_place_error(record)
        if derived is not None:
            if xy_error is None:
                xy_error = derived[0]
            if z_error is None:
                z_error = derived[1]
            error_origin = (
                "derived_from_positions"
                if "final_xy_error_m" not in record and "final_z_error_m" not in record
                else "mixed_reported_and_derived"
            )
    if mode == PLANNING_ONLY:
        # A mock planning regression did not execute or place the object.  False
        # task/execution flags and static positions are not outcome measurements.
        normalized["execution_success"] = None
        normalized["task_success"] = None
        normalized["tracking_error_rad"] = None
        xy_error = None
        z_error = None
        error_origin = "not_applicable"
    normalized["xy_error_m"] = xy_error
    normalized["z_error_m"] = z_error
    normalized["place_error_origin"] = error_origin
    return normalized


def _record_number(
    record: dict[str, Any], key: str, path: Path, index: int
) -> float | None:
    if key not in record or record[key] is None:
        return None
    value = record[key]
    if isinstance(value, bool):
        raise ValueError(f"record {index} in {path} has invalid numeric {key}")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"record {index} in {path} has invalid numeric {key}"
        ) from exc
    if not math.isfinite(number):
        raise ValueError(f"record {index} in {path} has non-finite {key}")
    return number


def _record_bool(
    record: dict[str, Any], key: str, path: Path, index: int
) -> bool | None:
    if key not in record or record[key] is None:
        return None
    if not isinstance(record[key], bool):
        raise ValueError(f"record {index} in {path} has non-boolean {key}")
    return record[key]


def _derive_place_error(record: dict[str, Any]) -> tuple[float, float] | None:
    final = record.get("object_final_position")
    target = record.get("target_position")
    if not isinstance(final, (list, tuple)) or not isinstance(target, (list, tuple)):
        return None
    if len(final) < 3 or len(target) < 3:
        return None
    try:
        values = [float(value) for value in (*final[:3], *target[:3])]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    dx = values[0] - values[3]
    dy = values[1] - values[4]
    dz = values[2] - values[5]
    return math.hypot(dx, dy), abs(dz)


def _summarize_records(
    dataset: str,
    key: tuple[str, ...],
    records: list[dict[str, Any]],
) -> ReportGroup:
    mode, backend, simulator, scene, planner_id, grasp_mode, pose_source = key
    planning_times = _reported(records, "planning_time_sec")
    path_lengths = [
        record["path_length_rad"]
        for record in records
        if record["planning_success"] and record["path_length_rad"] is not None
    ]
    tracking_errors = _reported(records, "tracking_error_rad")
    xy_errors = _reported(records, "xy_error_m")
    z_errors = _reported(records, "z_error_m")
    perception_latencies = _reported(records, "perception_latency_sec")
    object_errors = _reported(records, "object_position_error_m")
    target_errors = _reported(records, "target_position_error_m")
    obstacle_errors = _reported(records, "obstacle_position_error_m")
    failure_counts: Counter[str] = Counter()
    for record in records:
        if mode == PLANNING_ONLY:
            if record["planning_success"]:
                continue
            failure_counts[
                record["failure_type"] or "unclassified_planning_failure"
            ] += 1
        else:
            if record["task_success"] is True:
                continue
            failure_counts[record["failure_type"] or "unclassified_failure"] += 1

    origin_values = {record["place_error_origin"] for record in records}
    place_origin = origin_values.pop() if len(origin_values) == 1 else "mixed"
    display = _display_label(
        dataset, backend, simulator, scene, planner_id, grasp_mode, pose_source
    )
    return ReportGroup(
        dataset=dataset,
        group_key="/".join(
            (backend, simulator, scene, planner_id, grasp_mode, pose_source)
        ),
        display_label=display,
        evaluation_mode=mode,
        episodes=len(records),
        planning_success_rate=_bool_rate(records, "planning_success"),
        execution_success_rate=(
            None if mode == PLANNING_ONLY else _bool_rate(records, "execution_success")
        ),
        task_success_rate=(
            None if mode == PLANNING_ONLY else _bool_rate(records, "task_success")
        ),
        planning_time_mean_sec=_safe_mean(planning_times),
        planning_time_median_sec=_safe_median(planning_times),
        planning_time_p95_sec=_percentile(planning_times, 0.95),
        path_length_mean_rad=_safe_mean(path_lengths),
        path_length_median_rad=_safe_median(path_lengths),
        tracking_error_mean_rad=_safe_mean(tracking_errors),
        final_xy_error_mean_m=_safe_mean(xy_errors),
        final_z_error_mean_m=_safe_mean(z_errors),
        pose_source=pose_source,
        perception_success_rate=_bool_rate(records, "perception_success"),
        perception_latency_mean_sec=_safe_mean(perception_latencies),
        perception_latency_median_sec=_safe_median(perception_latencies),
        perception_latency_p95_sec=_percentile(perception_latencies, 0.95),
        object_position_error_mean_m=_safe_mean(object_errors),
        object_position_error_p95_m=_percentile(object_errors, 0.95),
        target_position_error_mean_m=_safe_mean(target_errors),
        target_position_error_p95_m=_percentile(target_errors, 0.95),
        obstacle_position_error_mean_m=_safe_mean(obstacle_errors),
        obstacle_position_error_p95_m=_percentile(obstacle_errors, 0.95),
        failure_counts=dict(sorted(failure_counts.items())),
        metric_sample_counts={
            "planning_success_rate": len(records),
            "execution_success_rate": sum(
                record["execution_success"] is not None for record in records
            ),
            "task_success_rate": sum(
                record["task_success"] is not None for record in records
            ),
            "planning_time": len(planning_times),
            "path_length": len(path_lengths),
            "tracking_error": len(tracking_errors),
            "final_xy_error": len(xy_errors),
            "final_z_error": len(z_errors),
            "perception_success_rate": sum(
                record["perception_success"] is not None for record in records
            ),
            "perception_latency": len(perception_latencies),
            "object_position_error": len(object_errors),
            "target_position_error": len(target_errors),
            "obstacle_position_error": len(obstacle_errors),
        },
        metric_origins={
            "planning_time": "reported_episode_values",
            "path_length": "reported_planning_success_episode_values",
            "tracking_error": (
                "reported_episode_values" if tracking_errors else "unavailable"
            ),
            "placement_error": place_origin,
            "perception": (
                "reported_episode_values" if perception_latencies else "unavailable"
            ),
        },
    )


def _dataset_from_summary(
    label: str,
    path: Path,
    digest: str,
    summary: dict[str, Any],
) -> LoadedDataset:
    raw_groups = summary.get("backend_groups") or summary.get("groups")
    if not isinstance(raw_groups, dict) or not raw_groups:
        raise ValueError(f"benchmark summary has no groups: {path}")
    top_mode = _normalize_mode(summary.get("evaluation_mode"))
    groups: list[ReportGroup] = []
    for group_key, values in sorted(raw_groups.items()):
        if not isinstance(values, dict):
            raise ValueError(f"summary group {group_key!r} in {path} is not an object")
        mode = _summary_group_mode(top_mode, values)
        backend, simulator, scene, planner = _parse_summary_group_key(str(group_key))
        failure_counts = _summary_failure_counts(values.get("failure_counts"), mode)
        groups.append(
            ReportGroup(
                dataset=label,
                group_key=str(group_key),
                display_label=_display_label(
                    label,
                    backend,
                    simulator,
                    scene,
                    planner,
                    "",
                    str(values.get("pose_source", "")),
                ),
                evaluation_mode=mode,
                episodes=_summary_int(values.get("episodes"), "episodes", path),
                planning_success_rate=_summary_number(
                    values.get("planning_success_rate"), "planning_success_rate", path
                ),
                execution_success_rate=(
                    None
                    if mode == PLANNING_ONLY
                    else _summary_number(
                        values.get("execution_success_rate"),
                        "execution_success_rate",
                        path,
                    )
                ),
                task_success_rate=(
                    None
                    if mode == PLANNING_ONLY
                    else _summary_number(
                        values.get("task_success_rate"), "task_success_rate", path
                    )
                ),
                planning_time_mean_sec=_first_summary_number(
                    values,
                    ("avg_top_level_planning_time_sec", "avg_planning_time_sec"),
                    path,
                ),
                planning_time_median_sec=_summary_number(
                    values.get("median_planning_time_sec"),
                    "median_planning_time_sec",
                    path,
                ),
                planning_time_p95_sec=_summary_number(
                    values.get("p95_planning_time_sec"),
                    "p95_planning_time_sec",
                    path,
                ),
                path_length_mean_rad=_first_summary_number(
                    values,
                    ("avg_top_level_path_length_rad", "avg_path_length_rad"),
                    path,
                ),
                path_length_median_rad=_summary_number(
                    values.get("median_path_length_rad"),
                    "median_path_length_rad",
                    path,
                ),
                tracking_error_mean_rad=(
                    None
                    if mode == PLANNING_ONLY
                    else _summary_number(
                        values.get("avg_max_joint_tracking_error_rad"),
                        "avg_max_joint_tracking_error_rad",
                        path,
                    )
                ),
                final_xy_error_mean_m=(
                    None
                    if mode == PLANNING_ONLY
                    else _summary_number(
                        values.get("avg_final_xy_error_m"),
                        "avg_final_xy_error_m",
                        path,
                    )
                ),
                final_z_error_mean_m=(
                    None
                    if mode == PLANNING_ONLY
                    else _summary_number(
                        values.get("avg_final_z_error_m"),
                        "avg_final_z_error_m",
                        path,
                    )
                ),
                pose_source=str(values.get("pose_source", "")),
                perception_success_rate=_summary_number(
                    values.get("perception_success_rate"),
                    "perception_success_rate",
                    path,
                ),
                perception_latency_mean_sec=_summary_number(
                    values.get("avg_perception_latency_sec"),
                    "avg_perception_latency_sec",
                    path,
                ),
                perception_latency_median_sec=_summary_number(
                    values.get("median_perception_latency_sec"),
                    "median_perception_latency_sec",
                    path,
                ),
                perception_latency_p95_sec=_summary_number(
                    values.get("p95_perception_latency_sec"),
                    "p95_perception_latency_sec",
                    path,
                ),
                object_position_error_mean_m=_nested_summary_number(
                    values, "object_position_error_m", "mean", path
                ),
                object_position_error_p95_m=_nested_summary_number(
                    values, "object_position_error_m", "p95", path
                ),
                target_position_error_mean_m=_nested_summary_number(
                    values, "target_position_error_m", "mean", path
                ),
                target_position_error_p95_m=_nested_summary_number(
                    values, "target_position_error_m", "p95", path
                ),
                obstacle_position_error_mean_m=_nested_summary_number(
                    values, "obstacle_position_error_m", "mean", path
                ),
                obstacle_position_error_p95_m=_nested_summary_number(
                    values, "obstacle_position_error_m", "p95", path
                ),
                failure_counts=failure_counts,
                metric_sample_counts={
                    "planning_success_rate": None,
                    "execution_success_rate": None,
                    "task_success_rate": None,
                    "planning_time": None,
                    "path_length": None,
                    "tracking_error": None,
                    "final_xy_error": None,
                    "final_z_error": None,
                    "perception_success_rate": None,
                    "perception_latency": None,
                    "object_position_error": None,
                    "target_position_error": None,
                    "obstacle_position_error": None,
                },
                metric_origins={
                    "planning_time": "reported_aggregate",
                    "path_length": "reported_aggregate",
                    "tracking_error": "reported_aggregate_or_unavailable",
                    "placement_error": "reported_aggregate_or_unavailable",
                    "perception": "reported_aggregate_or_unavailable",
                },
            )
        )

    modes = sorted({group.evaluation_mode for group in groups})
    warnings = [
        "aggregate summary has no task seeds; paired coverage and "
        "missing-sample counts are unavailable"
    ]
    if summary.get("batch_status") not in (None, "complete"):
        warnings.append(
            f"batch_status is {summary.get('batch_status')!r}; data may be incomplete"
        )
    return LoadedDataset(
        label=label,
        path=path,
        source_kind="summary_json",
        sha256=digest,
        groups=groups,
        evaluation_modes=modes,
        episode_count=_summary_int(summary.get("total_episodes"), "total_episodes", path),
        coverage_keys=None,
        batch_status=(
            None if summary.get("batch_status") is None else str(summary["batch_status"])
        ),
        warnings=warnings,
    )


def _normalize_mode(value: Any) -> str | None:
    if value in ("independent_planning_only", "planning_only"):
        return PLANNING_ONLY
    if value == FULL_EPISODE:
        return FULL_EPISODE
    return None


def _summary_group_mode(top_mode: str | None, values: dict[str, Any]) -> str:
    scope = values.get("top_level_motion_metric_scope")
    if scope == "independent_planning_requests_total":
        return PLANNING_ONLY
    return top_mode or FULL_EPISODE


def _parse_summary_group_key(group_key: str) -> tuple[str, str, str, str]:
    parts = group_key.split("/")
    if len(parts) >= 4:
        return parts[0], parts[1], parts[2], "/".join(parts[3:])
    if len(parts) == 2:
        return "pybullet", "pybullet", parts[0], parts[1]
    return "unknown", "unknown", group_key, "unknown"


def _summary_failure_counts(value: Any, mode: str) -> dict[str, int]:
    if not isinstance(value, dict):
        return {}
    ignored = {"success"}
    if mode == PLANNING_ONLY:
        ignored.add("not_executed")
    counts: dict[str, int] = {}
    for key, count in value.items():
        if str(key) in ignored:
            continue
        try:
            parsed = int(count)
        except (TypeError, ValueError):
            continue
        if parsed > 0:
            counts[str(key)] = parsed
    return dict(sorted(counts.items()))


def _summary_int(value: Any, name: str, path: Path) -> int:
    if value is None:
        return 0
    if isinstance(value, bool):
        raise ValueError(f"summary {path} has invalid {name}")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"summary {path} has invalid {name}") from exc
    if parsed < 0:
        raise ValueError(f"summary {path} has negative {name}")
    return parsed


def _summary_number(value: Any, name: str, path: Path) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"summary {path} has invalid numeric {name}")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"summary {path} has invalid numeric {name}") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"summary {path} has non-finite {name}")
    return parsed


def _first_summary_number(
    values: dict[str, Any], names: Sequence[str], path: Path
) -> float | None:
    for name in names:
        if name in values:
            return _summary_number(values[name], name, path)
    return None


def _nested_summary_number(
    values: dict[str, Any], group: str, name: str, path: Path
) -> float | None:
    nested = values.get(group)
    if not isinstance(nested, dict):
        return None
    return _summary_number(nested.get(name), f"{group}.{name}", path)


def _display_label(
    dataset: str,
    backend: str,
    simulator: str,
    scene: str,
    planner: str,
    grasp_mode: str,
    pose_source: str,
) -> str:
    parts = [dataset, f"{backend}/{simulator}", scene, planner]
    if grasp_mode and grasp_mode != "assisted":
        parts.append(grasp_mode)
    if pose_source:
        parts.append(pose_source)
    return " · ".join(parts)


def _reported(records: Sequence[dict[str, Any]], key: str) -> list[float]:
    return [record[key] for record in records if record[key] is not None]


def _bool_rate(records: Sequence[dict[str, Any]], key: str) -> float | None:
    values = [record[key] for record in records if record[key] is not None]
    if not values:
        return None
    return sum(bool(value) for value in values) / len(values)


def _safe_mean(values: Sequence[float]) -> float | None:
    return mean(values) if values else None


def _safe_median(values: Sequence[float]) -> float | None:
    return median(values) if values else None


def _percentile(values: Iterable[float], probability: float) -> float | None:
    items = sorted(values)
    if not items:
        return None
    index = (len(items) - 1) * probability
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return items[lower]
    return items[lower] * (upper - index) + items[upper] * (index - lower)


def _coverage_key_json(key: tuple[str, int]) -> dict[str, Any]:
    return {"scene": key[0], "task_seed": key[1]}


def _write_chart(
    path: Path, chart_kind: str, title: str, groups: Sequence[ReportGroup]
) -> None:
    if chart_kind == "rates":
        if groups[0].evaluation_mode == PLANNING_ONLY:
            series = [("Planning success", "planning_success_rate", 1.0, "%")]
            subtitle = "Planning-only regression; execution and task success are not applicable."
        else:
            series = [
                ("Planning success", "planning_success_rate", 1.0, "%"),
                ("Execution success", "execution_success_rate", 1.0, "%"),
                ("Task success", "task_success_rate", 1.0, "%"),
            ]
            subtitle = "N/A means the source did not report that outcome."
        values = [
            [getattr(group, attribute) for _, attribute, _, _ in series]
            for group in groups
        ]
        _render_grouped_horizontal(
            path,
            title,
            subtitle,
            groups,
            [name for name, _, _, _ in series],
            values,
            axis_max=1.0,
            axis_label="success rate",
            value_format=lambda value: f"{value * 100:.1f}%",
            tick_format=lambda value: f"{value * 100:.0f}%",
        )
        return

    if chart_kind == "planning_time":
        values = [
            [
                _scale_optional(group.planning_time_mean_sec, 1000.0),
                _scale_optional(group.planning_time_median_sec, 1000.0),
                _scale_optional(group.planning_time_p95_sec, 1000.0),
            ]
            for group in groups
        ]
        _render_grouped_horizontal(
            path,
            title,
            "Top-level planning metric scope is preserved from each source; "
            "missing statistics are N/A.",
            groups,
            ["Mean", "Median", "P95"],
            values,
            axis_label="planning time (ms)",
            value_format=lambda value: f"{value:.3g} ms",
        )
        return

    if chart_kind == "path_length":
        values = [
            [group.path_length_mean_rad, group.path_length_median_rad]
            for group in groups
        ]
        _render_grouped_horizontal(
            path,
            title,
            "Successful plans with a reported top-level joint-path metric; "
            "missing statistics are N/A.",
            groups,
            ["Mean", "Median"],
            values,
            axis_label="joint path length (rad)",
            value_format=lambda value: f"{value:.4g} rad",
        )
        return

    if chart_kind == "place_error":
        values = [
            [
                _scale_optional(group.final_xy_error_mean_m, 1000.0),
                _scale_optional(group.final_z_error_mean_m, 1000.0),
            ]
            for group in groups
        ]
        _render_grouped_horizontal(
            path,
            title,
            "Absolute final center error; raw legacy rows may be derived from "
            "recorded final/target poses.",
            groups,
            ["XY error", "Z error"],
            values,
            axis_label="mean absolute error (mm)",
            value_format=lambda value: f"{value:.3g} mm",
        )
        return

    if chart_kind == "tracking_error":
        values = [[group.tracking_error_mean_rad] for group in groups]
        _render_grouped_horizontal(
            path,
            title,
            "Mean of reported episode maximum joint tracking errors; unavailable "
            "streams remain N/A.",
            groups,
            ["Max joint tracking error"],
            values,
            axis_label="mean error (rad)",
            value_format=lambda value: f"{value:.4g} rad",
        )
        return

    if chart_kind == "perception":
        rate_values = [
            [_scale_optional(group.perception_success_rate, 100.0)]
            for group in groups
        ]
        _render_grouped_horizontal(
            path,
            title,
            "Required-object detection rate for persisted RGB-D estimates.",
            groups,
            ["Detection success"],
            rate_values,
            axis_max=100.0,
            axis_label="detection rate (%)",
            value_format=lambda value: f"{value:.1f}%",
            tick_format=lambda value: f"{value:.0f}%",
        )
        return

    if chart_kind == "perception_latency":
        _render_grouped_horizontal(
            path,
            title,
            "EstimateScene service latency from request to completed estimate.",
            groups,
            ["Mean", "P95"],
            [
                [
                    _scale_optional(group.perception_latency_mean_sec, 1000.0),
                    _scale_optional(group.perception_latency_p95_sec, 1000.0),
                ]
                for group in groups
            ],
            axis_label="latency (ms)",
            value_format=lambda value: f"{value:.3g} ms",
        )
        return

    if chart_kind == "perception_error":
        values = [
            [
                _scale_optional(group.object_position_error_p95_m, 1000.0),
                _scale_optional(group.target_position_error_p95_m, 1000.0),
                _scale_optional(group.obstacle_position_error_p95_m, 1000.0),
            ]
            for group in groups
        ]
        _render_grouped_horizontal(
            path,
            title,
            "P95 world-frame center error; obstacle is N/A in clear scenes.",
            groups,
            ["Cube", "Target", "Obstacle"],
            values,
            axis_label="P95 position error (mm)",
            value_format=lambda value: f"{value:.3g} mm",
        )
        return

    if chart_kind == "failures":
        _render_failure_counts(
            path,
            title,
            (
                "Planning failures only; successful planning-only rows marked "
                "not_executed are excluded."
                if groups[0].evaluation_mode == PLANNING_ONLY
                else (
                    "Task-failure episodes by persisted failure_type; successful "
                    "episodes are excluded."
                )
            ),
            groups,
        )
        return
    raise ValueError(f"unknown chart kind: {chart_kind}")


def _scale_optional(value: float | None, scale: float) -> float | None:
    return None if value is None else value * scale


def _render_grouped_horizontal(
    path: Path,
    title: str,
    subtitle: str,
    groups: Sequence[ReportGroup],
    series_names: Sequence[str],
    values: Sequence[Sequence[float | None]],
    *,
    axis_label: str,
    value_format,
    axis_max: float | None = None,
    tick_format=None,
) -> None:
    numeric_values = [value for row in values for value in row if value is not None]
    upper = axis_max if axis_max is not None else _nice_upper(max(numeric_values, default=0.0))
    tick_format = tick_format or (lambda value: f"{value:.3g}")
    width = 1500
    left = min(560, max(330, max(len(group.display_label) for group in groups) * 7 + 25))
    right = 140
    legend_top = 86
    plot_top = 145
    bar_height = 16
    series_gap = 5
    group_gap = 22
    group_height = len(series_names) * (bar_height + series_gap) + group_gap
    plot_height = max(120, len(groups) * group_height)
    height = plot_top + plot_height + 105
    plot_width = width - left - right
    lines = _svg_header(width, height, title)
    lines.extend(_svg_titles(title, subtitle, width))
    lines.extend(_svg_legend(series_names, legend_top, left, width - right))
    lines.extend(
        _svg_grid(left, plot_top, plot_width, plot_height, upper, tick_format, axis_label)
    )

    for group_index, (group, row) in enumerate(zip(groups, values)):
        group_y = plot_top + group_index * group_height
        label_y = group_y + (len(series_names) * (bar_height + series_gap)) / 2
        lines.append(
            f'<text class="group-label" x="{left - 14}" y="{label_y:.1f}" '
            f'text-anchor="end">{escape(group.display_label)}</text>'
        )
        for series_index, value in enumerate(row):
            y = group_y + series_index * (bar_height + series_gap)
            color = _SERIES_COLORS[series_index % len(_SERIES_COLORS)]
            if value is None:
                lines.append(
                    f'<line x1="{left + 3}" y1="{y + bar_height / 2:.1f}" '
                    f'x2="{left + 25}" y2="{y + bar_height / 2:.1f}" '
                    'stroke="#9AA4AE" stroke-width="3"/>'
                )
                lines.append(
                    f'<text class="na-label" x="{left + 33}" y="{y + 12}">N/A</text>'
                )
                continue
            bar_width = 0.0 if upper <= 0 else max(0.0, value / upper * plot_width)
            lines.append(
                f'<rect x="{left}" y="{y}" width="{bar_width:.2f}" '
                f'height="{bar_height}" rx="3" fill="{color}"/>'
            )
            near_edge = bar_width > plot_width - 105
            text_x = left + bar_width - 6 if near_edge else left + bar_width + 7
            anchor = "end" if near_edge else "start"
            text_color = "#FFFFFF" if near_edge else "#263238"
            lines.append(
                f'<text class="value-label" x="{text_x:.2f}" y="{y + 12}" '
                f'text-anchor="{anchor}" fill="{text_color}">{escape(value_format(value))}</text>'
            )

    lines.extend(_svg_footer(width, height))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def _render_failure_counts(
    path: Path,
    title: str,
    subtitle: str,
    groups: Sequence[ReportGroup],
) -> None:
    categories = sorted(
        {name for group in groups for name in group.failure_counts},
        key=lambda name: (
            -sum(group.failure_counts.get(name, 0) for group in groups),
            name,
        ),
    )
    width = 1500
    left = min(560, max(330, max(len(group.display_label) for group in groups) * 7 + 25))
    right = 140
    legend_rows = max(1, math.ceil(max(1, len(categories)) / 4))
    plot_top = 128 + legend_rows * 25
    row_height = 48
    plot_height = max(120, len(groups) * row_height)
    height = plot_top + plot_height + 105
    plot_width = width - left - right
    totals = [sum(group.failure_counts.values()) for group in groups]
    upper = _nice_upper(max(totals, default=0))
    lines = _svg_header(width, height, title)
    lines.extend(_svg_titles(title, subtitle, width))
    if categories:
        lines.extend(
            _svg_legend(
                categories,
                86,
                left,
                width - right,
                colors=[_failure_color(index) for index in range(len(categories))],
                columns=4,
            )
        )
    else:
        lines.append(
            f'<text class="empty-label" x="{left}" y="105">No failure episodes recorded.</text>'
        )
    lines.extend(
        _svg_grid(
            left,
            plot_top,
            plot_width,
            plot_height,
            upper,
            lambda value: f"{value:.0f}",
            "failure episodes",
        )
    )

    for group_index, group in enumerate(groups):
        y = plot_top + group_index * row_height + 8
        lines.append(
            f'<text class="group-label" x="{left - 14}" y="{y + 14}" '
            f'text-anchor="end">{escape(group.display_label)}</text>'
        )
        cursor = left
        for category_index, category in enumerate(categories):
            count = group.failure_counts.get(category, 0)
            if count <= 0:
                continue
            segment_width = count / upper * plot_width if upper else 0.0
            color = _failure_color(category_index)
            lines.append(
                f'<rect x="{cursor:.2f}" y="{y}" width="{segment_width:.2f}" '
                f'height="24" fill="{color}"/>'
            )
            if segment_width >= 25:
                lines.append(
                    f'<text class="segment-label" x="{cursor + segment_width / 2:.2f}" '
                    f'y="{y + 17}" text-anchor="middle">{count}</text>'
                )
            cursor += segment_width
        total = totals[group_index]
        if total == 0:
            lines.append(
                f'<text class="na-label" x="{left + 7}" y="{y + 17}">0 failures</text>'
            )
        else:
            lines.append(
                f'<text class="value-label" x="{min(cursor + 8, width - right + 8):.2f}" '
                f'y="{y + 17}">total {total}</text>'
            )

    lines.extend(_svg_footer(width, height))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")


def _svg_header(width: int, height: int, title: str) -> list[str]:
    return [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}" role="img" aria-labelledby="chart-title">'
        ),
        f'<title id="chart-title">{escape(title)}</title>',
        "<style>",
        'text { font-family: Inter, "Noto Sans", "DejaVu Sans", sans-serif; fill: #263238; }',
        ".chart-title { font-size: 24px; font-weight: 700; }",
        ".chart-subtitle { font-size: 13px; fill: #52606D; }",
        ".legend-label, .tick-label { font-size: 12px; }",
        ".axis-label { font-size: 13px; font-weight: 600; }",
        ".group-label { font-size: 12px; font-weight: 600; }",
        ".value-label, .na-label { font-size: 11px; }",
        ".na-label, .empty-label { fill: #6B7785; font-style: italic; }",
        ".segment-label { fill: #FFFFFF; font-size: 11px; font-weight: 700; }",
        ".footer { font-size: 11px; fill: #6B7785; }",
        "</style>",
        f'<rect width="{width}" height="{height}" fill="#FFFFFF"/>',
    ]


def _svg_titles(title: str, subtitle: str, width: int) -> list[str]:
    return [
        f'<text class="chart-title" x="32" y="38">{escape(title)}</text>',
        f'<text class="chart-subtitle" x="32" y="64">{escape(subtitle)}</text>',
        f'<line x1="32" y1="72" x2="{width - 32}" y2="72" stroke="#D9E1E8"/>',
    ]


def _svg_legend(
    names: Sequence[str],
    y: int,
    left: int,
    right_edge: int,
    *,
    colors: Sequence[str] | None = None,
    columns: int | None = None,
) -> list[str]:
    if not names:
        return []
    columns = columns or len(names)
    columns = max(1, min(columns, len(names)))
    column_width = max(170, (right_edge - left) / columns)
    lines: list[str] = []
    for index, name in enumerate(names):
        row = index // columns
        column = index % columns
        x = left + column * column_width
        item_y = y + row * 25
        color = (
            colors[index]
            if colors is not None
            else _SERIES_COLORS[index % len(_SERIES_COLORS)]
        )
        lines.append(
            f'<rect x="{x:.1f}" y="{item_y - 11}" width="13" height="13" '
            f'rx="2" fill="{color}"/>'
        )
        lines.append(
            f'<text class="legend-label" x="{x + 20:.1f}" y="{item_y}">{escape(name)}</text>'
        )
    return lines


def _svg_grid(
    left: int,
    top: int,
    width: int,
    height: int,
    upper: float,
    tick_format,
    axis_label: str,
) -> list[str]:
    lines = []
    for index in range(6):
        fraction = index / 5
        x = left + fraction * width
        value = upper * fraction
        lines.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + height}" '
            'stroke="#E7ECF0" stroke-width="1"/>'
        )
        lines.append(
            f'<text class="tick-label" x="{x:.2f}" y="{top + height + 22}" '
            f'text-anchor="middle">{escape(tick_format(value))}</text>'
        )
    lines.append(
        f'<line x1="{left}" y1="{top + height}" x2="{left + width}" '
        f'y2="{top + height}" stroke="#52606D"/>'
    )
    lines.append(
        f'<text class="axis-label" x="{left + width / 2:.2f}" y="{top + height + 48}" '
        f'text-anchor="middle">{escape(axis_label)}</text>'
    )
    return lines


def _svg_footer(width: int, height: int) -> list[str]:
    return [
        f'<text class="footer" x="{width - 32}" y="{height - 18}" text-anchor="end">'
        "Recorded data only · N/A = not reported or not applicable</text>"
    ]


def _nice_upper(value: float) -> float:
    if value <= 0:
        return 1.0
    magnitude = 10 ** math.floor(math.log10(value))
    normalized = value / magnitude
    for candidate in (1.0, 2.0, 2.5, 5.0, 10.0):
        if normalized <= candidate:
            return candidate * magnitude
    return 10.0 * magnitude


def _failure_color(index: int) -> str:
    if index < len(_FAILURE_COLORS):
        return _FAILURE_COLORS[index]
    # Deterministic fallback for unusually broad failure taxonomies.
    hue = (index * 137.508) % 360
    return f"hsl({hue:.1f},55%,45%)"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Generate dependency-light SVG charts from recorded Panda EpisodeResult "
            "JSONL or benchmark summary JSON."
        )
    )
    parser.add_argument(
        "--input",
        action="append",
        required=True,
        metavar="[LABEL=]PATH",
        help="Input JSONL/JSON; repeat to compare datasets.",
    )
    parser.add_argument(
        "--output-dir", type=Path, required=True, help="New report output directory."
    )
    parser.add_argument("--title", default="Panda pick-place benchmark")
    parser.add_argument(
        "--check-paired-coverage",
        action="store_true",
        help="Compare unique (scene, task_seed) coverage across raw inputs.",
    )
    parser.add_argument(
        "--require-paired-coverage",
        action="store_true",
        help="Fail unless raw inputs have identical (scene, task_seed) coverage.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite report files already present in the output directory.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        datasets = [load_dataset(input_spec) for input_spec in args.input]
        coverage = None
        if args.check_paired_coverage or args.require_paired_coverage:
            coverage = validate_paired_coverage(datasets)
        if args.require_paired_coverage and not (
            coverage and coverage.get("available") and coverage.get("paired")
        ):
            reason = (coverage or {}).get("reason", "task coverage differs")
            if coverage and coverage.get("available"):
                reason = "task coverage differs; inspect missing_by_input"
            parser.error(f"paired coverage validation failed: {reason}")
        manifest_path = generate_report(
            datasets,
            args.output_dir.expanduser(),
            args.title,
            force=args.force,
            paired_coverage=coverage,
        )
    except (FileExistsError, OSError, ValueError) as exc:
        parser.error(str(exc))
    print(manifest_path.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
