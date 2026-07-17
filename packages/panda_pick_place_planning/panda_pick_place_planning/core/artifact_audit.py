"""Fail-closed audit for curated benchmark artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from panda_pick_place_planning.core.benchmark import summarize_results
from panda_pick_place_planning.core.schemas import EpisodeResult


SUMMARY_KEYS = ("schema_version", "total_episodes", "groups", "backend_groups")
GAZEBO_REQUIRED_METRICS = (
    "max_joint_tracking_error_rad",
    "final_xy_error_m",
    "final_z_error_m",
    "final_object_speed_mps",
    "execution_collision",
)
MOCK_NULL_METRICS = (
    "max_joint_tracking_error_rad",
    "final_xy_error_m",
    "final_z_error_m",
    "final_object_speed_mps",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_episode_records(path: Path) -> list[dict]:
    """Read strict non-empty JSONL objects with useful line diagnostics."""
    records = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL line {line_number}: {exc}") from exc
            if not isinstance(record, dict):
                raise ValueError(f"JSONL line {line_number} is not an object")
            records.append(record)
    if not records:
        raise ValueError("episode JSONL is empty")
    return records


def audit_episode_semantics(records: list[dict], mode: str) -> None:
    """Enforce final-result nullability and execution semantics."""
    if mode not in {"moveit_mock", "gazebo_assisted"}:
        raise ValueError(f"unsupported audit mode: {mode}")
    identifiers = set()
    coverage = set()
    for index, record in enumerate(records):
        identifier = int(record.get("episode_id", -1))
        if identifier in identifiers:
            raise ValueError(f"duplicate episode_id: {identifier}")
        identifiers.add(identifier)
        coverage.add((str(record.get("scene", "")), int(record.get("task_seed", -1))))
        if int(record.get("schema_version", 0)) != 2:
            raise ValueError(f"episode {index} is not schema v2")
        if not bool(record.get("planning_scene_sync_success", False)):
            raise ValueError(f"episode {index} reports PlanningScene sync failure")
        if mode == "moveit_mock":
            if record.get("simulator") != "moveit_mock":
                raise ValueError(f"episode {index} is not moveit_mock")
            if bool(record.get("task_success")) or bool(record.get("execution_success")):
                raise ValueError(f"episode {index} falsely reports mock execution")
            if record.get("failure_type") != "not_executed":
                raise ValueError(f"episode {index} lacks not_executed semantics")
            for field in MOCK_NULL_METRICS:
                if record.get(field) is not None:
                    raise ValueError(f"episode {index} mock metric {field} is not null")
        else:
            if record.get("simulator") != "gazebo":
                raise ValueError(f"episode {index} is not Gazebo")
            if record.get("grasp_mode") != "assisted":
                raise ValueError(f"episode {index} is not assisted grasp")
            for field in GAZEBO_REQUIRED_METRICS:
                if record.get(field) is None:
                    raise ValueError(f"episode {index} Gazebo metric {field} is null")
            if record.get("failure_type") == "runtime_error":
                raise ValueError(f"episode {index} has unclassified runtime_error")
    if len(coverage) != len(records):
        raise ValueError("duplicate (scene, task_seed) coverage")


def audit_summary(records: list[dict], summary_path: Path) -> dict:
    """Recompute every core summary key and require exact equality."""
    persisted = json.loads(summary_path.read_text(encoding="utf-8"))
    recomputed = summarize_results(
        EpisodeResult.from_dict(record) for record in records
    )
    for key in SUMMARY_KEYS:
        if persisted.get(key) != recomputed.get(key):
            raise ValueError(f"summary mismatch for key: {key}")
    if persisted.get("batch_status") != "complete":
        raise ValueError("batch_status is not complete")
    expected = int(persisted.get("expected_episodes", -1))
    received = int(persisted.get("received_episodes", -1))
    if expected != len(records) or received != len(records):
        raise ValueError("summary episode counts do not match JSONL")
    return persisted


def audit_report(report_dir: Path, episodes_path: Path) -> list[Path]:
    """Check the report manifest source hash and parse every declared SVG."""
    manifest_path = report_dir / "report_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected_hash = _sha256(episodes_path)
    sources = manifest.get("sources", [])
    if not any(source.get("sha256") == expected_hash for source in sources):
        raise ValueError("report manifest does not hash the episode JSONL")
    paths = [manifest_path]
    for chart in manifest.get("charts", []):
        chart_path = report_dir / str(chart["path"])
        ET.parse(chart_path)
        paths.append(chart_path)
    if len(paths) == 1:
        raise ValueError("report manifest declares no SVG charts")
    return paths


def write_artifact_manifest(root: Path, paths: list[Path]) -> Path:
    """Hash final inputs and generated report files without self-reference."""
    unique_paths = sorted({path.resolve() for path in paths})
    artifacts = {
        str(path.relative_to(root.resolve())): {
            "sha256": _sha256(path),
            "bytes": path.stat().st_size,
        }
        for path in unique_paths
    }
    output = root / "artifact_manifest.json"
    output.write_text(
        json.dumps(
            {"schema_version": 1, "artifacts": artifacts},
            indent=2,
            ensure_ascii=False,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return output


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--expected-episodes", type=int, required=True)
    parser.add_argument(
        "--mode", choices=("moveit_mock", "gazebo_assisted"), required=True
    )
    parser.add_argument("--extra", action="append", type=Path, default=[])
    parsed = parser.parse_args(args)

    records = load_episode_records(parsed.episodes)
    if len(records) != parsed.expected_episodes:
        parser.error(
            f"expected {parsed.expected_episodes} episodes, found {len(records)}"
        )
    audit_episode_semantics(records, parsed.mode)
    audit_summary(records, parsed.summary)
    report_paths = audit_report(parsed.report_dir, parsed.episodes)
    root = parsed.episodes.parent
    all_paths = [
        parsed.episodes,
        parsed.summary,
        *parsed.extra,
        *report_paths,
    ]
    for path in all_paths:
        if not path.is_file():
            parser.error(f"artifact does not exist: {path}")
        try:
            path.resolve().relative_to(root.resolve())
        except ValueError:
            parser.error(f"artifact is outside result root: {path}")
    output = write_artifact_manifest(root, all_paths)
    print(output.resolve())
    return 0
