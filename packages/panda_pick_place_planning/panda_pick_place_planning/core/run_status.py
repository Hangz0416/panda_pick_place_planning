"""Durable run-level status kept separate from episode outcomes."""

from __future__ import annotations

import json
from pathlib import Path
import time
from typing import Any


def _atomic_write(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def finalize_batch_run(
    output_path: str | Path,
    summary_path: str | Path,
    *,
    expected_episodes: int,
    batch_exit_code: int,
) -> dict[str, Any]:
    """Verify durable artifacts before a batch is allowed to shut down."""
    output = Path(output_path).expanduser()
    summary = Path(summary_path).expanduser()
    records: list[dict[str, Any]] = []
    parse_error = ""
    if output.exists():
        try:
            records = [
                json.loads(line)
                for line in output.read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
        except (OSError, ValueError) as exc:
            parse_error = str(exc)
    summary_total = None
    if summary.exists():
        try:
            summary_total = int(
                json.loads(summary.read_text(encoding="utf-8"))["total_episodes"]
            )
        except (KeyError, OSError, TypeError, ValueError):
            summary_total = None

    actual = len(records)
    result_saved = (
        not parse_error
        and actual == int(expected_episodes)
        and summary_total == int(expected_episodes)
    )
    failed_episodes = sum(not bool(item.get("task_success")) for item in records)
    episode_status = (
        "success"
        if result_saved and batch_exit_code == 0 and failed_episodes == 0
        else "failed"
    )
    failure_type = ""
    if not result_saved:
        failure_type = "result_save_incomplete"
    elif batch_exit_code != 0:
        failure_type = "batch_process_failed"
    elif failed_episodes:
        failure_type = "episode_failures_present"
    status = {
        "readiness": "ready",
        "episode_status": episode_status,
        "failure_type": failure_type,
        "expected_episodes": int(expected_episodes),
        "saved_episodes": actual,
        "failed_episodes": failed_episodes,
        "result_saved": result_saved,
        "result_path": str(output.resolve()),
        "summary_path": str(summary.resolve()),
        "batch_exit_code": int(batch_exit_code),
        "teardown": "requested",
        "teardown_events": [],
        "parse_error": parse_error,
        "updated_unix_sec": time.time(),
    }
    _atomic_write(output.parent / "run_status.json", status)
    return status


def write_readiness_failure(
    output_path: str | Path, *, exit_code: int
) -> dict[str, Any]:
    output = Path(output_path).expanduser()
    status = {
        "readiness": "failed",
        "readiness_exit_code": int(exit_code),
        "episode_status": "not_started",
        "failure_type": "readiness_failed",
        "result_saved": False,
        "result_path": str(output.resolve()),
        "teardown": "requested",
        "teardown_events": [],
        "updated_unix_sec": time.time(),
    }
    _atomic_write(output.parent / "run_status.json", status)
    return status


def record_teardown_exit(
    output_path: str | Path, *, process_name: str, returncode: int
) -> dict[str, Any] | None:
    """Record teardown failures without changing saved episode truth."""
    if int(returncode) in (0, -2):
        return None
    output = Path(output_path).expanduser()
    status_path = output.parent / "run_status.json"
    if not status_path.exists():
        return None
    try:
        status = json.loads(status_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    event = {"process": str(process_name), "returncode": int(returncode)}
    events = list(status.get("teardown_events", []))
    if event not in events:
        events.append(event)
    status["teardown_events"] = events
    status["teardown"] = "abnormal_exit"
    status["updated_unix_sec"] = time.time()
    _atomic_write(status_path, status)
    return status
