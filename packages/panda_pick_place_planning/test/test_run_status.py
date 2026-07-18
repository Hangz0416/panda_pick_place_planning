import json

from panda_pick_place_planning.core.run_status import (
    finalize_batch_run,
    record_teardown_exit,
    write_readiness_failure,
)


def _write_artifacts(tmp_path, rows):
    output = tmp_path / "episodes.jsonl"
    output.write_text(
        "".join(json.dumps(row) + "\n" for row in rows),
        encoding="utf-8",
    )
    summary = tmp_path / "summary.json"
    summary.write_text(
        json.dumps({"total_episodes": len(rows)}),
        encoding="utf-8",
    )
    return output, summary


def test_batch_status_requires_complete_durable_episode_and_summary_files(tmp_path):
    output, summary = _write_artifacts(
        tmp_path,
        [{"task_success": True}, {"task_success": True}],
    )

    status = finalize_batch_run(
        output,
        summary,
        expected_episodes=2,
        batch_exit_code=0,
    )

    assert status["result_saved"] is True
    assert status["episode_status"] == "success"
    assert status["teardown"] == "requested"
    assert json.loads((tmp_path / "run_status.json").read_text()) == status


def test_episode_failure_is_not_misclassified_as_teardown_failure(tmp_path):
    output, summary = _write_artifacts(
        tmp_path,
        [{"task_success": True}, {"task_success": False}],
    )

    status = finalize_batch_run(
        output,
        summary,
        expected_episodes=2,
        batch_exit_code=0,
    )

    assert status["result_saved"] is True
    assert status["episode_status"] == "failed"
    assert status["failure_type"] == "episode_failures_present"
    assert status["teardown"] == "requested"


def test_teardown_exit_updates_only_run_status(tmp_path):
    output, summary = _write_artifacts(tmp_path, [{"task_success": True}])
    finalize_batch_run(
        output,
        summary,
        expected_episodes=1,
        batch_exit_code=0,
    )

    status = record_teardown_exit(
        output,
        process_name="move_group-13",
        returncode=-11,
    )

    assert status["episode_status"] == "success"
    assert status["result_saved"] is True
    assert status["teardown"] == "abnormal_exit"
    assert status["teardown_events"] == [
        {"process": "move_group-13", "returncode": -11}
    ]


def test_readiness_failure_is_separate_from_episode_status(tmp_path):
    output = tmp_path / "episodes.jsonl"
    status = write_readiness_failure(output, exit_code=7)

    assert status["readiness"] == "failed"
    assert status["episode_status"] == "not_started"
    assert status["result_saved"] is False
