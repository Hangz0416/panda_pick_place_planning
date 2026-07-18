import json
from types import SimpleNamespace

from panda_gz_bringup import launch_supervisor


def test_supervisor_retries_one_readiness_failure(tmp_path, monkeypatch):
    output = tmp_path / "run" / "episodes.jsonl"
    status_path = output.parent / "run_status.json"
    attempts = []

    def fake_run(command, check):
        assert not check
        attempts.append(command)
        status_path.parent.mkdir(parents=True, exist_ok=True)
        if len(attempts) == 1:
            status_path.write_text(
                json.dumps({"readiness": "failed"}), encoding="utf-8"
            )
            return SimpleNamespace(returncode=1)
        status_path.write_text(
            json.dumps({"readiness": "ready", "episode_status": "success"}),
            encoding="utf-8",
        )
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(launch_supervisor.subprocess, "run", fake_run)

    result = launch_supervisor.main(
        ["--output-path", str(output), "--max-attempts", "2", "--headless"]
    )

    assert result == 0
    assert len(attempts) == 2


def test_supervisor_does_not_retry_an_episode_failure(tmp_path, monkeypatch):
    output = tmp_path / "run" / "episodes.jsonl"
    status_path = output.parent / "run_status.json"
    attempts = []

    def fake_run(command, check):
        assert not check
        attempts.append(command)
        status_path.parent.mkdir(parents=True, exist_ok=True)
        status_path.write_text(
            json.dumps(
                {
                    "readiness": "ready",
                    "episode_status": "failed",
                    "failure_type": "no_feasible_object",
                }
            ),
            encoding="utf-8",
        )
        return SimpleNamespace(returncode=3)

    monkeypatch.setattr(launch_supervisor.subprocess, "run", fake_run)

    result = launch_supervisor.main(
        ["--output-path", str(output), "--max-attempts", "2"]
    )

    assert result == 3
    assert len(attempts) == 1
    status = json.loads(status_path.read_text(encoding="utf-8"))
    assert status["teardown"] == "abnormal_exit"
    assert status["launch_exit_code"] == 3
