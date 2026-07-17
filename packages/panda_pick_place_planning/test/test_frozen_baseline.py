import hashlib
import json
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).parents[3]
BASELINE_ROOT = REPOSITORY_ROOT / "baselines" / "v0.1-pybullet-baseline"


def test_frozen_v1_artifacts_match_manifest_hashes():
    manifest = json.loads((BASELINE_ROOT / "manifest.json").read_text())

    assert manifest["baseline"] == "v0.1-pybullet-baseline"
    assert manifest["episodes"] == 200
    for filename, expected in manifest["artifacts"].items():
        artifact = BASELINE_ROOT / filename
        assert artifact.is_file(), f"missing frozen artifact: {filename}"
        actual = hashlib.sha256(artifact.read_bytes()).hexdigest()
        assert f"sha256:{actual}" == expected
