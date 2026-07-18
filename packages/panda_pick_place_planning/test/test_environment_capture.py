from pathlib import Path
from types import SimpleNamespace
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from panda_pick_place_planning.core.environment_capture import (
    capture_environment,
    git_state,
    source_status_dirty,
    source_tree_hash,
)


class EnvironmentCaptureTest(unittest.TestCase):
    def test_source_dirty_state_ignores_generated_evidence_only(self):
        self.assertFalse(
            source_status_dirty(
                "?? results/runs/run/episodes.jsonl\n"
                " M docs/media/demo.mp4"
            )
        )
        self.assertTrue(source_status_dirty(" M panda_pick_place/core.py"))

    def test_git_state_rejects_non_repository(self):
        with patch(
            "panda_pick_place_planning.core.environment_capture._command",
            return_value=None,
        ):
            with self.assertRaisesRegex(ValueError, "not a git repository"):
                git_state(Path("/tmp"))

    def test_capture_keeps_physics_model_unvalidated(self):
        with patch(
            "panda_pick_place_planning.core.environment_capture.git_state",
            return_value={"commit": "a" * 40, "branch": "test", "dirty": False},
        ), patch(
            "panda_pick_place_planning.core.environment_capture.package_versions",
            return_value={},
        ):
            document = capture_environment(
                Path("/tmp"),
                "gazebo",
                object_variant="rect_prism",
                workpiece_count=3,
                pose_source="rgbd",
                perception_profile="combined",
            )
        self.assertEqual(document["ros_distro"], "humble")
        self.assertFalse(document["experiment"]["physics_model_validated"])
        self.assertFalse(document["inertial_source"]["official_properties"])
        self.assertEqual(
            document["experiment"]["object_variant"], "rect_prism"
        )
        self.assertEqual(document["experiment"]["workpiece_count"], 3)
        self.assertEqual(
            document["experiment"]["perception_profile"], "combined"
        )

    def test_three_workpieces_require_rectangular_variant(self):
        with self.assertRaisesRegex(ValueError, "three workpieces require"):
            capture_environment(
                Path("/tmp"),
                "gazebo",
                object_variant="cube",
                workpiece_count=3,
            )

    def test_source_tree_hash_excludes_recorded_results_and_media(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "results").mkdir()
            (root / "docs" / "media").mkdir(parents=True)
            (root / "source.py").write_text("source-v1")
            (root / "results" / "episode.jsonl").write_text("result-v1")
            (root / "docs" / "media" / "demo.mp4").write_text("media-v1")
            listing = SimpleNamespace(
                returncode=0,
                stdout=(
                    b"source.py\0results/episode.jsonl\0"
                    b"docs/media/demo.mp4\0"
                ),
            )
            with patch(
                "panda_pick_place_planning.core.environment_capture.subprocess.run",
                return_value=listing,
            ):
                first_hash, first_count = source_tree_hash(root)
                (root / "results" / "episode.jsonl").write_text("result-v2")
                excluded_hash, excluded_count = source_tree_hash(root)
                (root / "source.py").write_text("source-v2")
                changed_hash, changed_count = source_tree_hash(root)
        self.assertEqual(first_count, 1)
        self.assertEqual(excluded_count, 1)
        self.assertEqual(changed_count, 1)
        self.assertEqual(first_hash, excluded_hash)
        self.assertNotEqual(first_hash, changed_hash)


if __name__ == "__main__":
    unittest.main()
