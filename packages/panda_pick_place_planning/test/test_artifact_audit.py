import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from panda_pick_place_planning.core.artifact_audit import (
    audit_episode_semantics,
    audit_report,
)


def mock_record():
    return {
        "schema_version": 2,
        "episode_id": 0,
        "task_seed": 42,
        "scene": "clear",
        "simulator": "moveit_mock",
        "planning_scene_sync_success": True,
        "planning_success": True,
        "task_success": False,
        "execution_success": False,
        "failure_type": "not_executed",
        "max_joint_tracking_error_rad": None,
        "final_xy_error_m": None,
        "final_z_error_m": None,
        "final_object_speed_mps": None,
    }


class ArtifactAuditTest(unittest.TestCase):
    def test_mock_null_semantics_pass_and_fake_execution_fails(self):
        record = mock_record()
        audit_episode_semantics([record], "moveit_mock")
        record["task_success"] = True
        with self.assertRaisesRegex(ValueError, "falsely reports mock execution"):
            audit_episode_semantics([record], "moveit_mock")

    def test_gazebo_null_metric_fails(self):
        record = mock_record()
        record.update(
            {
                "simulator": "gazebo",
                "grasp_mode": "assisted",
                "failure_type": "",
                "execution_collision": False,
                "max_joint_tracking_error_rad": 0.01,
                "final_xy_error_m": 0.01,
                "final_z_error_m": 0.001,
                "final_object_speed_mps": None,
            }
        )
        with self.assertRaisesRegex(ValueError, "final_object_speed_mps is null"):
            audit_episode_semantics([record], "gazebo_assisted")

    def test_report_svg_and_source_hash_are_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = root / "episodes.jsonl"
            episodes.write_text("{}\n", encoding="utf-8")
            import hashlib

            digest = hashlib.sha256(episodes.read_bytes()).hexdigest()
            report = root / "report"
            report.mkdir()
            svg = report / "chart.svg"
            svg.write_text("<svg xmlns='http://www.w3.org/2000/svg'/>", encoding="utf-8")
            (report / "report_manifest.json").write_text(
                json.dumps(
                    {
                        "sources": [{"sha256": digest}],
                        "charts": [{"path": "chart.svg"}],
                    }
                ),
                encoding="utf-8",
            )
            paths = audit_report(report, episodes)
            self.assertIn(svg, paths)
            ET.parse(svg)


if __name__ == "__main__":
    unittest.main()
