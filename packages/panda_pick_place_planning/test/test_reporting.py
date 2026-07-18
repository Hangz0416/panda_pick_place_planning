"""Tests for report generation using synthetic fixtures only.

The values in this module are deliberately fabricated test inputs and are never
written to the repository's results or baseline directories.
"""

import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree

from panda_pick_place_planning.core.reporting import (
    FULL_EPISODE,
    LoadedDataset,
    PLANNING_ONLY,
    generate_report,
    load_dataset,
    main,
    validate_paired_coverage,
)


def synthetic_episode(
    episode_id,
    task_seed,
    *,
    scene="clear",
    simulator="gazebo",
    planning_success=True,
    task_success=True,
    failure_type="",
):
    """Return synthetic test data, not an experimental result."""

    return {
        "schema_version": 2,
        "episode_id": episode_id,
        "task_seed": task_seed,
        "planner": "rrt_connect",
        "planner_id": "RRTConnectkConfigDefault",
        "planner_seed": 99000 + task_seed,
        "scene": scene,
        "backend": "moveit",
        "simulator": simulator,
        "grasp_mode": "assisted",
        "planning_success": planning_success,
        "planning_time_sec": 0.02 + episode_id * 0.01,
        "path_length_rad": 1.0 + episode_id * 0.1,
        "execution_success": task_success,
        "task_success": task_success,
        "failure_type": failure_type,
        "object_final_position": [0.51, 0.19, 0.03],
        "target_position": [0.50, 0.20, 0.025],
    }


def write_jsonl(path, records):
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )


class ReportingTest(unittest.TestCase):
    def test_manifest_keeps_relative_source_paths_portable(self):
        dataset = LoadedDataset(
            label="portable",
            path=Path("results/example.jsonl"),
            source_kind="episode_jsonl",
            sha256="0" * 64,
            groups=[],
            evaluation_modes=[FULL_EPISODE],
            episode_count=0,
            coverage_keys=set(),
        )

        self.assertEqual(
            dataset.manifest_source()["path"], "results/example.jsonl"
        )

    def test_full_episode_jsonl_generates_valid_svg_and_tracks_missing_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "synthetic_test_episodes.jsonl"
            first = synthetic_episode(0, 42)
            first["max_joint_tracking_error_rad"] = 0.02
            second = synthetic_episode(
                1,
                43,
                scene="obstacle",
                task_success=False,
                failure_type="execution_collision_object_obstacle",
            )
            second["max_joint_tracking_error_rad"] = None
            write_jsonl(input_path, [first, second])

            dataset = load_dataset(f"synthetic={input_path}")
            manifest_path = generate_report([dataset], root / "report")
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertEqual(dataset.evaluation_modes, [FULL_EPISODE])
            self.assertEqual(len(manifest["groups"]), 2)
            sample_counts = {
                group["group_key"]: group["metric_sample_counts"]["tracking_error"]
                for group in manifest["groups"]
            }
            self.assertEqual(sum(sample_counts.values()), 1)
            self.assertTrue(
                any(
                    group["metric_origins"]["placement_error"]
                    == "derived_from_positions"
                    for group in manifest["groups"]
                )
            )
            expected = {
                "full_episode_success_rates.svg",
                "full_episode_planning_time.svg",
                "full_episode_path_length.svg",
                "full_episode_failure_counts.svg",
                "full_episode_place_error.svg",
                "full_episode_tracking_error.svg",
            }
            self.assertEqual(
                {chart["path"] for chart in manifest["charts"]}, expected
            )
            for filename in expected:
                ElementTree.parse(root / "report" / filename)
            tracking_svg = (root / "report" / "full_episode_tracking_error.svg").read_text(
                encoding="utf-8"
            )
            self.assertIn("N/A", tracking_svg)

    def test_planning_only_does_not_turn_not_executed_into_task_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "synthetic_planning_only.jsonl"
            record = synthetic_episode(
                0, 42, simulator="moveit_mock", task_success=False, failure_type="not_executed"
            )
            record["execution_success"] = False
            record["final_xy_error_m"] = 0.0
            write_jsonl(input_path, [record])

            dataset = load_dataset(str(input_path))
            group = dataset.groups[0]
            manifest_path = generate_report([dataset], root / "report")
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertEqual(group.evaluation_mode, PLANNING_ONLY)
            self.assertIsNone(group.execution_success_rate)
            self.assertIsNone(group.task_success_rate)
            self.assertIsNone(group.final_xy_error_mean_m)
            self.assertEqual(group.failure_counts, {})
            self.assertNotIn(
                "full_episode_place_error.svg",
                {chart["path"] for chart in manifest["charts"]},
            )
            success_svg = (root / "report" / "planning_only_success_rates.svg").read_text(
                encoding="utf-8"
            )
            self.assertIn("execution and task success are not applicable", success_svg)
            self.assertNotIn("Task success</text>", success_svg)

    def test_rgbd_records_generate_perception_charts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "synthetic_rgbd.jsonl"
            record = synthetic_episode(0, 42, scene="obstacle")
            record.update(
                {
                    "pose_source": "rgbd",
                    "perception_success": True,
                    "perception_latency_sec": 0.05,
                    "object_position_error_m": 0.002,
                    "target_position_error_m": 0.003,
                    "obstacle_position_error_m": 0.004,
                    "schema_version": 5,
                    "object_variant": "rect_prism",
                    "perception_profile": "nominal",
                    "object_yaw_error_rad": 0.03,
                    "grasp_selection_success": True,
                    "grasp_selection_latency_sec": 0.07,
                    "grasp_candidate_count": 2,
                    "selected_grasp_yaw_rad": -0.2,
                    "selected_grasp_path_length_rad": 1.1,
                    "detected_workpiece_count": 3,
                    "selected_object_id": "workpiece_1",
                    "target_selection_success": True,
                    "target_selection_latency_sec": 0.08,
                    "selected_object_score": 1.14,
                    "total_candidate_count": 6,
                    "feasible_candidate_count": 5,
                    "selected_object_position_error_m": 0.002,
                    "selected_object_yaw_error_rad": 0.03,
                    "max_nonselected_displacement_m": 0.001,
                }
            )
            write_jsonl(input_path, [record])

            manifest_path = generate_report(
                [load_dataset(f"rgbd-test={input_path}")], root / "report"
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            chart_names = {chart["path"] for chart in manifest["charts"]}
            expected = {
                "full_episode_perception.svg",
                "full_episode_perception_latency.svg",
                "full_episode_perception_error.svg",
                "full_episode_yaw_error.svg",
                "full_episode_grasp_selection.svg",
                "full_episode_grasp_candidate_path.svg",
                "full_episode_target_selection.svg",
                "full_episode_candidate_counts.svg",
                "full_episode_selection_cost.svg",
                "full_episode_nonselected_motion.svg",
            }
            self.assertTrue(expected.issubset(chart_names))
            self.assertEqual(manifest["groups"][0]["pose_source"], "rgbd")
            self.assertEqual(
                manifest["groups"][0]["object_variant"], "rect_prism"
            )
            self.assertEqual(
                manifest["groups"][0]["grasp_selection_success_rate"], 1.0
            )
            self.assertEqual(
                manifest["groups"][0]["target_selection_success_rate"], 1.0
            )
            self.assertEqual(
                manifest["groups"][0]["feasible_candidate_count_mean"], 5.0
            )
            for filename in expected:
                ElementTree.parse(root / "report" / filename)

    def test_legacy_summary_preserves_unreported_statistics_as_none(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary_path = root / "synthetic_legacy_summary.json"
            summary_path.write_text(
                json.dumps(
                    {
                        "total_episodes": 2,
                        "groups": {
                            "clear/linear": {
                                "episodes": 2,
                                "planning_success_rate": 1.0,
                                "task_success_rate": 1.0,
                                "avg_planning_time_sec": 0.01,
                                "avg_path_length_rad": 0.4,
                                "failure_counts": {"success": 2},
                            }
                        },
                    }
                ),
                encoding="utf-8",
            )

            dataset = load_dataset(f"legacy-test={summary_path}")
            group = dataset.groups[0]
            generate_report([dataset], root / "report")

            self.assertIsNone(group.planning_time_median_sec)
            self.assertIsNone(group.planning_time_p95_sec)
            self.assertIsNone(group.tracking_error_mean_rad)
            planning_svg = (root / "report" / "full_episode_planning_time.svg").read_text(
                encoding="utf-8"
            )
            self.assertGreaterEqual(planning_svg.count("N/A"), 2)

    def test_paired_coverage_reports_exact_missing_scene_seed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first_path = root / "synthetic_first.jsonl"
            second_path = root / "synthetic_second.jsonl"
            write_jsonl(
                first_path,
                [synthetic_episode(0, 42), synthetic_episode(1, 43, scene="obstacle")],
            )
            write_jsonl(second_path, [synthetic_episode(0, 42)])

            validation = validate_paired_coverage(
                [
                    load_dataset(f"first-test={first_path}"),
                    load_dataset(f"second-test={second_path}"),
                ]
            )

            self.assertTrue(validation["available"])
            self.assertFalse(validation["paired"])
            self.assertEqual(
                validation["missing_by_input"]["second-test"],
                [{"scene": "obstacle", "task_seed": 43}],
            )

    def test_cli_requires_paired_coverage_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first_path = root / "synthetic_first.jsonl"
            second_path = root / "synthetic_second.jsonl"
            write_jsonl(first_path, [synthetic_episode(0, 42)])
            write_jsonl(second_path, [synthetic_episode(0, 42)])
            report_dir = root / "report"

            result = main(
                [
                    "--input",
                    f"first-test={first_path}",
                    "--input",
                    f"second-test={second_path}",
                    "--output-dir",
                    str(report_dir),
                    "--require-paired-coverage",
                ]
            )
            self.assertEqual(result, 0)
            manifest = json.loads(
                (report_dir / "report_manifest.json").read_text(encoding="utf-8")
            )
            self.assertTrue(manifest["paired_coverage"]["paired"])
            with self.assertRaises(SystemExit) as raised:
                main(
                    [
                        "--input",
                        str(first_path),
                        "--output-dir",
                        str(report_dir),
                    ]
                )
            self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
