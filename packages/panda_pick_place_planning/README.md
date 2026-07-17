# panda_pick_place_planning

This package owns the shared Python pick/place FSM, the PyBullet and MoveIt
backend adapters, typed ROS task/result conversion, and sequential batch
orchestration.

## Verified status

- All five packages (`interfaces`, `planning`, `moveit_bridge`, `moveit_config`,
  `gz_bringup`) clean-build successfully. The final suite is 178/178 with zero
  errors, failures, or skipped tests.
- The real PyBullet V2 seed-42 benchmark contains 200 episodes. Clear Linear
  and RRT-Connect, plus obstacle RRT-Connect, are 50/50 successful; obstacle
  Linear is 0/50 with 50 `planning_collision` results. There are zero collision
  episodes. The seed-42 clear and obstacle demos also succeeded.

| Scene/planner | Success | Mean plan | Mean path | Mean execution |
| --- | ---: | ---: | ---: | ---: |
| clear/Linear | 100% | 1.750 ms | 0.778358 rad | 0.085929 s |
| clear/RRT-Connect | 100% | 1.922 ms | 0.778358 rad | 0.079812 s |
| obstacle/Linear | 0% | 0.560 ms | N/A | 0.052244 s |
| obstacle/RRT-Connect | 100% | 33.157 ms | 1.312717 rad | 0.093104 s |

The raw V1/V2 paired report at
`results/v2/reports/v1_v2_pybullet_compare/` records coverage 100/100,
`paired=true`, empty missing sets, and no success/failure classification drift.
This does not claim equal timing or path-length values. The final V2 data and
single-version report are under `results/v2/pybullet_shared_geometry_200_final/`
and `results/v2/reports/pybullet_final/`.

The complete 200-episode V1 raw JSONL/CSV and summary are frozen under
`baselines/v0.1-pybullet-baseline/`; a regression test recomputes every
artifact SHA256 recorded by the manifest.

- Final-code MoveIt planning-only smoke and 5+5 passed, followed by 50 clear +
  50 obstacle at 100/100 planning success and zero PlanningScene sync errors.
  All rows retain `task_success=false` and `failure_type=not_executed`.
- Gazebo assisted 5+5 passed, followed by 50 clear + 50 obstacle at 100/100
  full task success, zero collision episodes, zero scene-sync errors, and zero
  unclassified runtime errors. Obstacle/seed 42 deterministic reset passed
  10/10. These are guarded DetachableJoint results, not physics-grasp results.
- Videos and physics grasp are not delivered. Humble MoveIt 2.5.9 can segfault
  in `move_group` teardown after a completed batch; the episode writers finish
  cleanly and the persisted result still has to pass the independent audit.

## MoveIt planning-only batch

Start the MoveIt mock stack first (the batch launch deliberately does not start
or duplicate `move_group`, controllers, or the C++ bridge):

```bash
ros2 launch panda_moveit_config mock_demo.launch.py use_rviz:=false
```

Run the required five-episode-per-scene quick check in another sourced shell:

```bash
ros2 launch panda_pick_place_planning batch_benchmark.launch.py \
  episodes_per_scene:=5 \
  backend:=moveit simulator:=moveit_mock \
  use_gazebo:=false moveit_execute:=false \
  output_path:=results/mock_quick.jsonl \
  summary_path:=results/mock_quick_summary.json
```

Only after the quick check and runtime/bridge checks pass, run the formal
`50 clear + 50 obstacle` planning batch:

```bash
ros2 launch panda_pick_place_planning batch_benchmark.launch.py \
  episodes_per_scene:=50 \
  backend:=moveit simulator:=moveit_mock \
  use_gazebo:=false moveit_execute:=false \
  output_path:=results/mock_100.jsonl \
  summary_path:=results/mock_100_summary.json
```

The default `scenes_csv:=clear,obstacle` and `planner:=rrt_connect` produce 10
episodes for the quick check and 100 for the formal run. The batch publisher
waits for the matching `EpisodeResult` before publishing the next task. Its
result subscription is volatile, so a retained result from an earlier run
cannot advance a new batch. A timeout writes an incomplete summary with
`batch_status: timeout`; it does not invent a failed episode.

For `simulator:=moveit_mock`, the node uses an independent planning-regression
runner. It requests home, pre-grasp, transfer, and place plans from the current
mock state, but never executes a trajectory, attaches an object, releases an
object, or changes object state. A successful regression has
`planning_success=true`, `task_success=false`, `execution_success=false`, and
`failure_type=not_executed`. It is not Gazebo physics execution or a real grasp
result. Per-episode data is written by `evaluator_node`; the batch sequencer
writes `evaluation_mode=independent_planning_only` in the aggregate summary and
publishes `/benchmark_complete`.

## Metric scope

To preserve the frozen V1 result meaning, full PyBullet and MoveIt + Gazebo
episodes keep the top-level `planning_time_sec`, `path_length_rad`,
`path_waypoints`, and `trajectory_points` scoped to the critical `TRANSFER`
motion. `stage_metrics` contains the planning/execution metrics for all FSM
states. MoveIt mock planning-only results have no transfer execution; their
top-level motion metrics are the sum of the four explicitly independent
planning requests. Summaries expose this distinction as
`top_level_motion_metric_scope` and retain the legacy aggregate key names while
also providing `avg_top_level_planning_time_sec` and
`avg_top_level_path_length_rad`.

`max_joint_tracking_error_rad` is `null` in JSON (and NaN only at the ROS float
message boundary) until `/panda_arm_controller/controller_state` supplies real
error samples. In planning-only mode, final XY/Z error and object speed are also
`null`/NaN because no object motion ran. A missing observation is never reported
as zero.

Scene dimensions, safety margins, success thresholds, and backend control
defaults are loaded from the installed `config/scene_defaults.yaml`. The
standalone `run_demo` and `run_benchmark` commands accept `--config` when an
explicit configuration file is needed.

## Dependency-light SVG reports

Generate separate planning-only/full-episode SVG charts directly from recorded
`EpisodeResult` JSONL. The command uses only the Python standard library:

```bash
ros2 run panda_pick_place_planning generate_benchmark_report \
  --input mock=results/mock_100.jsonl \
  --input gazebo=results/gazebo_assisted_100.jsonl \
  --output-dir results/v2_report \
  --check-paired-coverage \
  --title "Panda V2 benchmark"
```

A summary JSON is also accepted, for example
`--input v1=baselines/v0.1-pybullet-baseline/benchmark_summary.json`. Summary
files do not retain task seeds, so paired `(scene, task_seed)` coverage can only
be checked with raw JSONL inputs. Use `--require-paired-coverage` to make unequal
coverage fail instead of only recording it in `report_manifest.json`.

The report contains SVGs for success/planning rates, planning-time
mean/median/P95, path length, placement error, joint tracking error, and failure
counts. Missing values remain `N/A`; planning-only output does not treat
`not_executed` as an execution or task failure. For legacy raw JSONL, placement
error may be calculated from the recorded final and target positions, and its
origin is recorded in the manifest. Existing report files are not overwritten
unless `--force` is passed, protecting frozen baseline artifacts.

## Gazebo assisted batch

The verified wrapper owns exactly one Gazebo/MoveIt/controller stack, waits for
readiness, starts the sequential assisted batch, and shuts the stack down when
the batch exits:

```bash
ros2 launch panda_gz_bringup gazebo_batch.launch.py \
  episodes_per_scene:=5 seed:=42 scenes_csv:=clear,obstacle \
  headless:=true use_rviz:=false \
  output_path:=results/gazebo_quick.jsonl \
  summary_path:=results/gazebo_quick_summary.json
```

Use `episodes_per_scene:=50` and a new output path only after the 5+5 check.
The curated formal raw data, summary, reset validation, SVG report, failure
evidence, environment, and hash manifest are under
`results/v2/gazebo_assisted_final/`.

Execution collision truth is queried from the typed
`/panda_gz/get_execution_collision_state` service. An unavailable query becomes
`collision_state_unavailable`; a valid collision classification is preserved as
the corresponding execution-collision failure. The legacy
`collision_monitor_available` flag remains compatibility metadata and cannot
turn an unavailable query into a collision-free result. Assisted and
physics-grasp results must use separate output paths.
