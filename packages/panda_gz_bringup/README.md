# panda_gz_bringup

ROS 2 Humble + modern Gazebo Fortress bringup for the Panda pick-place project.
It deliberately uses `ros_gz`, `gz_ros2_control/GazeboSimSystem`, and Fortress'
`ignition-gazebo-*` system ABI. It does not use Gazebo Classic.

## Verification boundary

The Gazebo assisted runtime is verified with the complete MoveIt/controller
stack. The fixed seed-42 regression contains 50 clear and 50 obstacle episodes;
both groups completed 50/50 tasks. The same stack also completed the readiness
checks and deterministic reset validation. Curated raw data and summaries are
documented in `../../docs/experiments.md`.

This evidence is limited to guarded assisted grasp. The physics monitor and
interfaces exist, but `physics_model_validated=false` deliberately rejects a
physics-grasp launch until mass/inertia, contact geometry, friction, and runtime
behavior receive a separate audit.

The shared description has one authoritative frame chain:
`world -> panda_link0 -> panda_hand -> panda_grasptarget`. Pose goals use the
collision-free `panda_grasptarget`; attached objects use the physical
`panda_hand` link.

```bash
ros2 launch panda_gz_bringup panda_gz.launch.py \
  scene:=obstacle grasp_mode:=assisted headless:=false
```

底层组合入口为 `panda_moveit_gz.launch.py`；面向演示的唯一推荐入口是：

```bash
ros2 launch panda_gz_bringup panda_visual_pick_place.launch.py \
  scene:=clear seed:=42 object_variant:=rect_prism \
  pose_source:=rgbd perception_profile:=nominal \
  headless:=false use_rviz:=true
```

只启动底层 MoveIt + Gazebo 栈、暂不启动任务时使用：

```bash
ros2 launch panda_gz_bringup panda_moveit_gz.launch.py \
  scene:=clear seed:=42 grasp_mode:=assisted \
  headless:=false use_rviz:=true start_application:=false
```

That wrapper reuses `panda_gz.launch.py`, then adds exactly one `move_group`,
one `panda_moveit_bridge`, optional RViz, and optional evaluator / manipulation /
task-generator nodes. The Gazebo child launch remains the sole owner of the
state publisher and Gazebo-created controller manager. With
`start_application:=true`, `stack_ready_gate` waits for `/clock`, joint state,
cube odometry, all three active controllers, reset/scene/collision services,
and motion/arm/gripper actions before starting the shared FSM.

The recommended wrapper exposes `scene`, `seed`, `object_variant`, `pose_source`,
`perception_profile`, `headless`, and `use_rviz`. Lower-level bringup also
accepts `grasp_mode:=assisted|physics`, `use_sim_time`, and `auto_reset`; these
are intentionally fixed by the recommended visual workflow.

Management interfaces:

- `/panda_gz/reset_episode` (`panda_pick_place_interfaces/srv/ResetEpisode`),
  carrying episode ID, task seed, scene, grasp mode, object variant/yaw,
  object position and target
- `/panda_gz/attach_object` (`std_srvs/srv/Trigger`)
- `/panda_gz/detach_object` (`std_srvs/srv/Trigger`)
- `/panda_gz/get_physical_grasp_state`
  (`panda_pick_place_interfaces/srv/GetPhysicalGraspState`)
- `/panda_gz/get_execution_collision_state`
  (`panda_pick_place_interfaces/srv/GetExecutionCollisionState`)
- `/panda_gz/reset_status` and `/panda_gz/grasp_status`
  (`diagnostic_msgs/msg/DiagnosticArray`, including stable numeric status codes)

Assisted attach is rejected unless the hand-to-object offset, relative
orientation, distance, and gripper clearance all pass. The default pre-attach
command is 38 mm per finger, i.e. 76 mm total opening; the orientation gate
accepts the gripper's pi-symmetric relative pose for the 60x40x50 mm rectangular
workpiece. Gazebo's DetachableJoint cannot reattach models while they are
already in contact. `physics` mode never sends an attach command; finger
collision and friction alone determine the grasp.

The physics monitor requires sustained left and right finger contacts, records
the first `panda_grasptarget -> cube` relative pose, and reports translation
drift, rotation drift, slip, release, object speed, and release stable time. It
does not infer a successful grasp merely from a closed gripper command.

The physics observation and latch logic is implemented, but physics is
deliberately fail-closed. The shared Panda description does not yet provide an
audited mass/inertia model suitable for credible dynamics. The default
`physics_model_validated:=false` rejects a physics launch and the typed monitor
returns `physics_model_unvalidated`; it must not be overridden until the model
and runtime behavior are independently validated.

## Execution collision evidence

The shared Gazebo Xacro adds 100 Hz contact sensors to `panda_link0` through
`panda_link7`, `panda_hand`, `panda_leftfinger`, and `panda_rightfinger`.
`panda_link8` and `panda_grasptarget` have no collision geometry. All robot
sensors publish scoped collision pairs on Gazebo Transport topic
`/panda/robot_contacts`, bridged to ROS `/panda_gz/robot_contacts`. The cube's
SDF sensor publishes `/cube/contacts`, bridged to
`/panda_gz/cube_contacts`.

The pure contact classifier applies these rules:

- SRDF-disabled adjacent / rigid robot pairs are allowed only as robot–robot
  pairs; the same links colliding with the environment are still checked.
- `panda_hand` or either finger touching the cube is an allowed grasp contact.
- `panda_link0` touching the tabletop is an allowed base support contact; every
  other robot–table contact and every robot–obstacle contact is invalid.
- Cube–table contact is allowed while resting or released. While carried it is
  allowed only within the configured source pickup or target placement
  tolerances; contact elsewhere is a carried-object collision.
- Cube–obstacle / ground, non-ACM self-collision, arm–cube, and a monitored
  entity touching an unknown model are invalid and latched per episode.

The typed query returns `available=false` until robot contacts, cube contacts,
and cube pose have all arrived. This is intentionally different from a manual
"collision monitor available" flag.

The reset manager performs a model-only Gazebo reset (so `/clock` does not jump
backwards), detaches the cube, reapplies task poses, sends the Panda arm to the
configured home and opens `panda_hand_controller`, verifies arm/hand position and zero
velocity tolerances, reapplies task poses while paused, and resets both monitor
states. Humble's stock bridge does not convert `SetEntityPose`; the package's
small C++ `entity_pose_bridge` maps the standard ROS service to Fortress's
native Transport service. No joint state is mutated behind `ros2_control`.

With a matching `rect_prism` stack running, the deterministic ten-reset audit is:

```bash
ros2 run panda_gz_bringup validate_episode_reset \
  --repetitions 10 --scene clear --seed 42 --object-variant rect_prism \
  --output results/runs/gazebo_rgbd_grasp_pose/reset_validation.json
```

The Gazebo collision table matches the PyBullet 1.5 x 1.0 x 0.05 m slab with its top
at z=0; table legs are visual-only. Assisted and physical results must be
reported separately. This package contains measurement interfaces, not claimed
physical-grasp success rates.
