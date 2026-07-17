# panda_gz_bringup

ROS 2 Humble + modern Gazebo Fortress bringup for the Panda pick-place project.
It deliberately uses `ros_gz`, `gz_ros2_control/GazeboSimSystem`, and Fortress'
`ignition-gazebo-*` system ABI. It does not use Gazebo Classic.

## Verification boundary

This package is currently **static-tested but not runtime-verified**. Its asset,
launch-contract, reset, readiness, grasp, and collision-classifier suite is
**44/44**. Those tests are not Gazebo runtime execution evidence. The current
machine is missing `ignition-msgs8`, `ros_gz`, and
`gz_ros2_control`, so this package was not part of the final four-package build.
No Gazebo controller/runtime reset, assisted 50+50 batch, physics episode, or
video has been verified. A launch file or static test is not execution evidence.

The shared description has one authoritative frame chain:
`world -> panda_link0 -> panda_hand -> panda_grasptarget`. Pose goals use the
collision-free `panda_grasptarget`; attached objects use the physical
`panda_hand` link.

```bash
ros2 launch panda_gz_bringup panda_gz.launch.py \
  scene:=obstacle grasp_mode:=assisted headless:=false
```

The complete MoveIt + Gazebo stack has one entry point:

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

Launch arguments are `scene:=clear|obstacle`,
`grasp_mode:=assisted|physics`, `headless:=true|false`,
`use_sim_time:=true|false`, and `auto_reset:=true|false`.

Management interfaces:

- `/panda_gz/reset_episode` (`panda_pick_place_interfaces/srv/ResetEpisode`),
  carrying episode ID, task seed, scene, grasp mode, object position and target
- `/panda_gz/attach_object` (`std_srvs/srv/Trigger`)
- `/panda_gz/detach_object` (`std_srvs/srv/Trigger`)
- `/panda_gz/get_physical_grasp_state`
  (`panda_pick_place_interfaces/srv/GetPhysicalGraspState`)
- `/panda_gz/get_execution_collision_state`
  (`panda_pick_place_interfaces/srv/GetExecutionCollisionState`)
- `/panda_gz/reset_status` and `/panda_gz/grasp_status`
  (`diagnostic_msgs/msg/DiagnosticArray`, including stable numeric status codes)

Assisted attach is rejected unless the hand-to-cube offset, hand orientation,
distance, and gripper clearance all pass. The default pre-attach opening is
60 mm (configurable, with an 8 mm total tolerance) around the 50 mm cube
because Gazebo's DetachableJoint cannot reattach models while they are already
in contact. `physics` mode never sends an attach command; finger collision and
friction alone determine the grasp.

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
V1 home and opens `panda_hand_controller`, verifies arm/hand position and zero
velocity tolerances, reapplies task poses while paused, and resets both monitor
states. Humble's stock bridge does not convert `SetEntityPose`; the package's
small C++ `entity_pose_bridge` maps the standard ROS service to Fortress's
native Transport service. No joint state is mutated behind `ros2_control`.

The Gazebo collision table is the V1-sized 1.5 x 1.0 x 0.05 m slab with its top
at z=0; table legs are visual-only. Assisted and physical results must be
reported separately. This package contains measurement interfaces, not claimed
physical-grasp success rates.
