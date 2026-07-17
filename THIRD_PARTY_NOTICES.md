# Third-party notices

## Panda kinematic description and meshes

`packages/panda_moveit_config/config/panda_description.urdf.xacro` keeps the
kinematic structure, joint limits, and mesh paths from
`moveit_resources_panda_description` 2.0.7. The meshes remain an external ROS
package dependency and are not redistributed here.

- Upstream: <https://github.com/moveit/moveit_resources>
- ROS package version: 2.0.7
- License: BSD-3-Clause
- Source URDF SHA-256:
  `cd4d2b212152560863fbc0b343f0011d6acd925b0343eb6f1b6fa9ae2666e2ff`

## Panda identified inertia parameters

The mass, center-of-mass, and inertia values in that Xacro are transcribed
from the Franka Robotics `franka_ros` repository:

- Upstream: <https://github.com/frankarobotics/franka_ros>
- Tag: `0.10.2`
- Commit: `c11f000c2737acc22ed8dfeff40555b2644a4294`
- Source file: `franka_description/robots/common/inertial.yaml`
- License: Apache-2.0

The upstream file explicitly says these are not official Panda inertial
properties. They are identification results reported in *Identification of
the Franka Robotics Panda Robot With Retrieval of Feasible Parameters Using
Penalty-Based Optimization* by Gaz, Cognetti, Oliva, Robuffo Giordano, and De
Luca. This project uses them only to make the Gazebo assisted-grasp controller
model dynamically well formed. `physics_model_validated` remains `false`, and
the values are not evidence that contact-only physical grasping is realistic.
