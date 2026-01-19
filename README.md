# mpc_controller

Minimal ROS 2 control package that embeds the [OCS2](https://leggedrobotics.github.io/ocs2/) mobile manipulator MPC stack in a `ros2_control` workflow.

The package is intentionally self‑contained (`urdf`/`xacro`, `task` and `rviz` config), but it **depends on the upstream OCS2 ROS 2 repositories** for the solver libraries and marker/MPC nodes.

## Prerequisites

1. Install ROS 2 Humble (or a newer distro with the same APIs).
2. Clone and build [`ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2) up to `ocs2_mobile_manipulator_ros` `pkg`:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install --packages-up-to ocs2_mobile_manipulator_ros
    ```
    - Make sure the [`ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2) workspace is sourced before building this package.

## Build and Run Demo

```bash
# Clone this repo
git clone https://github.com/wei-hsuan-cheng/mpc_controller.git

# rosdep install
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and install
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select mpc_controller \
  --parallel-workers 2 --executor sequential \
  && . install/setup.bash
```

```bash
# E.g., Ridgeback + UR5 mobile manipulator
ros2 launch mpc_controller ridgeback_ur5.launch.py \
  solver:=ddp \
  commandType:=marker \
  use_fake_hardware:=true

# sovler:=ddp, sqp
# commandType:=marker, twist, trajectory
# use_fake_hardware:=true, false
```

Key arguments:
- `taskFile`, `urdfFile`, `libFolder`: OCS2 inputs (used by both MPC node and controller).
- `baseCmdTopic`, `odomTopic`: topics used by the controller for base command / odometry.


If you built [`ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2) in another workspace, source it **before** running the commands above (so their messages and plugins are discoverable).

Useful arguments:

| Argument            | Default  | Description                                                    |
|---------------------|----------|----------------------------------------------------------------|
| `rviz`              | `true`   | Enable/disable RViz + interactive marker.                      |
| `taskFile`          | local    | Path to the OCS2 `task.info` file.                             |
| `libFolder`         | local    | Folder containing the auto-generated OCS2 libs.                |
| `urdfFile`          | local    | URDF used by visualization nodes.                              |
| `commandType`       | `marker` | Target interface to start: `marker`, `twist`, or `trajectory`. |
| `markerPublishRate` | `100.0`  | Interactive marker publish rate (Hz, `commandType=marker`).    |

## Folder layout

```bash
auto_generated/   -> pre-built OCS2 libraries (CppAD) for robot
config/           -> ros2_control YAML + task files
description/      -> xacro/urdf assets (ros2_control settings included)
include/          -> Header files
rviz/             -> RViz configuration copied from OCS2 examples
src/
  control/        -> ros2_control controllers (OCS2 bridge)
  visualization/  -> Marker, trajectory, and pose visualizations in RViz2
```

## MPC rollout topic

```bash
# joint velocity command and feedback, e.g., joint_1
/mobile_manipulator_mpc_policy/input_trajectory[0]/value[2]
/joint_states/velocity[8]
```

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)