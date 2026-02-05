# mpc_controller

Minimal ROS 2 package that embeds the [OCS2](https://leggedrobotics.github.io/ocs2/) **mobile manipulator** MPC stack in a [`ros2_control`](https://control.ros.org/humble/index.html) workflow.

The package is intentionally self‑contained (`urdf`/`xacro`, `task` and `rviz` config), but it **depends on the upstream OCS2 ROS 2 repositories** for the solver libraries.

## Prerequisites

1. Install ROS 2 Humble (or a newer distro with the same APIs).
2. Clone and build [`ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2) up to `ocs2_mobile_manipulator_ros` `pkg`:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install \
      --packages-up-to ocs2_mobile_manipulator_ros \
      --parallel-workers 2 --executor sequential \
      && . install/setup.bash
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

[`ur5`](./launch/ur5.launch.py)
```bash
# Fixed-base UR5 (no mobile base)
ros2 launch mpc_controller ur5.launch.py \
  solver:=ddp \
  commandType:=marker \
  use_fake_hardware:=true
# sovler:=ddp, sqp
# commandType:=marker, twist, trajectory
```

[`ridgeback_ur5`](./launch/ridgeback_ur5.launch.py)
```bash
# Ridgeback + UR5 mobile manipulator
ros2 launch mpc_controller ridgeback_ur5.launch.py \
  solver:=ddp \
  commandType:=marker \
  use_fake_hardware:=true
# sovler:=ddp, sqp
# commandType:=marker, twist, trajectory
```

If you built [`ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2) in another workspace, source it **before** running the commands above (so their messages and plugins are discoverable).

## Folder layout

Key parameters are specified in [`task.info`](./config/ridgeback_ur5/task.info) and [`ros2_controllers.yaml`](./config/ridgeback_ur5/ros2_controllers.yaml).

```bash

auto_generated/   -> pre-built OCS2 libraries (CppAD) for robot
config/           -> ros2_control YAML + task files
description/      -> xacro/urdf assets (ros2_control settings included)
include/          -> Header files
rviz/             -> RViz configuration copied from OCS2 examples
src/
  command/        -> Support different reference command (marker/twist/trajecotry)
  control/        -> ros2_control controllers (OCS2 bridge)
  mpc/            -> OCS2 MPC node (DDP or SQP solver supported)
  sim/            -> Simple planar base fake odom simulation
  visualization/  -> Marker, trajectory, and pose visualizations in RViz2
```

## Robot model meta-information (fixed vs. floating base)

The robot kinematic model type is configured in the [`task.info`](./config/ridgeback_ur5/task.info) via model_information.
This lets you switch between a fixed-base arm and different floating/mobile base variants by changing a single block:

```bash
; robot model meta-information
model_information {
  manipulatorModelType     1      // 0: Default-arm (fixed base)
                                  // 1: Wheel-based mobile manipulator
                                  // 2: Floating-arm manipulator
                                  // 3: Fully actuated floating-arm manipulator

  ; motion joints in the URDF to consider fixed (optional)
  removeJoints {
    // [0] "ur_arm_shoulder_lift_joint"
  }

  ; base frame of the robot (from URDF)
  baseFrame                       "base_link"

  ; end-effector frame of the robot (from URDF)
  eeFrame                         "ur_arm_tool0" // e.g., ur_arm_tool0, ur_arm_flange
}
```

Model type selection:
- `manipulatorModelType=0` for a standard fixed-base arm. 
- `manipulatorModelType=1/2/3` when you want the base to be mobile/floating (wheel-based or free-floating), while keeping the same arm/EE frames from your URDF.

## Mode schedule
Start MPC controller and switch between joint, base, EE, tracking, or custom blending modes by topic.
```bash
# First switch to mode 1
ros2 topic pub -1 /mobile_manipulator_mode_schedule ocs2_msgs/msg/ModeSchedule "{event_times: [], mode_sequence: [1]}"
# Then start MPC controller
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
   "{activate_controllers: ["mpc_controller"]}"
# Echo current mode
ros2 topic echo /mobile_manipulator_mode_schedule
```

Details of the mode schedule:
```bash
referenceBlending
{
  normalize   true
  eps         1e-9

  default
  {
    alphaEe     0.0
    alphaBase   0.0
    alphaJoint  1.0
  }

  ; Each entry defines weights for a specific mode number.
  ; Two supported syntaxes:
  ;   [0] "mode alphaEe alphaBase alphaJoint"
  ;   [0] { mode 0 alphaEe 0 alphaBase 1 alphaJoint 0 }
  modeWeights
  {
    [0] "0  0.0  0.0  1.0"  ; joint-only
    [1] "1  1.0  0.0  0.0"  ; EE-only
    [2] "2  1.0  0.0  1.0"  ; joint + EE
    [3] "3  0.0  1.0  0.0"  ; base-only
    [4] "4  1.0  1.0  0.0"  ; base + EE
  }
}
```
Note that `modeWeights=3/4` are only for floating-base manipulator.

## MPC rollout topic

```bash
# Fixed-base manipulator: joint velocity command and position rollout, e.g., joint_1
/mobile_manipulator_mpc_policy/input_trajectory[0]/value[0]
/mobile_manipulator_mpc_policy/state_trajectory[0]/value[0]

# Floating-base manipulator: joint velocity command and feedback, e.g., joint_1
/mobile_manipulator_mpc_policy/input_trajectory[0]/value[2]
/joint_states/velocity[8]
```

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)