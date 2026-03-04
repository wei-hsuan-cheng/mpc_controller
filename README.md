# mpc_controller

Minimal ROS 2 package that embeds the [OCS2](https://leggedrobotics.github.io/ocs2/) **mobile manipulator** MPC stack in a [`ros2_control`](https://control.ros.org/humble/index.html) workflow.

The package is intentionally self‑contained (`urdf`/`xacro`, `task` and `rviz` config), but it **depends on the upstream OCS2 ROS 2 repositories** for the solver libraries.

## Prerequisites

Install ROS 2 Humble (or a newer distro with the same APIs).

## Build and Run Demo

```bash
# Clone the related repositories
git clone \
  --recursive https://github.com/wei-hsuan-cheng/ocs2_ros2.git \
  -b humble_stable

git clone \
  https://github.com/wei-hsuan-cheng/mobile_manipulator_mpc.git \
  -b humble_stable

git clone \
  https://github.com/wei-hsuan-cheng/mpc_controller.git \
  -b humble_stable

# rosdep install
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and install
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-up-to mpc_controller \
  --parallel-workers 4 --executor sequential \
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

[`pr2`](./launch/pr2.launch.py)
```bash
# PR2 mobile manipulator
ros2 launch mpc_controller pr2.launch.py \
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
    [5] "5  0.0  1.0  1.0"  ; base + joint
  }
}
```
Note that `modeWeights=3/4/5` are only for floating-base manipulator.

## MPC rollout topic

```bash
# Fixed-base manipulator: joint velocity command and position rollout, e.g., joint_1
/mobile_manipulator_mpc_policy/input_trajectory[0]/value[0]
/mobile_manipulator_mpc_policy/state_trajectory[0]/value[0]

# Floating-base manipulator: joint velocity command and feedback, e.g., joint_1
/mobile_manipulator_mpc_policy/input_trajectory[0]/value[2]
/joint_states/velocity[8]
```

## EE trajectory tracking action examples:

```bash
# Screw move
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_screw_move \
  mpc_cartesian_planner/action/ExecuteScrewMove \
  "{
    duration: 5.0, 
    dt: 0.02, 
    time_scaling: 'min_jerk', 
    screw_uhat: [0.0, 1.0, 0.0], 
    screw_r: [-0.2, 0.0, 0.0], 
    screw_theta: 0.7854, 
    screw_in_tool_frame: true
  }"

# Linear move: [dx, dy, dz, thz, thy, thx]
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_linear_move \
  mpc_cartesian_planner/action/ExecuteLinearMove \
  "{
    duration: 5.0, 
    dt: 0.02, 
    time_scaling: 'min_jerk', 
    linear_move_offset: [0.0, 0.0, 0.1, 0.0, 0.7854, 0.0],
    linear_move_in_tool_frame: true
  }"

# Target pose: [x, y, z, qw, qx, qy, qz]
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_target_pose \
  mpc_cartesian_planner/action/ExecuteTargetPose \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: 'min_jerk',
    target_pose: [0.45, 0.2, 0.7, 0.707, 0.0, 0.0, -0.707]
  }"

# Figure eight
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_figure_eight \
  mpc_cartesian_planner/action/ExecuteFigureEight \
  "{
    duration: 5.0,
    dt: 0.02,
    amplitude: 0.20,
    frequency: 0.05,
    plane_axis: [1.0, 0.0, 1.0]
  }"
```

## Combined EE/base trajectory tracking action examples:

```bash
# 1) Pure EE TT: unchanged
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_target_pose \
  mpc_cartesian_planner/action/ExecuteTargetPose \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: min_jerk,
    target_pose: [0.45, 0.20, 1.0, 0.707, 0.0, 0.0, -0.707]
  }"

# 2) Pure base TT: absolute SE(2) target pose
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_target_pose \
  mpc_cartesian_planner/action/ExecuteCombinedTargetPose \
  "{
    duration: 5.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: false, hold_ee: false, ee_target_pose: [],
    enable_base: true, hold_base: false,
    base_target_pose: [1.0, 0.4, 1.5708]
  }"

# 3) Pure base TT: relative linear move in base frame [dx, dy, dyaw]
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_linear_move \
  mpc_cartesian_planner/action/ExecuteCombinedLinearMove \
  "{
    duration: 4.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: false, hold_ee: false, ee_linear_move_offset: [],
    ee_linear_move_in_tool_frame: true,
    enable_base: true, hold_base: false,
    base_linear_move_offset: [0.5, 0.0, 0.0],
    base_linear_move_in_body_frame: true
  }"

# 4) Pure base screw/arc move
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_screw_move \
  mpc_cartesian_planner/action/ExecuteCombinedScrewMove \
  "{
    duration: 20.0,
    dt: 0.1,
    time_scaling: min_jerk,
    enable_ee: false, hold_ee: false,
    ee_screw_uhat: [0.0, 0.0, 1.0],
    ee_screw_r: [0.0, 0.0, 0.0],
    ee_screw_theta: 0.0,
    ee_screw_in_tool_frame: true,
    enable_base: true, hold_base: false,
    base_screw_r: [0.0, -0.5],
    base_screw_theta: 6.28,
    base_screw_in_body_frame: true
  }"

# 5) Fixed EE in space while base moves
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_linear_move \
  mpc_cartesian_planner/action/ExecuteCombinedLinearMove \
  "{
    duration: 4.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: true, hold_ee: true, ee_linear_move_offset: [],
    ee_linear_move_in_tool_frame: true,
    enable_base: true, hold_base: false,
    base_linear_move_offset: [0.4, 0.0, 0.5236],
    base_linear_move_in_body_frame: true
  }"

# 6) EE + base combined target pose
ros2 action send_goal \
  /mobile_manipulator/trajectory_tracking/execute_combined_target_pose \
  mpc_cartesian_planner/action/ExecuteCombinedTargetPose \
  "{
    duration: 6.0,
    dt: 0.02,
    time_scaling: min_jerk,
    enable_ee: true, hold_ee: false,
    ee_target_pose: [0.55, -0.05, 0.68, 0.707, 0.0, 0.0, -0.707],
    enable_base: true, hold_base: false,
    base_target_pose: [0.8, -0.2, 0.7854]
  }"
```

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)
