#include "mpc_controller/control/OCS2Controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <rclcpp/logging.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace mpc_controller
{

using ocs2::vector_t;
using ocs2::SystemObservation;
using ocs2::TargetTrajectories;
using ocs2::mobile_manipulator::MobileManipulatorInterface;

controller_interface::CallbackReturn OCS2Controller::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  // Control mode: true -> send JointJog to MoveIt Servo; false -> write velocities directly to hardware.
  if (node->has_parameter("use_moveit_servo")) {
    use_moveit_servo_ = node->get_parameter("use_moveit_servo").as_bool();
  } else {
    use_moveit_servo_ = node->declare_parameter<bool>("use_moveit_servo", false);
  }

  // Read parameters that are provided via ros2_control YAML / launch arguments
  task_file_ = node->get_parameter("taskFile").as_string();
  lib_folder_ = node->get_parameter("libFolder").as_string();
  urdf_file_ = node->get_parameter("urdfFile").as_string();
  future_time_offset_ = node->get_parameter("futureTimeOffset").as_double();
  command_smoothing_alpha_ = node->get_parameter("commandSmoothingAlpha").as_double();
  global_frame_ = node->get_parameter("globalFrame").as_string();

  arm_joint_names_ = node->get_parameter("arm_joints").as_string_array();

  // Optional topics, fall back to compiled-in defaults if not set
  if (node->has_parameter("base_cmd_topic")) {
    base_cmd_topic_ = node->get_parameter("base_cmd_topic").as_string();
  }
  if (node->has_parameter("odom_topic")) {
    odom_topic_ = node->get_parameter("odom_topic").as_string();
  }
  if (node->has_parameter("joint_jog_topic")) {
    joint_jog_topic_ = node->get_parameter("joint_jog_topic").as_string();
  }

  command_smoothing_alpha_ = std::clamp(command_smoothing_alpha_, 0.0, 1.0);

  // Log configuration for easier debugging.
  std::string arm_joints_str;
  for (size_t i = 0; i < arm_joint_names_.size(); ++i) {
    arm_joints_str += arm_joint_names_[i];
    if (i + 1 < arm_joint_names_.size()) {
      arm_joints_str += ", ";
    }
  }

  RCLCPP_INFO(
    node->get_logger(),
    "[OCS2Controller] configuration:\n"
    "  taskFile: %s\n"
    "  libFolder: %s\n"
    "  urdfFile: %s\n"
    "  futureTimeOffset: %.6f\n"
    "  commandSmoothingAlpha (clamped): %.6f\n"
    "  globalFrame: %s\n"
    "  use_moveit_servo: %s\n"
    "  arm_joints: [%s]\n"
    "  base_cmd_topic: %s\n"
    "  odom_topic: %s\n"
    "  joint_jog_topic: %s",
    task_file_.c_str(),
    lib_folder_.c_str(),
    urdf_file_.c_str(),
    future_time_offset_,
    command_smoothing_alpha_,
    global_frame_.c_str(),
    use_moveit_servo_ ? "true" : "false",
    arm_joints_str.c_str(),
    base_cmd_topic_.c_str(),
    odom_topic_.c_str(),
    joint_jog_topic_.c_str());

  if (task_file_.empty() || lib_folder_.empty() || urdf_file_.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[OCS2Controller] parameters 'taskFile', 'libFolder' or 'urdfFile' are empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    interface_ = std::make_unique<MobileManipulatorInterface>(
      task_file_, lib_folder_, urdf_file_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Visualization helper
  try {
    visualization_node_ = std::make_shared<rclcpp::Node>(
      node->get_name() + std::string("_visualizer"), node->get_namespace());
    visualization_ = std::make_unique<MobileManipulatorVisualization>(
      visualization_node_, *interface_, task_file_, urdf_file_, global_frame_);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "[OCS2Controller]: failed to init visualization: %s", e.what());
    visualization_.reset();
    visualization_node_.reset();
  }

  try {
    mrt_ = std::make_unique<ocs2::MRT_ROS_Interface>("mobile_manipulator");
    mrt_->initRollout(&interface_->getRollout());

    auto options = rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
    const auto bridge_name = node->get_name() + std::string("_mrt_bridge");
    mrt_node_ = std::make_shared<rclcpp::Node>(bridge_name, node->get_namespace(), options);
    mrt_->launchNodes(mrt_node_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize MRT interface: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto & info = interface_->getManipulatorModelInfo();
  state_dim_ = info.stateDim;
  input_dim_ = info.inputDim;

  if (arm_joint_names_.size() != static_cast<size_t>(info.armDim)) {
    RCLCPP_WARN(
      node->get_logger(),
      "Configured %zu arm joints but model expects %zu. Using provided list.",
      arm_joint_names_.size(), static_cast<size_t>(info.armDim));
  }

  // Init observation / command buffers
  initial_observation_.state = vector_t::Zero(state_dim_);
  initial_observation_.input = vector_t::Zero(input_dim_);
  initial_observation_.time = 0.0;
  initial_observation_.mode = 0;
  initial_target_ = TargetTrajectories();
  last_command_ = vector_t::Zero(input_dim_);

  // Base command and odom interfaces
  base_cmd_pub_ =
    node->create_publisher<geometry_msgs::msg::Twist>(base_cmd_topic_, rclcpp::SystemDefaultsQoS());

  joint_jog_pub_ =
    node->create_publisher<control_msgs::msg::JointJog>(joint_jog_topic_, rclcpp::SystemDefaultsQoS());

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      base_pose_from_odom_[0] = msg->pose.pose.position.x;
      base_pose_from_odom_[1] = msg->pose.pose.position.y;
      const auto & q = msg->pose.pose.orientation;
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      base_pose_from_odom_[2] = std::atan2(siny_cosp, cosy_cosp);
    });

  mpc_reset_done_ = false;
  handles_initialized_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!initializeHandles()) {
    RCLCPP_ERROR(get_node()->get_logger(), "[OCS2Controller] failed to initialize state handles.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto now = get_node()->now();
  initial_observation_ = buildObservation(now);
  initial_target_ = computeInitialTarget(initial_observation_.state, initial_observation_.time);

  if (mrt_) {
    mrt_->reset();
    resetMpc();
  }

  handles_initialized_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  handles_initialized_ = false;
  joint_position_states_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OCS2Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Only claim hardware position command interfaces when we are in direct
  // hardware-control mode. When using MoveIt Servo (use_moveit_servo_ == true),
  // the ServoController owns these interfaces and we publish JointJog commands
  // instead, so we must not claim them here to avoid resource conflicts.
  if (!use_moveit_servo_) {
    for (const auto & joint : arm_joint_names_) {
      config.names.push_back(joint + std::string("/") + hardware_interface::HW_IF_POSITION);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration OCS2Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : arm_joint_names_) {
    config.names.push_back(joint + std::string("/") + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

void OCS2Controller::applyCommandUsingNextState(const vector_t& command, const vector_t& x_next)
{
  if (!base_cmd_pub_) {
    return;
  }

  const size_t arm_dim = arm_joint_names_.size();

  // --- Base twist command (cmd_vel) ---
  if (static_cast<size_t>(command.size()) >= 2) {
    geometry_msgs::msg::Twist base_cmd;
    base_cmd.linear.x  = command(0);
    base_cmd.linear.y  = 0.0;
    base_cmd.linear.z  = 0.0;
    base_cmd.angular.x = 0.0;
    base_cmd.angular.y = 0.0;
    base_cmd.angular.z = command(1);
    base_cmd_pub_->publish(base_cmd);
  }

  // --- Arm position commands (hardware supports position interface @ 250Hz) ---
  if (joint_position_commands_.size() < arm_dim || arm_dim == 0) {
    return;
  }

  // Assume wheel-based state layout: [x, y, yaw, q0, q1, ...]
  const Eigen::Index arm_state_offset = 3;

  for (size_t idx = 0; idx < arm_dim; ++idx) {
    auto* cmd = joint_position_commands_[idx];
    if (!cmd) {
      continue;
    }

    const Eigen::Index si = arm_state_offset + static_cast<Eigen::Index>(idx);
    if (si >= x_next.size()) {
      continue;
    }

    // Set next-step desired joint position from rollout result
    cmd->set_value(x_next(si));
  }
}

controller_interface::return_type OCS2Controller::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!handles_initialized_ || !mrt_) {
    return controller_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  // 1) Build observation (current measured state)
  auto observation = buildObservation(time);

  // 2) Feed observation to MRT and update policy (non-blocking / best effort)
  mrt_->setCurrentObservation(observation);
  mrt_->spinMRT();
  mrt_->updatePolicy();

  // Prepare buffers
  vector_t x_next(state_dim_);
  vector_t u_end(input_dim_);
  size_t mode_end = 0;

  if (mrt_->initialPolicyReceived()) {
    // 3) Rollout ONE controller period with the same integrator as ocs2 dummy mrt sim (e.g., ODE45)
    //    This makes base+arm propagation consistent (instead of ZOH + external fake odom).
    mrt_->rolloutPolicy(
      observation.time + future_time_offset_,   // compensate measurement/actuation delay if needed
      observation.state,
      dt,
      x_next,
      u_end,
      mode_end
    );

    // Optional: command smoothing
    // WARNING: smoothing u_end AFTER rollout makes (u_end, x_next) inconsistent.
    // Suggestion: temporarily disable smoothing to validate stability, or implement smoothing INSIDE policy (more work).
    if (command_smoothing_alpha_ < 1.0) {
      // If you still want smoothing, at least apply it only to the base cmd to avoid x_next mismatch on arm.
      vector_t u_smoothed = u_end;
      u_smoothed(0) = command_smoothing_alpha_ * u_end(0) + (1.0 - command_smoothing_alpha_) * last_command_(0);
      u_smoothed(1) = command_smoothing_alpha_ * u_end(1) + (1.0 - command_smoothing_alpha_) * last_command_(1);
      // Keep arm part unchanged to stay consistent with x_next.
      for (Eigen::Index i = 2; i < u_end.size(); ++i) {
        u_smoothed(i) = u_end(i);
      }
      u_end = u_smoothed;
    }

    // 4) Apply base cmd (twist) AND arm pos command using x_next
    applyCommandUsingNextState(u_end, x_next);

    last_command_ = u_end;

    // Visualization (same as before)
    if (visualization_) {
      visualization_->update(observation.state, mrt_->getPolicy(), mrt_->getCommand());
    }

  } else {
    // No policy yet: send zeros
    vector_t zero = vector_t::Zero(input_dim_);
    // Build a "do nothing" next state from current measured state
    x_next = observation.state;
    applyCommandUsingNextState(zero, x_next);
    last_command_ = zero;
  }

  return controller_interface::return_type::OK;
}

void OCS2Controller::resetMpc()
{
  if (mpc_reset_done_ || !mrt_) {
    return;
  }

  try {
    const auto now = get_node()->now();
    initial_observation_ = buildObservation(now);
    initial_target_ =
      computeInitialTarget(initial_observation_.state, initial_observation_.time);
    mrt_->resetMpcNode(initial_target_);
    mpc_reset_done_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "[OCS2Controller] requested MPC reset.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "[OCS2Controller] Failed to reset MPC node: %s", e.what());
  }
}

SystemObservation OCS2Controller::buildObservation(const rclcpp::Time & time) const
{
  SystemObservation obs;
  obs.time = time.seconds();
  obs.state = vector_t::Zero(state_dim_);
  obs.input = last_command_;
  obs.mode = 0;

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (state_dim_ >= 3) {
      obs.state(0) = base_pose_from_odom_[0];
      obs.state(1) = base_pose_from_odom_[1];
      obs.state(2) = base_pose_from_odom_[2];
    }
  }

  for (size_t idx = 0; idx < arm_joint_names_.size(); ++idx) {
    const Eigen::Index state_index = static_cast<Eigen::Index>(3 + idx);
    if (state_index < obs.state.size() && idx < joint_position_states_.size()) {
      obs.state(state_index) = joint_position_states_[idx]->get_value();
    }
  }

  return obs;
}

void OCS2Controller::applyCommand(const vector_t & command, double dt)
{
  if (!base_cmd_pub_) {
    return;
  }

  if (static_cast<size_t>(command.size()) < 2 + arm_joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "[OCS2Controller] command size %ld, expected at least %zu. Skipping.",
      command.size(), 2 + arm_joint_names_.size());
    return;
  }

  auto node = get_node();

  // Base twist (cmd_vel)
  geometry_msgs::msg::Twist base_cmd;
  base_cmd.linear.x = command(0);
  base_cmd.linear.y = 0.0;
  base_cmd.linear.z = 0.0;
  base_cmd.angular.x = 0.0;
  base_cmd.angular.y = 0.0;
  base_cmd.angular.z = command(1);
  base_cmd_pub_->publish(base_cmd);

  const size_t arm_dim = arm_joint_names_.size();

  if (use_moveit_servo_) {
    // Arm joint velocities via JointJog for MoveIt Servo
    if (!joint_jog_pub_) {
      return;
    }

    control_msgs::msg::JointJog jog;
    jog.header.stamp = node->get_clock()->now();
    jog.header.frame_id = "";

    jog.joint_names.reserve(arm_dim);
    jog.velocities.reserve(arm_dim);

    for (size_t idx = 0; idx < arm_dim; ++idx) {
      jog.joint_names.push_back(arm_joint_names_[idx]);
      jog.velocities.push_back(command(2 + idx));
    }
    jog.duration = 0.0;

    joint_jog_pub_->publish(jog);
  } else {
    // Directly write integrated positions to hardware position command interfaces.
    // The OCS2 command vector encodes joint velocities; here we integrate them with
    // the controller update period dt to obtain position commands.
    if (joint_position_commands_.size() < arm_dim || joint_position_states_.size() < arm_dim) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "[OCS2Controller] missing position command handles for %zu joints.", arm_dim);
      return;
    }

    for (size_t idx = 0; idx < arm_dim; ++idx) {
      auto * state = joint_position_states_[idx];
      auto * cmd = joint_position_commands_[idx];
      if (!state || !cmd) {
        continue;
      }
      const double q = state->get_value();
      const double dq = command(2 + idx);  // rad/s from OCS2
      const double q_next = q + dq * dt;
      cmd->set_value(q_next);
    }
  }
}

TargetTrajectories OCS2Controller::computeInitialTarget(
  const vector_t & state, double time) const
{
  vector_t init_target;
  const auto & pin_interface = interface_->getPinocchioInterface();
  const auto & model = pin_interface.getModel();
  auto data = pin_interface.getData();

  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);

  const auto & info = interface_->getManipulatorModelInfo();
  if (interface_->dual_arm_) {
    init_target.resize(14);
    const auto left_id = model.getFrameId(info.eeFrame);
    const auto & left = data.oMf[left_id];
    Eigen::Quaterniond left_q(left.rotation());
    init_target.segment<3>(0) = left.translation();
    init_target.segment<4>(3) = left_q.coeffs();

    const auto right_id = model.getFrameId(info.eeFrame1);
    const auto & right = data.oMf[right_id];
    Eigen::Quaterniond right_q(right.rotation());
    init_target.segment<3>(7) = right.translation();
    init_target.segment<4>(10) = right_q.coeffs();
  } else {
    init_target.resize(7);
    const auto ee_id = model.getFrameId(info.eeFrame);
    const auto & ee = data.oMf[ee_id];
    Eigen::Quaterniond quat(ee.rotation());
    init_target.head<3>() = ee.translation();
    init_target.tail<4>() = quat.coeffs();
  }

  const vector_t zero_input =
    vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
  return TargetTrajectories({time}, {init_target}, {zero_input});
}

bool OCS2Controller::initializeHandles()
{
  auto matches_name = [](const std::string & resource_name, const std::string & expected_joint) {
    if (resource_name == expected_joint) {
      return true;
    }
    const auto slash_idx = resource_name.find('/');
    if (slash_idx == std::string::npos) {
      return false;
    }
    return resource_name.substr(0, slash_idx) == expected_joint;
  };

  auto find_state =
    [this, &matches_name](
    const std::string & name,
    const std::string & interface) -> hardware_interface::LoanedStateInterface * {
      for (auto & state : state_interfaces_) {
        if (matches_name(state.get_name(), name) && state.get_interface_name() == interface) {
          return &state;
        }
      }
      return nullptr;
    };

  auto find_command =
    [this, &matches_name](
    const std::string & name,
    const std::string & interface) -> hardware_interface::LoanedCommandInterface * {
      for (auto & cmd : command_interfaces_) {
        if (matches_name(cmd.get_name(), name) && cmd.get_interface_name() == interface) {
          return &cmd;
        }
      }
      return nullptr;
    };

  joint_position_states_.clear();
  joint_position_commands_.clear();

  bool ok = true;

  for (const auto & joint : arm_joint_names_) {
    auto * pos = find_state(joint, hardware_interface::HW_IF_POSITION);
    if (!pos) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "[OCS2Controller] missing position state for joint '%s'.",
        joint.c_str());
      for (auto & state : state_interfaces_) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "  state %s/%s", state.get_name().c_str(),
          state.get_interface_name().c_str());
      }
      ok = false;
      continue;
    }
    joint_position_states_.push_back(pos);

    if (!use_moveit_servo_) {
      auto * pos_cmd = find_command(joint, hardware_interface::HW_IF_POSITION);
      if (!pos_cmd) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "[OCS2Controller] missing position command for joint '%s'.",
          joint.c_str());
        ok = false;
      } else {
        joint_position_commands_.push_back(pos_cmd);
      }
    }
  }

  return ok;
}

}  // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(
  mpc_controller::OCS2Controller,
  controller_interface::ControllerInterface)
