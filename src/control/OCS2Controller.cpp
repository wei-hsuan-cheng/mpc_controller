#include "mpc_controller/control/OCS2Controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

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

  // ----- existing params -----
  if (node->has_parameter("use_moveit_servo")) {
    use_moveit_servo_ = node->get_parameter("use_moveit_servo").as_bool();
  } else {
    use_moveit_servo_ = node->declare_parameter<bool>("use_moveit_servo", false);
  }

  task_file_ = node->get_parameter("taskFile").as_string();
  lib_folder_ = node->get_parameter("libFolder").as_string();
  urdf_file_ = node->get_parameter("urdfFile").as_string();
  future_time_offset_ = node->get_parameter("futureTimeOffset").as_double();
  command_smoothing_alpha_ = node->get_parameter("commandSmoothingAlpha").as_double();
  global_frame_ = node->get_parameter("globalFrame").as_string();
  arm_joint_names_ = node->get_parameter("arm_joints").as_string_array();

  if (node->has_parameter("base_cmd_topic")) base_cmd_topic_ = node->get_parameter("base_cmd_topic").as_string();
  if (node->has_parameter("odom_topic")) odom_topic_ = node->get_parameter("odom_topic").as_string();
  if (node->has_parameter("joint_jog_topic")) joint_jog_topic_ = node->get_parameter("joint_jog_topic").as_string();

  command_smoothing_alpha_ = std::clamp(command_smoothing_alpha_, 0.0, 1.0);

  // ----- NEW: dummy-like policy sync params (optional) -----
  if (node->has_parameter("syncPolicyUpdates")) {
    sync_policy_updates_ = node->get_parameter("syncPolicyUpdates").as_bool();
  } else {
    sync_policy_updates_ = node->declare_parameter<bool>("syncPolicyUpdates", true);
  }

  if (node->has_parameter("mpcDesiredFrequency")) {
    mpc_desired_frequency_ = node->get_parameter("mpcDesiredFrequency").as_double();
  } else {
    mpc_desired_frequency_ = node->declare_parameter<double>("mpcDesiredFrequency", 100.0);
  }

  if (node->has_parameter("mrtDesiredFrequency")) {
    mrt_desired_frequency_ = node->get_parameter("mrtDesiredFrequency").as_double();
  } else {
    mrt_desired_frequency_ = node->declare_parameter<double>("mrtDesiredFrequency", 250.0);
  }

  if (node->has_parameter("policyTimeToleranceFactor")) {
    policy_time_tolerance_factor_ = node->get_parameter("policyTimeToleranceFactor").as_double();
  } else {
    policy_time_tolerance_factor_ = node->declare_parameter<double>("policyTimeToleranceFactor", 0.1);
  }

  if (node->has_parameter("maxPolicyWaitSeconds")) {
    max_policy_wait_seconds_ = node->get_parameter("maxPolicyWaitSeconds").as_double();
  } else {
    max_policy_wait_seconds_ = node->declare_parameter<double>("maxPolicyWaitSeconds", 0.0);
  }

  mpc_desired_frequency_ = std::max(1e-3, mpc_desired_frequency_);
  mrt_desired_frequency_ = std::max(1e-3, mrt_desired_frequency_);
  policy_time_tolerance_factor_ = std::clamp(policy_time_tolerance_factor_, 0.0, 1.0);

  // dummy behavior: floor division
  mpc_update_ratio_ = std::max<size_t>(
    static_cast<size_t>(mrt_desired_frequency_ / mpc_desired_frequency_), 1);

  RCLCPP_INFO(
    node->get_logger(),
    "[OCS2Controller] policy sync:\n"
    "  syncPolicyUpdates: %s\n"
    "  mpcDesiredFrequency: %.3f\n"
    "  mrtDesiredFrequency: %.3f\n"
    "  mpcUpdateRatio: %zu\n"
    "  policyTimeToleranceFactor: %.3f\n"
    "  maxPolicyWaitSeconds: %.3f (0=auto)",
    sync_policy_updates_ ? "true" : "false",
    mpc_desired_frequency_,
    mrt_desired_frequency_,
    mpc_update_ratio_,
    policy_time_tolerance_factor_,
    max_policy_wait_seconds_);

  if (task_file_.empty() || lib_folder_.empty() || urdf_file_.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "[OCS2Controller] parameters 'taskFile', 'libFolder' or 'urdfFile' are empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // ----- create interface -----
  try {
    interface_ = std::make_unique<MobileManipulatorInterface>(task_file_, lib_folder_, urdf_file_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // ----- visualization (optional) -----
  try {
    visualization_node_ = std::make_shared<rclcpp::Node>(
      node->get_name() + std::string("_visualizer"), node->get_namespace());
    visualization_ = std::make_unique<MobileManipulatorVisualization>(
      visualization_node_, *interface_, task_file_, urdf_file_, global_frame_);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "[OCS2Controller] visualization init failed: %s", e.what());
    visualization_.reset();
    visualization_node_.reset();
  }

  // ----- MRT -----
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

  // dims
  const auto & info = interface_->getManipulatorModelInfo();
  state_dim_ = info.stateDim;
  input_dim_ = info.inputDim;

  // buffers
  initial_observation_.state = vector_t::Zero(state_dim_);
  initial_observation_.input = vector_t::Zero(input_dim_);
  initial_observation_.time = 0.0;
  initial_observation_.mode = 0;
  initial_target_ = TargetTrajectories();

  last_command_ = vector_t::Zero(input_dim_);
  last_base_cmd_ = vector_t::Zero(2);
  last_arm_pos_cmd_.assign(static_cast<size_t>(info.armDim), 0.0);

  // pubs/subs
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
  loop_counter_ = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!initializeHandles()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "[OCS2Controller] failed to initialize state handles.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto now = get_node()->now();

  // build initial observation
  initial_observation_ = buildObservation(now);
  initial_target_ = computeInitialTarget(initial_observation_.state, initial_observation_.time);

  // initialize last_arm_pos_cmd_ to current joints to guarantee alpha=0 truly holds
  const size_t arm_dim = arm_joint_names_.size();
  last_arm_pos_cmd_.resize(arm_dim, 0.0);
  for (size_t i = 0; i < arm_dim; ++i) {
    const Eigen::Index si = 3 + static_cast<Eigen::Index>(i);
    if (si < initial_observation_.state.size()) {
      last_arm_pos_cmd_[i] = initial_observation_.state(si);
    }
  }
  last_base_cmd_ = vector_t::Zero(2);
  last_command_.setZero();

  // hold immediately
  applyHoldCommand(initial_observation_);

  if (mrt_) {
    mrt_->reset();
    mpc_reset_done_ = false;
    resetMpc();
  }

  loop_counter_ = 0;
  handles_initialized_ = true;

  RCLCPP_INFO(get_node()->get_logger(),
    "[OCS2Controller] activated: hold current state, wait for MPC policy.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  handles_initialized_ = false;
  joint_position_states_.clear();
  joint_position_commands_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OCS2Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

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

bool OCS2Controller::policyUpdatedForTime(double time, double policy_dt)
{
  if (!mrt_) return false;

  const bool policy_updated = mrt_->updatePolicy();
  if (!policy_updated) return false;

  // Safe to call getPolicy() now
  const auto& policy = mrt_->getPolicy();
  if (policy.timeTrajectory_.empty()) return false;

  const double t0 = policy.timeTrajectory_.front();
  const double tol = policy_time_tolerance_factor_ * policy_dt;

  return std::abs(time - t0) < tol;
}

bool OCS2Controller::waitForFreshPolicy(double desired_time, double policy_dt)
{
  if (!mrt_) return false;

  // auto wait time: up to ~2 MPC periods * ratio (so it may slow down like dummy)
  double max_wait = max_policy_wait_seconds_;
  if (max_wait <= 0.0) {
    const double mpc_period = 1.0 / std::max(1e-3, mpc_desired_frequency_);
    max_wait = 2.0 * mpc_period;   // enough for “next” solve
  }
  max_wait = std::max(max_wait, 0.0);

  const auto start = get_node()->now();
  while (rclcpp::ok()) {
    mrt_->spinMRT();

    if (policyUpdatedForTime(desired_time, policy_dt)) {
      return true;
    }

    const double waited = (get_node()->now() - start).seconds();
    if (waited > max_wait) {
      return false;
    }
  }
  return false;
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

  // policy_dt is based on MPC frequency (like dummy)
  const double policy_dt = 1.0 / std::max(1e-3, mpc_desired_frequency_);

  // 1) Build observation
  auto observation = buildObservation(time);

  // 2) Dummy-like: only “gate / wait” every mpc_update_ratio_ steps
  const bool at_mpc_boundary = sync_policy_updates_ && (loop_counter_ % mpc_update_ratio_ == 0);

  if (at_mpc_boundary) {
    // Send observation to MRT (so MPC solves around this time)
    mrt_->setCurrentObservation(observation);

    // Wait until a fresh policy arrives (or timeout), otherwise HOLD
    const bool fresh = waitForFreshPolicy(observation.time, policy_dt);

    if (!fresh || !mrt_->initialPolicyReceived()) {
      applyHoldCommand(observation);
      ++loop_counter_;
      return controller_interface::return_type::OK;
    }
  } else {
    // Non-boundary: best-effort update, no waiting (keeps last policy)
    mrt_->spinMRT();
    (void)mrt_->updatePolicy();
    if (!mrt_->initialPolicyReceived()) {
      applyHoldCommand(observation);
      ++loop_counter_;
      return controller_interface::return_type::OK;
    }
  }

  // 3) Before rollout: guard against “requested time > received plan”
  const auto& policy = mrt_->getPolicy();
  if (policy.timeTrajectory_.empty()) {
    applyHoldCommand(observation);
    ++loop_counter_;
    return controller_interface::return_type::OK;
  }

  const double t_req = observation.time + future_time_offset_;
  const double t_end = policy.timeTrajectory_.back();

  if (t_req > t_end) {
    // This directly addresses your log:
    // "requested currentTime is greater than the received plan"
    applyHoldCommand(observation);
    ++loop_counter_;
    return controller_interface::return_type::OK;
  }

  // 4) Rollout one controller period
  vector_t x_next(state_dim_);
  vector_t u_end(input_dim_);
  size_t mode_end = 0;

  mrt_->rolloutPolicy(
    t_req,
    observation.state,
    dt,
    x_next,
    u_end,
    mode_end
  );

  // 5) Apply command with alpha LPF (base vel + arm pos)
  applyFilteredRolloutCommand(u_end, x_next);

  // For obs.input next time (not critical, but keep consistent)
  last_command_ = u_end;

  // visualization (optional): only safe after updatePolicy was called
  if (visualization_) {
    visualization_->update(observation.state, policy, mrt_->getCommand());
  }

  ++loop_counter_;
  return controller_interface::return_type::OK;
}

void OCS2Controller::applyHoldCommand(const SystemObservation& observation)
{
  // base hold: publish zero twist
  if (base_cmd_pub_) {
    geometry_msgs::msg::Twist base_cmd;
    base_cmd.linear.x = 0.0;
    base_cmd.angular.z = 0.0;
    base_cmd_pub_->publish(base_cmd);
  }
  last_base_cmd_ = vector_t::Zero(2);

  // arm hold: command current measured joint positions
  const size_t arm_dim = arm_joint_names_.size();
  if (!use_moveit_servo_ && joint_position_commands_.size() >= arm_dim) {
    for (size_t i = 0; i < arm_dim; ++i) {
      const Eigen::Index si = 3 + static_cast<Eigen::Index>(i);
      if (si < observation.state.size()) {
        joint_position_commands_[i]->set_value(observation.state(si));
        if (i < last_arm_pos_cmd_.size()) {
          last_arm_pos_cmd_[i] = observation.state(si);
        }
      }
    }
  }

  // keep last_command_ zero-ish to avoid “alpha=0 keeps moving”
  if (last_command_.size() == input_dim_) {
    last_command_.setZero();
  }
}

void OCS2Controller::applyFilteredRolloutCommand(const vector_t& u_end, const vector_t& x_next)
{
  const double a = command_smoothing_alpha_;

  // ---- base (vel) LPF ----
  vector_t u_base(2);
  u_base.setZero();
  if (u_end.size() >= 2) {
    u_base(0) = u_end(0);
    u_base(1) = u_end(1);
  }

  vector_t blended_base = a * u_base + (1.0 - a) * last_base_cmd_;
  last_base_cmd_ = blended_base;

  if (base_cmd_pub_) {
    geometry_msgs::msg::Twist base_cmd;
    base_cmd.linear.x = blended_base(0);
    base_cmd.angular.z = blended_base(1);
    base_cmd_pub_->publish(base_cmd);
  }

  // ---- arm (pos) LPF ----
  const size_t arm_dim = arm_joint_names_.size();
  if (arm_dim == 0) return;

  if (!use_moveit_servo_) {
    if (joint_position_commands_.size() < arm_dim) return;
    if (last_arm_pos_cmd_.size() != arm_dim) last_arm_pos_cmd_.assign(arm_dim, 0.0);

    for (size_t i = 0; i < arm_dim; ++i) {
      const Eigen::Index si = 3 + static_cast<Eigen::Index>(i);
      if (si >= x_next.size()) continue;

      const double q_next = x_next(si);
      const double q_cmd = a * q_next + (1.0 - a) * last_arm_pos_cmd_[i];
      last_arm_pos_cmd_[i] = q_cmd;

      joint_position_commands_[i]->set_value(q_cmd);
    }
  } else {
    // If you later want MoveIt Servo mode, you can still publish JointJog here.
    // (Not changing your core impls now.)
  }
}

void OCS2Controller::resetMpc()
{
  if (mpc_reset_done_ || !mrt_) {
    return;
  }

  try {
    const auto now = get_node()->now();

    initial_observation_ = buildObservation(now);
    initial_target_ = computeInitialTarget(initial_observation_.state, initial_observation_.time);

    mrt_->resetMpcNode(initial_target_);
    mpc_reset_done_ = true;

    RCLCPP_INFO(get_node()->get_logger(), "[OCS2Controller] requested MPC reset.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "[OCS2Controller] Failed to reset MPC node: %s", e.what());
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

  const vector_t zero_input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
  return TargetTrajectories({time}, {init_target}, {zero_input});
}

bool OCS2Controller::initializeHandles()
{
  auto matches_name = [](const std::string & resource_name, const std::string & expected_joint) {
    if (resource_name == expected_joint) return true;
    const auto slash_idx = resource_name.find('/');
    if (slash_idx == std::string::npos) return false;
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
      RCLCPP_ERROR(get_node()->get_logger(),
        "[OCS2Controller] missing position state for joint '%s'.", joint.c_str());
      ok = false;
      continue;
    }
    joint_position_states_.push_back(pos);

    if (!use_moveit_servo_) {
      auto * pos_cmd = find_command(joint, hardware_interface::HW_IF_POSITION);
      if (!pos_cmd) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "[OCS2Controller] missing position command for joint '%s'.", joint.c_str());
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
