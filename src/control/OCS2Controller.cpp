#include "mpc_controller/control/OCS2Controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace mpc_controller
{

using ocs2::SystemObservation;
using ocs2::TargetTrajectories;
using ocs2::vector_t;
using ocs2::mobile_manipulator_mpc::MobileManipulatorInterface;
using ocs2::mobile_manipulator_mpc::ManipulatorModelType;

// ===== Param helpers =====

void OCS2Controller::declareIfUndeclaredNotSet(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name)
{
  if (node->has_parameter(name)) {
    return;
  }
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  node->declare_parameter(name, rclcpp::ParameterValue{}, desc);
}

double OCS2Controller::getParamAsDouble(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& name,
  double default_value)
{
  declareIfUndeclaredNotSet(node, name);

  rclcpp::Parameter p;
  if (!node->get_parameter(name, p)) return default_value;

  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return default_value;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return p.as_double();
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(p.as_int());
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return std::stod(p.as_string());

  throw std::runtime_error("Parameter '" + name + "' must be int or double.");
}

std::string OCS2Controller::getParamAsString(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& name,
  const std::string& default_value)
{
  declareIfUndeclaredNotSet(node, name);

  rclcpp::Parameter p;
  if (!node->get_parameter(name, p)) return default_value;

  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return default_value;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return p.as_string();

  throw std::runtime_error("Parameter '" + name + "' must be string.");
}

bool OCS2Controller::getParamAsBool(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& name,
  bool default_value)
{
  declareIfUndeclaredNotSet(node, name);

  rclcpp::Parameter p;
  if (!node->get_parameter(name, p)) return default_value;

  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return default_value;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) return p.as_bool();

  throw std::runtime_error("Parameter '" + name + "' must be bool.");
}

std::vector<std::string> OCS2Controller::getParamAsStringArray(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& name,
  const std::vector<std::string>& default_value)
{
  declareIfUndeclaredNotSet(node, name);

  rclcpp::Parameter p;
  if (!node->get_parameter(name, p)) return default_value;

  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return default_value;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) return p.as_string_array();

  throw std::runtime_error("Parameter '" + name + "' must be string array.");
}

// ===== Loop helpers =====

controller_interface::CallbackReturn OCS2Controller::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

OCS2Controller::LoopMode OCS2Controller::parseLoopMode(const std::string & s)
{
  if (s == "sync" || s == "synchronized") return LoopMode::kSynchronized;
  if (s == "realtime" || s == "rt") return LoopMode::kRealtime;
  return LoopMode::kAuto;
}

void OCS2Controller::resolveLoopMode()
{
  if (loop_mode_ == LoopMode::kAuto) {
    loop_mode_ = (mpc_desired_frequency_ > 0.0) ? LoopMode::kSynchronized : LoopMode::kRealtime;
  }
}

OCS2Controller::ArmCommandMode OCS2Controller::parseArmCommandMode(const std::string& s)
{
  if (s == "integrate_u" || s == "integrate" || s == "u") {
    return ArmCommandMode::kIntegrateU;
  }
  return ArmCommandMode::kState;
}

// ===== Model layout =====
void OCS2Controller::resolveModelLayout()
{
  // Default: wheel-based (type 1), unicycle layout until interface data is available.
  model_type_ = ManipulatorModelType::WheelBasedMobileManipulator;
  base_state_dim_ = 3;
  base_input_dim_ = 2;
  arm_state_offset_ = 3;
  arm_input_offset_ = 2;

  if (!interface_) return;

  const auto& info = interface_->getManipulatorModelInfo();
  model_type_ = info.manipulatorModelType;

  switch (model_type_) {
    case ManipulatorModelType::DefaultManipulator:
      // state=[q_arm], input=[dq_arm]
      base_state_dim_ = 0;
      base_input_dim_ = 0;
      arm_state_offset_ = 0;
      arm_input_offset_ = 0;
      break;

    case ManipulatorModelType::WheelBasedMobileManipulator:
      // state=[x y yaw q_arm], input=[vx omega dq_arm] or [vx vy omega dq_arm]
      base_state_dim_ = 3;
      base_input_dim_ = info.inputDim - info.armDim;
      arm_state_offset_ = 3;
      arm_input_offset_ = base_input_dim_;
      break;

    case ManipulatorModelType::FloatingArmManipulator:
      // state=[x y z r p y q_arm], input=[dq_arm]
      base_state_dim_ = 6;
      base_input_dim_ = 0;
      arm_state_offset_ = 6;
      arm_input_offset_ = 0;
      break;

    case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
      // state=[x y z r p y q_arm], input=[vx vy vz wx wy wz dq_arm]
      base_state_dim_ = 6;
      base_input_dim_ = 6;
      arm_state_offset_ = 6;
      arm_input_offset_ = 6;
      break;

    default:
      throw std::runtime_error("[OCS2Controller] Unsupported manipulatorModelType.");
  }
}

controller_interface::CallbackReturn OCS2Controller::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  // --- required paths ---
  task_file_   = getParamAsString(node, "taskFile", "");
  lib_folder_  = getParamAsString(node, "libFolder", "");
  urdf_file_   = getParamAsString(node, "urdfFile", "");

  global_frame_ = getParamAsString(node, "globalFrame", "world");

  // --- topics ---
  base_cmd_topic_  = getParamAsString(node, "base_cmd_topic", base_cmd_topic_);
  odom_topic_      = getParamAsString(node, "odom_topic", odom_topic_);
  joint_jog_topic_ = getParamAsString(node, "joint_jog_topic", joint_jog_topic_);

  // --- numeric ---
  future_time_offset_       = getParamAsDouble(node, "futureTimeOffset", 0.0);
  command_smoothing_alpha_  = getParamAsDouble(node, "commandSmoothingAlpha", 1.0);

  mpc_desired_frequency_    = getParamAsDouble(node, "mpcDesiredFrequency", 100.0);
  mrt_desired_frequency_    = getParamAsDouble(node, "mrtDesiredFrequency", 250.0);

  policy_time_tolerance_factor_ = getParamAsDouble(node, "policyTimeToleranceFactor", 1.0);
  max_policy_wait_seconds_      = getParamAsDouble(node, "maxPolicyWaitSeconds", 0.0);

  perf_log_period_sec_ = getParamAsDouble(node, "perfLogPeriod", 2.0);

  // --- bool ---
  use_moveit_servo_ = getParamAsBool(node, "use_moveit_servo", false);

  // --- joints ---
  arm_joint_names_ = getParamAsStringArray(node, "arm_joints", {});

  // --- loop mode ---
  loop_mode_str_ = getParamAsString(node, "loopMode", "auto");
  loop_mode_ = parseLoopMode(loop_mode_str_);

  // --- arm command mode ---
  arm_command_mode_str_ = getParamAsString(node, "armCommandMode", "state");
  arm_command_mode_ = parseArmCommandMode(arm_command_mode_str_);
  integrate_u_use_measured_state_ = getParamAsBool(node, "integrateUUseMeasuredState", true);

  command_smoothing_alpha_ = std::clamp(command_smoothing_alpha_, 0.0, 1.0);
  policy_time_tolerance_factor_ = std::clamp(policy_time_tolerance_factor_, 0.0, 10.0);

  mrt_desired_frequency_ = std::max(1e-3, mrt_desired_frequency_);
  if (mpc_desired_frequency_ > 0.0) {
    mpc_desired_frequency_ = std::max(1e-3, mpc_desired_frequency_);
  }

  mrt_dt_ = 1.0 / std::max(1.0, mrt_desired_frequency_);
  policy_dt_ = (mpc_desired_frequency_ > 0.0)
    ? (1.0 / std::max(1.0, std::abs(mpc_desired_frequency_)))
    : mrt_dt_;

  if (mpc_desired_frequency_ > 0.0) {
    mpc_update_ratio_ =
      std::max<size_t>(static_cast<size_t>(mrt_desired_frequency_ / mpc_desired_frequency_), 1);
  } else {
    mpc_update_ratio_ = 1;
  }

  if (max_policy_wait_seconds_ <= 0.0) {
    max_policy_wait_seconds_ = (mpc_desired_frequency_ > 0.0) ? (2.0 * policy_dt_) : 0.5;
  }

  resolveLoopMode();

  if (task_file_.empty() || lib_folder_.empty() || urdf_file_.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "[OCS2Controller] parameters 'taskFile', 'libFolder' or 'urdfFile' are empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // --- interface ---
  try {
    interface_ = std::make_unique<MobileManipulatorInterface>(task_file_, lib_folder_, urdf_file_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Resolve model-dependent layout AFTER interface is created
  try {
    resolveModelLayout();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "[OCS2Controller] resolveModelLayout failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto & info = interface_->getManipulatorModelInfo();
  state_dim_ = info.stateDim;
  input_dim_ = info.inputDim;

  RCLCPP_INFO(
    node->get_logger(),
    "[OCS2Controller] modelType=%d | stateDim=%d inputDim=%d | base_state_dim=%zu base_input_dim=%zu | arm_state_offset=%zu arm_input_offset=%zu | armCommandMode=%s",
    static_cast<int>(model_type_), state_dim_, input_dim_,
    base_state_dim_, base_input_dim_, arm_state_offset_, arm_input_offset_, arm_command_mode_str_.c_str());

  // --- visualization (optional) ---
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

  // --- MRT ---
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

  // ---- buffers ----
  initial_observation_.state = vector_t::Zero(state_dim_);
  initial_observation_.input = vector_t::Zero(input_dim_);
  initial_observation_.time = 0.0;
  initial_observation_.mode = 0;
  initial_target_ = TargetTrajectories();

  last_command_ = vector_t::Zero(input_dim_);
  last_base_cmd_ = vector_t::Zero(static_cast<Eigen::Index>(base_input_dim_));
  last_arm_pos_cmd_.assign(static_cast<size_t>(info.armDim), 0.0);

  // ---- pubs/subs: only enable what is meaningful ----
  if (model_type_ == ManipulatorModelType::WheelBasedMobileManipulator) {
    base_cmd_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      base_cmd_topic_, rclcpp::SystemDefaultsQoS());
  } else {
    base_cmd_pub_.reset();
  }

  joint_jog_pub_ = node->create_publisher<control_msgs::msg::JointJog>(
    joint_jog_topic_, rclcpp::SystemDefaultsQoS());

  // subscribe odom only if base state contains at least (x,y,yaw)
  if (base_state_dim_ >= 3) {
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
  } else {
    odom_sub_.reset();
  }

  // loop reset
  mpc_reset_done_ = false;
  handles_initialized_ = false;
  loop_counter_ = 0;
  virtual_time_ = 0.0;
  waiting_for_fresh_policy_ = false;
  boundary_time_ = 0.0;

  // perf reset
  perf_inited_ = false;
  perf_policy_updates_ = 0;
  perf_update_policy_calls_ = 0;
  perf_update_policy_ms_sum_ = 0.0;
  perf_latest_policy_latency_ms_ = NAN;
  perf_latest_boundary_latency_ms_ = NAN;
  have_last_policy_t0_ = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_activate(const rclcpp_lifecycle::State &)
{
  if (!initializeHandles()) {
    RCLCPP_ERROR(get_node()->get_logger(), "[OCS2Controller] failed to initialize state handles.");
    return controller_interface::CallbackReturn::ERROR;
  }

  loop_counter_ = 0;
  virtual_time_ = 0.0;
  waiting_for_fresh_policy_ = false;
  boundary_time_ = 0.0;

  initial_observation_ = buildObservation(virtual_time_);
  initial_target_ = computeInitialTarget(initial_observation_.state, initial_observation_.time);

  // init LPF memory to current
  const size_t arm_dim = arm_joint_names_.size();
  last_arm_pos_cmd_.resize(arm_dim, 0.0);
  for (size_t i = 0; i < arm_dim; ++i) {
    const Eigen::Index si = static_cast<Eigen::Index>(arm_state_offset_ + i);
    if (si < initial_observation_.state.size()) {
      last_arm_pos_cmd_[i] = initial_observation_.state(si);
    }
  }

  if (last_base_cmd_.size() > 0) last_base_cmd_.setZero();
  if (last_command_.size() == input_dim_) last_command_.setZero();

  applyHoldCommand(initial_observation_);

  if (mrt_) {
    mrt_->reset();
    mpc_reset_done_ = false;
    resetMpc();
    mrt_->setCurrentObservation(initial_observation_);
  }

  handles_initialized_ = true;
  perf_inited_ = false;

  RCLCPP_INFO(get_node()->get_logger(),
    "[OCS2Controller] activated: dummy-like time base (dt=%.6f), perf log period=%.3fs",
    mrt_dt_, perf_log_period_sec_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OCS2Controller::on_deactivate(const rclcpp_lifecycle::State &)
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

bool OCS2Controller::updatePolicyTimed(double obs_time_sec)
{
  if (!mrt_) return false;

  const auto t0 = steady_clock_.now();
  mrt_->updatePolicy();
  const auto t1 = steady_clock_.now();

  const double dt_ms = (t1 - t0).seconds() * 1000.0;
  perf_update_policy_calls_ += 1;
  perf_update_policy_ms_sum_ += dt_ms;

  if (!mrt_->initialPolicyReceived()) return false;

  const auto& policy = mrt_->getPolicy();
  if (policy.timeTrajectory_.empty()) return false;

  const double policy_t0 = policy.timeTrajectory_.front();

  if (!have_last_policy_t0_ || std::abs(policy_t0 - last_policy_t0_) > 1e-12) {
    perf_policy_updates_ += 1;
    last_policy_t0_ = policy_t0;
    have_last_policy_t0_ = true;
  }

  perf_latest_policy_latency_ms_ = (obs_time_sec - policy_t0) * 1000.0;
  return true;
}

bool OCS2Controller::policyIsFreshForTime(double desired_time, double policy_dt)
{
  if (!mrt_ || !mrt_->initialPolicyReceived()) return false;

  const auto& policy = mrt_->getPolicy();
  if (policy.timeTrajectory_.empty()) return false;

  const double t0  = policy.timeTrajectory_.front();
  const double tol = policy_time_tolerance_factor_ * policy_dt;
  const double diff = std::abs(desired_time - t0);

  const bool ok = (diff < tol);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "[policy sync] desired=%.3f, policy_t0=%.3f, |diff|=%.3f > tol=%.3f (dt=%.3f, factor=%.3f)",
      desired_time, t0, diff, tol, policy_dt, policy_time_tolerance_factor_);
  }
  return ok;
}

void OCS2Controller::maybeLogPerf()
{
  if (perf_log_period_sec_ <= 0.0) return;

  const auto now = steady_clock_.now();
  if (!perf_inited_) {
    perf_inited_ = true;
    perf_window_start_ = now;
    perf_last_log_ = now;
    return;
  }

  const double elapsed = (now - perf_last_log_).seconds();
  if (elapsed < perf_log_period_sec_) return;

  const double window_sec = std::max(1e-9, (now - perf_window_start_).seconds());

  const double policy_rate_hz =
    static_cast<double>(perf_policy_updates_) / window_sec;

  const double avg_update_policy_ms =
    (perf_update_policy_calls_ > 0)
      ? (perf_update_policy_ms_sum_ / static_cast<double>(perf_update_policy_calls_))
      : 0.0;

  const double boundary_ms = perf_latest_boundary_latency_ms_;
  const double latency_ms = perf_latest_policy_latency_ms_;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "\n####################\n"
    "[perf] window=%.2f [s] | policy_rate=%.2f [Hz] | updatePolicy(avg)=%.3f [ms] | "
    "policy_staleness=%.3f [ms] | boundary_latency=%.3f [ms]"
    "\n####################\n",
    window_sec,
    policy_rate_hz,
    avg_update_policy_ms,
    std::isfinite(latency_ms) ? latency_ms : -1.0,
    std::isfinite(boundary_ms) ? boundary_ms : -1.0);

  perf_window_start_ = now;
  perf_last_log_ = now;
  perf_policy_updates_ = 0;
  perf_update_policy_calls_ = 0;
  perf_update_policy_ms_sum_ = 0.0;
}

controller_interface::return_type OCS2Controller::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!handles_initialized_ || !mrt_) {
    return controller_interface::return_type::ERROR;
  }

  mrt_->spinMRT();

  controller_interface::return_type ret =
    (loop_mode_ == LoopMode::kRealtime) ? runRealtimeLoopStep() : runSynchronizedLoopStep();

  maybeLogPerf();
  return ret;
}

controller_interface::return_type OCS2Controller::runRealtimeLoopStep()
{
  SystemObservation obs = buildObservation(virtual_time_);
  mrt_->setCurrentObservation(obs);

  (void)updatePolicyTimed(obs.time);

  if (!mrt_->initialPolicyReceived()) {
    applyHoldCommand(obs);
    return controller_interface::return_type::OK;
  }

  const auto & policy = mrt_->getPolicy();
  if (policy.timeTrajectory_.empty()) {
    applyHoldCommand(obs);
    return controller_interface::return_type::OK;
  }

  const double t_req = obs.time + future_time_offset_;
  if (t_req > policy.timeTrajectory_.back()) {
    applyHoldCommand(obs);
    return controller_interface::return_type::OK;
  }

  vector_t x_next(state_dim_);
  vector_t u_end(input_dim_);
  size_t mode_end = 0;

  mrt_->rolloutPolicy(t_req, obs.state, mrt_dt_, x_next, u_end, mode_end);
  applyFilteredRolloutCommand(u_end, x_next, mrt_dt_);
  last_command_ = u_end;

  virtual_time_ += mrt_dt_;
  ++loop_counter_;

  if (visualization_) {
    visualization_->update(obs.state, policy, mrt_->getCommand());
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type OCS2Controller::runSynchronizedLoopStep()
{
  const bool at_boundary = (loop_counter_ % mpc_update_ratio_ == 0);
  SystemObservation obs = buildObservation(virtual_time_);

  (void)updatePolicyTimed(obs.time);

  if (at_boundary) {
    if (!waiting_for_fresh_policy_) {
      waiting_for_fresh_policy_ = true;
      wait_start_steady_ = steady_clock_.now();
      boundary_time_ = obs.time;
      mrt_->setCurrentObservation(obs);
    }

    const bool fresh = policyIsFreshForTime(boundary_time_, policy_dt_);
    if (!fresh) {
      applyHoldCommand(obs);

      const double waited = (steady_clock_.now() - wait_start_steady_).seconds();
      if (waited <= max_policy_wait_seconds_) {
        return controller_interface::return_type::OK;
      }

      if (!mrt_->initialPolicyReceived()) {
        return controller_interface::return_type::OK;
      }
    }

    perf_latest_boundary_latency_ms_ =
      (steady_clock_.now() - wait_start_steady_).seconds() * 1000.0;

    waiting_for_fresh_policy_ = false;
  }

  if (!mrt_->initialPolicyReceived()) {
    applyHoldCommand(obs);
    return controller_interface::return_type::OK;
  }

  const auto & policy = mrt_->getPolicy();
  if (policy.timeTrajectory_.empty()) {
    applyHoldCommand(obs);
    return controller_interface::return_type::OK;
  }

  const double t_req = obs.time + future_time_offset_;
  if (t_req > policy.timeTrajectory_.back()) {
    applyHoldCommand(obs);
    return controller_interface::return_type::OK;
  }

  vector_t x_next(state_dim_);
  vector_t u_end(input_dim_);
  size_t mode_end = 0;

  mrt_->rolloutPolicy(t_req, obs.state, mrt_dt_, x_next, u_end, mode_end);
  applyFilteredRolloutCommand(u_end, x_next, mrt_dt_);
  last_command_ = u_end;

  SystemObservation next_obs;
  next_obs.time  = obs.time + mrt_dt_;
  next_obs.state = x_next;
  next_obs.input = u_end;
  next_obs.mode  = static_cast<int>(mode_end);

  const bool next_is_boundary = ((loop_counter_ + 1) % mpc_update_ratio_ == 0);
  if (next_is_boundary) {
    mrt_->setCurrentObservation(next_obs);
  }

  virtual_time_ += mrt_dt_;
  ++loop_counter_;

  if (visualization_) {
    visualization_->update(obs.state, policy, mrt_->getCommand());
  }

  return controller_interface::return_type::OK;
}

void OCS2Controller::applyHoldCommand(const SystemObservation & observation)
{
  // base hold: only for wheel-based model (Twist vx,[vy],omega)
  if (model_type_ == ManipulatorModelType::WheelBasedMobileManipulator && base_cmd_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    base_cmd_pub_->publish(cmd);
  }
  if (last_base_cmd_.size() > 0) last_base_cmd_.setZero();

  // arm hold
  const size_t arm_dim = arm_joint_names_.size();
  if (!use_moveit_servo_ && joint_position_commands_.size() >= arm_dim) {
    for (size_t i = 0; i < arm_dim; ++i) {
      const Eigen::Index si = static_cast<Eigen::Index>(arm_state_offset_ + i);
      if (si < observation.state.size()) {
        joint_position_commands_[i]->set_value(observation.state(si));
        if (i < last_arm_pos_cmd_.size()) last_arm_pos_cmd_[i] = observation.state(si);
      }
    }
  }

  if (last_command_.size() == input_dim_) last_command_.setZero();
}

void OCS2Controller::applyFilteredRolloutCommand(const vector_t & u_end, const vector_t & x_next, double dt_sec)
{
  const double a = command_smoothing_alpha_;

  // base LPF (vel) only for wheel-based
  if (model_type_ == ManipulatorModelType::WheelBasedMobileManipulator && base_input_dim_ >= 2) {
    vector_t u_base = vector_t::Zero(static_cast<Eigen::Index>(base_input_dim_));
    if (u_end.size() >= u_base.size()) {
      u_base = u_end.head(u_base.size());
    } else if (u_end.size() > 0) {
      u_base.head(u_end.size()) = u_end.head(u_end.size());
    }

    if (last_base_cmd_.size() != u_base.size()) {
      last_base_cmd_ = vector_t::Zero(u_base.size());
    }

    const vector_t blended_base = a * u_base + (1.0 - a) * last_base_cmd_;
    last_base_cmd_ = blended_base;

    if (base_cmd_pub_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = blended_base(0);
      if (base_input_dim_ >= 3) {
        cmd.linear.y = blended_base(1);
      }
      cmd.angular.z = blended_base(base_input_dim_ - 1);
      base_cmd_pub_->publish(cmd);
    }
  }

  // arm
  const size_t arm_dim = arm_joint_names_.size();
  if (arm_dim == 0) return;

  if (!use_moveit_servo_) {
    if (joint_position_commands_.size() < arm_dim) return;
    if (last_arm_pos_cmd_.size() != arm_dim) last_arm_pos_cmd_.assign(arm_dim, 0.0);

    // ensure sane dt for integration
    dt_sec = std::max(1e-6, dt_sec);

    if (arm_command_mode_ == ArmCommandMode::kState) {
      // MODE: state -> position
      for (size_t i = 0; i < arm_dim; ++i) {
        const Eigen::Index si = static_cast<Eigen::Index>(arm_state_offset_ + i);
        if (si >= x_next.size()) continue;

        const double q_next = x_next(si);
        const double q_cmd = a * q_next + (1.0 - a) * last_arm_pos_cmd_[i];
        last_arm_pos_cmd_[i] = q_cmd;
        joint_position_commands_[i]->set_value(q_cmd);
      }
    } else {
      // MODE: integrate_u -> position
      for (size_t i = 0; i < arm_dim; ++i) {
        const Eigen::Index ui = static_cast<Eigen::Index>(arm_input_offset_ + i);
        if (ui >= u_end.size()) continue;

        const double dq_cmd = u_end(ui);
        double q_base = last_arm_pos_cmd_[i];
        if (integrate_u_use_measured_state_) {
          if (i < joint_position_states_.size() && joint_position_states_[i] != nullptr) {
            q_base = joint_position_states_[i]->get_value();
          }
        }

        const double q_int = q_base + dq_cmd * dt_sec;
        // Same LPF form as state mode: blend new integrated position with previous value.
        const double q_cmd = a * q_int + (1.0 - a) * q_base;

        last_arm_pos_cmd_[i] = q_cmd;
        joint_position_commands_[i]->set_value(q_cmd);
      }
    }
  }
}

void OCS2Controller::resetMpc()
{
  if (mpc_reset_done_ || !mrt_) return;

  try {
    initial_observation_ = buildObservation(virtual_time_);
    initial_target_ = computeInitialTarget(initial_observation_.state, initial_observation_.time);
    mrt_->resetMpcNode(initial_target_);
    mpc_reset_done_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "[OCS2Controller] requested MPC reset.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "[OCS2Controller] Failed to reset MPC node: %s", e.what());
  }
}

SystemObservation OCS2Controller::buildObservation(double time_sec) const
{
  SystemObservation obs;
  obs.time = time_sec;
  obs.state = vector_t::Zero(state_dim_);
  obs.input = last_command_;
  obs.mode = 0;

  // base (only fill x,y,yaw if base_state_dim_ >= 3)
  if (base_state_dim_ >= 3) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    obs.state(0) = base_pose_from_odom_[0];
    obs.state(1) = base_pose_from_odom_[1];
    obs.state(2) = base_pose_from_odom_[2];
  }

  // arm joints
  for (size_t idx = 0; idx < arm_joint_names_.size(); ++idx) {
    const Eigen::Index si = static_cast<Eigen::Index>(arm_state_offset_ + idx);
    if (si < obs.state.size() && idx < joint_position_states_.size()) {
      obs.state(si) = joint_position_states_[idx]->get_value();
    }
  }

  return obs;
}

TargetTrajectories OCS2Controller::computeInitialTarget(const vector_t & state, double time) const
{
  vector_t init_target;

  const auto & pin_interface = interface_->getPinocchioInterface();
  const auto & model = pin_interface.getModel();
  auto data = pin_interface.getData();

  // NOTE:
  // state dimension here matches the pinocchio model created by MobileManipulatorInterface
  // for each manipulatorModelType, so forwardKinematics(model,data,state) is consistent.
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
    [this, &matches_name](const std::string & name, const std::string & interface) -> hardware_interface::LoanedStateInterface * {
      for (auto & state : state_interfaces_) {
        if (matches_name(state.get_name(), name) && state.get_interface_name() == interface) {
          return &state;
        }
      }
      return nullptr;
    };

  auto find_command =
    [this, &matches_name](const std::string & name, const std::string & interface) -> hardware_interface::LoanedCommandInterface * {
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
      RCLCPP_ERROR(get_node()->get_logger(), "[OCS2Controller] missing position state for joint '%s'.", joint.c_str());
      ok = false;
      continue;
    }
    joint_position_states_.push_back(pos);

    if (!use_moveit_servo_) {
      auto * pos_cmd = find_command(joint, hardware_interface::HW_IF_POSITION);
      if (!pos_cmd) {
        RCLCPP_ERROR(get_node()->get_logger(), "[OCS2Controller] missing position command for joint '%s'.", joint.c_str());
        ok = false;
      } else {
        joint_position_commands_.push_back(pos_cmd);
      }
    }
  }

  return ok;
}

}  // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(mpc_controller::OCS2Controller, controller_interface::ControllerInterface)
