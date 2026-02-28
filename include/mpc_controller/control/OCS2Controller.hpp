#ifndef MPC_CONTROLLER__OCS2_CONTROLLER_HPP_
#define MPC_CONTROLLER__OCS2_CONTROLLER_HPP_

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/ManipulatorModelInfo.h>  // <-- for ManipulatorModelType
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "mpc_controller/visualization/MobileManipulatorVisualization.h"

namespace mpc_controller
{

class OCS2Controller : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using vector_t = ocs2::vector_t;
  using SystemObservation = ocs2::SystemObservation;
  using TargetTrajectories = ocs2::TargetTrajectories;
  using MobileManipulatorInterface = ocs2::mobile_manipulator_mpc::MobileManipulatorInterface;
  using ManipulatorModelType = ocs2::mobile_manipulator_mpc::ManipulatorModelType;

  enum class LoopMode { kAuto, kSynchronized, kRealtime };

  // How to convert MPC outputs to arm position commands
  //  - kState:       use rollout state trajectory x_next as joint positions
  //  - kIntegrateU:  integrate dq (from rollout input u_end) into joint positions
  enum class ArmCommandMode { kState, kIntegrateU };

  // ===== Params helpers (robust to int/double overrides) =====
  static void declareIfUndeclaredNotSet(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name);

  static double getParamAsDouble(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,
    double default_value);

  static std::string getParamAsString(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,
    const std::string& default_value);

  static bool getParamAsBool(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,
    bool default_value);

  static std::vector<std::string> getParamAsStringArray(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,
    const std::vector<std::string>& default_value);

  // ===== Loop helpers =====
  static LoopMode parseLoopMode(const std::string & s);
  void resolveLoopMode();

  static ArmCommandMode parseArmCommandMode(const std::string& s);

  bool initializeHandles();
  SystemObservation buildObservation(double time_sec) const;
  TargetTrajectories computeInitialTarget(const vector_t & state, double time) const;

  void resetMpc();
  void applyHoldCommand(const SystemObservation & observation);
  void applyFilteredRolloutCommand(const vector_t & u_end, const vector_t & x_next, double dt_sec);

  controller_interface::return_type runSynchronizedLoopStep();
  controller_interface::return_type runRealtimeLoopStep();

  // Update policy once (timed) and update perf statistics.
  bool updatePolicyTimed(double obs_time_sec);

  // Check freshness using the CURRENT policy (no updatePolicy inside).
  bool policyIsFreshForTime(double desired_time, double policy_dt);

  // Perf logging
  void maybeLogPerf();

  // ===== Model layout =====
  void resolveModelLayout();

private:
  // ===== Parameters =====
  std::string task_file_;
  std::string lib_folder_;
  std::string urdf_file_;
  std::string global_frame_{"world"};

  double future_time_offset_{0.0};
  double command_smoothing_alpha_{1.0};
  bool use_moveit_servo_{false};
  std::vector<std::string> arm_joint_names_;

  std::string base_cmd_topic_{"/cmd_vel"};
  std::string odom_topic_{"/odom"};
  std::string joint_jog_topic_{"/servo_node/delta_joint_cmds"};

  // Dummy-like loop settings
  std::string loop_mode_str_{"auto"};
  LoopMode loop_mode_{LoopMode::kAuto};

  // Arm command conversion
  std::string arm_command_mode_str_{"state"};
  ArmCommandMode arm_command_mode_{ArmCommandMode::kState};
  bool integrate_u_use_measured_state_{true};

  double mpc_desired_frequency_{100.0};
  double mrt_desired_frequency_{250.0};
  double policy_time_tolerance_factor_{0.1};
  double max_policy_wait_seconds_{0.0};  // 0 -> auto

  // Perf logging period (sec). <=0 disables.
  double perf_log_period_sec_{2.0};

  // Derived
  double mrt_dt_{0.004};
  double policy_dt_{0.01};
  size_t mpc_update_ratio_{1};

  // ===== OCS2 objects =====
  std::unique_ptr<MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;
  std::shared_ptr<rclcpp::Node> mrt_node_;

  // Visualization (optional)
  std::shared_ptr<rclcpp::Node> visualization_node_;
  std::unique_ptr<MobileManipulatorVisualization> visualization_;

  // ===== ROS I/O =====
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Odom state
  mutable std::mutex odom_mutex_;
  std::array<double, 3> base_pose_from_odom_{0.0, 0.0, 0.0};  // x, y, yaw

  // ===== ros2_control handles =====
  std::vector<hardware_interface::LoanedStateInterface *> joint_position_states_;
  std::vector<hardware_interface::LoanedCommandInterface *> joint_position_commands_;

  // ===== Dimensions / buffers =====
  int state_dim_{0};
  int input_dim_{0};

  SystemObservation initial_observation_;
  TargetTrajectories initial_target_;

  // last OCS2 input used in obs.input
  vector_t last_command_;

  // ===== Model-dependent layout =====
  ManipulatorModelType model_type_{ManipulatorModelType::WheelBasedMobileManipulator};
  size_t base_state_dim_{3};
  size_t base_input_dim_{2};
  size_t arm_state_offset_{3};
  size_t arm_input_offset_{2};

  // Alpha LPF state
  vector_t last_base_cmd_{vector_t::Zero(0)};  // wheel-based: size=2 (unicycle) or 3 (omni), else empty
  std::vector<double> last_arm_pos_cmd_;

  // Loop state
  bool handles_initialized_{false};
  bool mpc_reset_done_{false};

  size_t loop_counter_{0};
  double virtual_time_{0.0};  // dummy-like time base

  // Non-blocking wait state for synchronized mode
  bool waiting_for_fresh_policy_{false};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time wait_start_steady_{0, 0, RCL_STEADY_TIME};
  double boundary_time_{0.0};

  // ===== Perf stats (steady clock based) =====
  bool perf_inited_{false};
  rclcpp::Time perf_window_start_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time perf_last_log_{0, 0, RCL_STEADY_TIME};

  // policy update rate
  size_t perf_policy_updates_{0};
  double last_policy_t0_{0.0};
  bool have_last_policy_t0_{false};

  // updatePolicy processing time (controller-side)
  size_t perf_update_policy_calls_{0};
  double perf_update_policy_ms_sum_{0.0};

  // latency / staleness (ms)
  double perf_latest_policy_latency_ms_{NAN};

  // boundary solve latency (ms) in sync mode
  double perf_latest_boundary_latency_ms_{NAN};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__OCS2_CONTROLLER_HPP_
