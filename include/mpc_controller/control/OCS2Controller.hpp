#ifndef MPC_CONTROLLER__OCS2_CONTROLLER_HPP_
#define MPC_CONTROLLER__OCS2_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
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
  using MobileManipulatorInterface = ocs2::mobile_manipulator::MobileManipulatorInterface;

  // ===== Helpers =====
  bool initializeHandles();

  SystemObservation buildObservation(const rclcpp::Time & time) const;

  TargetTrajectories computeInitialTarget(const vector_t & state, double time) const;

  void resetMpc();

  // Hold behavior: base=0, arm=hold current joint pos (from observation.state)
  void applyHoldCommand(const SystemObservation& observation);

  // Apply rollout result with alpha LPF:
  // - base uses u_end (vel) LPF
  // - arm uses x_next (pos) LPF
  void applyFilteredRolloutCommand(const vector_t& u_end, const vector_t& x_next);

  // ---- Dummy-like policy freshness gate ----
  bool policyUpdatedForTime(double time, double policy_dt);
  bool waitForFreshPolicy(double desired_time, double policy_dt);

private:
  // ===== Parameters =====
  std::string task_file_;
  std::string lib_folder_;
  std::string urdf_file_;
  std::string global_frame_;

  double future_time_offset_{0.0};
  double command_smoothing_alpha_{1.0};
  bool use_moveit_servo_{false};

  std::vector<std::string> arm_joint_names_;

  std::string base_cmd_topic_{"/cmd_vel"};
  std::string odom_topic_{"/odom"};
  std::string joint_jog_topic_{"/servo_node/delta_joint_cmds"};

  // Dummy-like sync settings
  bool sync_policy_updates_{true};
  double mpc_desired_frequency_{100.0};   // from task.info (default)
  double mrt_desired_frequency_{250.0};   // from task.info (default)
  double policy_time_tolerance_factor_{0.1};  // same as dummy
  double max_policy_wait_seconds_{0.0};       // 0 -> auto

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
  std::array<double, 3> base_pose_from_odom_{0.0, 0.0, 0.0}; // x, y, yaw

  // ===== ros2_control handles =====
  std::vector<hardware_interface::LoanedStateInterface*> joint_position_states_;
  std::vector<hardware_interface::LoanedCommandInterface*> joint_position_commands_;

  // ===== Dimensions / buffers =====
  int state_dim_{0};
  int input_dim_{0};

  SystemObservation initial_observation_;
  TargetTrajectories initial_target_;

  // last OCS2 input (vel) used in obs.input
  vector_t last_command_;

  // For alpha semantics on base and arm
  ocs2::vector_t last_base_cmd_{ocs2::vector_t::Zero(2)};  // [v, w]
  std::vector<double> last_arm_pos_cmd_;        // size = arm_dim

  // MPC reset / activation flags
  bool handles_initialized_{false};
  bool mpc_reset_done_{false};

  // Dummy-like counters
  size_t loop_counter_{0};
  size_t mpc_update_ratio_{1};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__OCS2_CONTROLLER_HPP_
