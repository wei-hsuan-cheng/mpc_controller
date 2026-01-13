#ifndef MPC_CONTROLLER__OCS2_CONTROLLER_HPP_
#define MPC_CONTROLLER__OCS2_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include "mpc_controller/visualization/MobileManipulatorVisualization.h"

namespace mpc_controller
{

class OCS2Controller : public controller_interface::ControllerInterface
{
public:
  OCS2Controller() = default;
  ~OCS2Controller() override = default;

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
  // Internal helpers
  void resetMpc();
  ocs2::SystemObservation buildObservation(const rclcpp::Time & time) const;
  void applyCommand(const ocs2::vector_t & command, double dt);
  bool initializeHandles();
  ocs2::TargetTrajectories computeInitialTarget(const ocs2::vector_t & state, double time) const;

  // Configuration
  std::vector<std::string> arm_joint_names_;
  std::string global_frame_{"odom"};

  // ROS topics
  std::string base_cmd_topic_{"/mb_servo_controller/cmd_vel"};
  std::string odom_topic_{"/mb_servo_controller/odom"};
  std::string joint_jog_topic_{"/servo_controller/delta_joint_cmds"};

  // State handles (from hardware)
  std::vector<hardware_interface::LoanedStateInterface *> joint_position_states_;
  // Command handles (optional, for direct hardware position control)
  std::vector<hardware_interface::LoanedCommandInterface *> joint_position_commands_;

  // OCS2 components
  std::unique_ptr<ocs2::mobile_manipulator::MobileManipulatorInterface> interface_;
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;
  rclcpp::Node::SharedPtr mrt_node_;

  ocs2::vector_t last_command_;
  ocs2::SystemObservation initial_observation_;
  ocs2::TargetTrajectories initial_target_;

  // Base twist / odometry
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_cmd_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_jog_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  mutable std::mutex odom_mutex_;
  std::array<double, 3> base_pose_from_odom_{0.0, 0.0, 0.0};

  // Params
  std::string task_file_;
  std::string lib_folder_;
  std::string urdf_file_;
  double future_time_offset_{0.02};
  double command_smoothing_alpha_{1.0};

  size_t state_dim_{0};
  size_t input_dim_{0};
  bool mpc_reset_done_{false};
  bool handles_initialized_{false};

  // Control mode
  bool use_moveit_servo_{false};

  // Visualization
  rclcpp::Node::SharedPtr visualization_node_;
  std::unique_ptr<MobileManipulatorVisualization> visualization_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__OCS2_CONTROLLER_HPP_
