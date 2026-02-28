// Copyright (c) 2024.
// Restores the marker / trajectory visualization that used to live in
// MobileManipulatorDummyVisualization and exposes it as a helper that can be
// owned by the OCS2 ros2_control plugin.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/CommandData.h>
#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/reference/MobileManipulatorReferenceManager.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include "mpc_controller/visualization/EnvironmentCollisionVisualization.h"

namespace mpc_controller {

class MobileManipulatorVisualization {
public:
  MobileManipulatorVisualization(const rclcpp::Node::SharedPtr &node,
                                 const ocs2::mobile_manipulator_mpc::MobileManipulatorInterface &interface,
                                 const std::string &task_file,
                                 const std::string &urdf_file,
                                 const std::string &global_frame);

  void update(const ocs2::vector_t &current_state,
              const ocs2::PrimalSolution &policy,
              const ocs2::CommandData &command);

private:
  void launchVisualizerNode(const std::string &task_file, const std::string &urdf_file);
  void publishTargetTrajectories(const rclcpp::Time &time_stamp, const ocs2::TargetTrajectories &target);
  void publishOptimizedTrajectory(const rclcpp::Time &time_stamp, const ocs2::PrimalSolution &policy);
  void publishBaseTransform(const rclcpp::Time &time_stamp, const ocs2::vector_t &state);

  template <typename It>
  static void assignHeader(It first, It last, const std_msgs::msg::Header &header) {
    for (; first != last; ++first) {
      first->header = header;
    }
  }

  template <typename It>
  static void assignIncreasingId(It first, It last, int start = 0) {
    for (; first != last; ++first) {
      first->id = start++;
    }
  }

  rclcpp::Node::SharedPtr node_;
  ocs2::PinocchioInterface pinocchio_interface_;
  const ocs2::mobile_manipulator_mpc::ManipulatorModelInfo model_info_;
  std::vector<std::string> remove_joint_names_;
  std::shared_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorReferenceManager> reference_manager_;

  bool dual_arm_mode_{false};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_state_markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr optimized_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_right_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string global_frame_;

  std::unique_ptr<ocs2::GeometryInterfaceVisualization> geometry_visualization_;
  std::unique_ptr<EnvironmentCollisionVisualization> env_collision_visualization_;
};

} // namespace mpc_controller
