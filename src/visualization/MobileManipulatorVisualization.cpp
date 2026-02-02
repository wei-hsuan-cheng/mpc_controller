#include "mpc_controller/visualization/MobileManipulatorVisualization.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <utility>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <mobile_manipulator_mpc/AccessHelperFunctions.h>
#include <mobile_manipulator_mpc/FactoryFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace mpc_controller {

using ocs2::CommandData;
using ocs2::GeometryInterfaceVisualization;
using ocs2::PrimalSolution;
using ocs2::TargetTrajectories;
using ocs2::mobile_manipulator_mpc::ManipulatorModelInfo;
using ocs2::mobile_manipulator_mpc::MobileManipulatorInterface;
using ocs2::mobile_manipulator_mpc::createPinocchioInterface;
using ocs2::mobile_manipulator_mpc::getArmJointAngles;
using ocs2::mobile_manipulator_mpc::getBaseOrientation;
using ocs2::mobile_manipulator_mpc::getBasePosition;

namespace {
constexpr double kTrajectoryLineWidth = 0.005;
constexpr std::array<double, 3> kBaseTrajectoryColor{0.6350, 0.0780, 0.1840};
constexpr std::array<double, 3> kEeTrajectoryColor{0.0, 0.4470, 0.7410};
} // namespace

MobileManipulatorVisualization::MobileManipulatorVisualization(const rclcpp::Node::SharedPtr &node,
                                                               const MobileManipulatorInterface &interface,
                                                               const std::string &task_file,
                                                               const std::string &urdf_file,
                                                               const std::string &global_frame)
    : node_(node),
      pinocchio_interface_(interface.getPinocchioInterface()),
      model_info_(interface.getManipulatorModelInfo()),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(node)),
      global_frame_(global_frame) {
  launchVisualizerNode(task_file, urdf_file);
}

void MobileManipulatorVisualization::launchVisualizerNode(const std::string &task_file, const std::string &urdf_file) {
  optimized_state_markers_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("/mobile_manipulator/optimizedStateTrajectory", 1);
  optimized_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/mobile_manipulator/optimizedPoseTrajectory", 1);
  target_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mobile_manipulator/targetPose", 1);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(task_file, pt);

  const auto model_type = ocs2::mobile_manipulator_mpc::loadManipulatorType(task_file, "model_information.manipulatorModelType");
  ocs2::loadData::loadStdVector<std::string>(task_file, "model_information.removeJoints", remove_joint_names_, false);

  bool activate_self_collision = true;
  ocs2::loadData::loadPtreeValue(pt, activate_self_collision, "selfCollision.activate", true);

  if (activate_self_collision) {
    auto viz_pinocchio = createPinocchioInterface(urdf_file, model_type, remove_joint_names_);
    std::vector<std::pair<size_t, size_t>> collision_pairs;
    ocs2::loadData::loadStdVectorOfPair(task_file, "selfCollision.collisionObjectPairs", collision_pairs, true);
    ocs2::PinocchioGeometryInterface geom_interface(viz_pinocchio, collision_pairs);
    geometry_visualization_ = std::make_unique<GeometryInterfaceVisualization>(
        std::move(viz_pinocchio), std::move(geom_interface), global_frame_);
  }
}

void MobileManipulatorVisualization::publishTargetTrajectories(const rclcpp::Time &time_stamp,
                                                               const TargetTrajectories &target) {
  if (target.stateTrajectory.empty()) {
    return;
  }
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header = ocs2::ros_msg_helpers::getHeaderMsg(global_frame_, time_stamp);
  const auto &desired_state = target.stateTrajectory.back();
  target_pose.pose.position = ocs2::ros_msg_helpers::getPointMsg(desired_state.head<3>());
  Eigen::Quaterniond desired_orientation;
  desired_orientation.coeffs() = desired_state.tail<4>();
  target_pose.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(desired_orientation);
  if (target_pose_pub_) {
    target_pose_pub_->publish(target_pose);
  }

  if (tf_broadcaster_) {
    // geometry_msgs::msg::TransformStamped target_tf;
    // target_tf.header = target_pose.header;
    // target_tf.child_frame_id = "command";
    // target_tf.transform.translation.x = target_pose.pose.position.x;
    // target_tf.transform.translation.y = target_pose.pose.position.y;
    // target_tf.transform.translation.z = target_pose.pose.position.z;
    // target_tf.transform.rotation = target_pose.pose.orientation;
    // tf_broadcaster_->sendTransform(target_tf);
  }
}

void MobileManipulatorVisualization::publishOptimizedTrajectory(const rclcpp::Time &time_stamp, const PrimalSolution &policy) {
  if (policy.stateTrajectory_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses.reserve(policy.stateTrajectory_.size());

  std::vector<geometry_msgs::msg::Point> base_trajectory;
  base_trajectory.reserve(policy.stateTrajectory_.size());

  for (const auto &state : policy.stateTrajectory_) {
    const auto base_position = getBasePosition(state, model_info_);
    const auto base_orientation = getBaseOrientation(state, model_info_);
    geometry_msgs::msg::Pose pose;
    pose.position = ocs2::ros_msg_helpers::getPointMsg(base_position);
    pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(base_orientation);
    base_trajectory.push_back(pose.position);
    pose_array.poses.push_back(pose);
  }

  std::vector<geometry_msgs::msg::Point> ee_trajectory;
  ee_trajectory.reserve(policy.stateTrajectory_.size());

  const auto &model = pinocchio_interface_.getModel();
  auto &data = pinocchio_interface_.getData();
  const auto ee_id = model.getBodyId(model_info_.eeFrame);

  for (const auto &state : policy.stateTrajectory_) {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);
    const auto &transform = data.oMf[ee_id];
    geometry_msgs::msg::Pose ee_pose;
    ee_pose.position = ocs2::ros_msg_helpers::getPointMsg(transform.translation());
    Eigen::Quaterniond quat(transform.rotation());
    ee_pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(quat);
    ee_trajectory.push_back(ee_pose.position);
    pose_array.poses.push_back(ee_pose);
  }

  marker_array.markers.emplace_back(
      ocs2::ros_msg_helpers::getLineMsg(std::move(base_trajectory), kBaseTrajectoryColor, kTrajectoryLineWidth));
  marker_array.markers.back().ns = "Base Trajectory";

  marker_array.markers.emplace_back(
      ocs2::ros_msg_helpers::getLineMsg(std::move(ee_trajectory), kEeTrajectoryColor, kTrajectoryLineWidth));
  marker_array.markers.back().ns = "EE Trajectory";

  pose_array.header = ocs2::ros_msg_helpers::getHeaderMsg(global_frame_, time_stamp);
  assignHeader(marker_array.markers.begin(), marker_array.markers.end(), pose_array.header);
  assignIncreasingId(marker_array.markers.begin(), marker_array.markers.end());

  optimized_state_markers_pub_->publish(marker_array);
  optimized_pose_pub_->publish(pose_array);
}

void MobileManipulatorVisualization::update(const ocs2::vector_t &current_state,
                                            const PrimalSolution &policy,
                                            const CommandData &command) {
  const auto stamp = node_->get_clock()->now();

  publishTargetTrajectories(stamp, command.mpcTargetTrajectories_);
  publishOptimizedTrajectory(stamp, policy);

  if (geometry_visualization_) {
    geometry_visualization_->publishDistances(current_state);
  }
}

} // namespace mpc_controller
