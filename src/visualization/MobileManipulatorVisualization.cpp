#include "mpc_controller/visualization/MobileManipulatorVisualization.h"

#include <Eigen/Geometry>

#include <exception>

#include <algorithm>
#include <array>
#include <utility>

#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <mobile_manipulator_mpc/AccessHelperFunctions.h>
#include <mobile_manipulator_mpc/EnvironmentCollisionConfig.h>
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
constexpr std::array<double, 3> kEeTrajectoryColorLeft{0.0, 0.4470, 0.7410};
constexpr std::array<double, 3> kEeTrajectoryColorRight{0.4940, 0.1840, 0.5560};

bool readDualArmModeFromTaskFile(const boost::property_tree::ptree& pt) {
  bool dualArmMode = false;
  try {
    ocs2::loadData::loadPtreeValue(pt, dualArmMode, "endEffector.dualArmMode", false);
    if (!dualArmMode) {
      ocs2::loadData::loadPtreeValue(pt, dualArmMode, "endEffectorTracking.dualArmMode", false);
    }
    if (!dualArmMode) {
      ocs2::loadData::loadPtreeValue(pt, dualArmMode, "finalEndEffector.dualArmMode", false);
    }
    if (!dualArmMode) {
      ocs2::loadData::loadPtreeValue(pt, dualArmMode, "finalEndEffectorTracking.dualArmMode", false);
    }
  } catch (...) {
    dualArmMode = false;
  }
  return dualArmMode;
}

Eigen::Quaterniond quatFromCoeffs(const Eigen::Vector4d& coeffs) {
  Eigen::Quaterniond q;
  q.coeffs() = coeffs;
  return q.normalized();
}

} // namespace

MobileManipulatorVisualization::MobileManipulatorVisualization(const rclcpp::Node::SharedPtr &node,
                                                               const MobileManipulatorInterface &interface,
                                                               const std::string &task_file,
                                                               const std::string &urdf_file,
                                                               const std::string &global_frame)
    : node_(node),
      pinocchio_interface_(interface.getPinocchioInterface()),
      model_info_(interface.getManipulatorModelInfo()),
      reference_manager_(std::dynamic_pointer_cast<ocs2::mobile_manipulator_mpc::MobileManipulatorReferenceManager>(
          interface.getReferenceManagerPtr())),
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

  dual_arm_mode_ = readDualArmModeFromTaskFile(pt);
  if (dual_arm_mode_) {
    target_pose_right_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mobile_manipulator/targetPoseRight", 1);
  }

  const auto model_type = ocs2::mobile_manipulator_mpc::loadManipulatorType(task_file, "model_information.manipulatorModelType");
  ocs2::loadData::loadStdVector<std::string>(task_file, "model_information.removeJoints", remove_joint_names_, false);

  bool activate_self_collision = true;
  ocs2::loadData::loadPtreeValue(pt, activate_self_collision, "selfCollision.activate", true);

  if (activate_self_collision) {
    auto viz_pinocchio = createPinocchioInterface(urdf_file, model_type, remove_joint_names_);

    std::vector<std::pair<size_t, size_t>> collision_object_pairs;
    std::vector<std::pair<std::string, std::string>> collision_link_pairs;
    ocs2::loadData::loadStdVectorOfPair(task_file, "selfCollision.collisionObjectPairs", collision_object_pairs, true);
    ocs2::loadData::loadStdVectorOfPair(task_file, "selfCollision.collisionLinkPairs", collision_link_pairs, true);

    ocs2::PinocchioGeometryInterface geom_interface(viz_pinocchio, collision_link_pairs, collision_object_pairs);
    geometry_visualization_ = std::make_unique<GeometryInterfaceVisualization>(
        std::move(viz_pinocchio), std::move(geom_interface), global_frame_);
  }

  const auto env_collision_config =
      ocs2::mobile_manipulator_mpc::loadEnvironmentCollisionConfig(task_file, "envCollision", pinocchio_interface_.getModel());
  if (env_collision_config.activate) {
    env_collision_visualization_ = std::make_unique<EnvironmentCollisionVisualization>(
        node_, pinocchio_interface_, reference_manager_, env_collision_config, global_frame_);
  }
}

void MobileManipulatorVisualization::publishTargetTrajectories(const rclcpp::Time &time_stamp,
                                                               const TargetTrajectories &target) {
  if (target.stateTrajectory.empty()) {
    return;
  }

  const auto &desired_state = target.stateTrajectory.back();

  // Single-arm target: [pos(3), quat(4)]
  if (!dual_arm_mode_ || desired_state.size() < 14) {
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header = ocs2::ros_msg_helpers::getHeaderMsg(global_frame_, time_stamp);
    target_pose.pose.position = ocs2::ros_msg_helpers::getPointMsg(desired_state.head<3>());
    target_pose.pose.orientation =
        ocs2::ros_msg_helpers::getOrientationMsg(quatFromCoeffs(desired_state.tail<4>()));

    if (target_pose_pub_) {
      target_pose_pub_->publish(target_pose);
    }
    return;
  }

  // Dual-arm target: [left pos(3), left quat(4), right pos(3), right quat(4)]
  geometry_msgs::msg::PoseStamped left_pose;
  left_pose.header = ocs2::ros_msg_helpers::getHeaderMsg(global_frame_, time_stamp);
  left_pose.pose.position = ocs2::ros_msg_helpers::getPointMsg(desired_state.segment<3>(0));
  left_pose.pose.orientation =
      ocs2::ros_msg_helpers::getOrientationMsg(quatFromCoeffs(desired_state.segment<4>(3)));

  geometry_msgs::msg::PoseStamped right_pose;
  right_pose.header = left_pose.header;
  right_pose.pose.position = ocs2::ros_msg_helpers::getPointMsg(desired_state.segment<3>(7));
  right_pose.pose.orientation =
      ocs2::ros_msg_helpers::getOrientationMsg(quatFromCoeffs(desired_state.segment<4>(10)));

  if (target_pose_pub_) {
    target_pose_pub_->publish(left_pose);
  }
  if (target_pose_right_pub_) {
    target_pose_right_pub_->publish(right_pose);
  }
}

void MobileManipulatorVisualization::publishOptimizedTrajectory(const rclcpp::Time &time_stamp, const PrimalSolution &policy) {
  if (policy.stateTrajectory_.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses.reserve(policy.stateTrajectory_.size() * (dual_arm_mode_ ? 3 : 2));

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

  const auto &model = pinocchio_interface_.getModel();
  auto &data = pinocchio_interface_.getData();

  const auto ee_left_id = model.getFrameId(model_info_.eeFrame);
  pinocchio::FrameIndex ee_right_id = 0;
  bool has_right_ee = dual_arm_mode_ && !model_info_.eeFrame1.empty();
  if (has_right_ee) {
    try {
      ee_right_id = model.getFrameId(model_info_.eeFrame1);
    } catch (const std::exception&) {
      has_right_ee = false;
    }
  }

  std::vector<geometry_msgs::msg::Point> ee_left_trajectory;
  ee_left_trajectory.reserve(policy.stateTrajectory_.size());
  std::vector<geometry_msgs::msg::Point> ee_right_trajectory;
  if (has_right_ee) {
    ee_right_trajectory.reserve(policy.stateTrajectory_.size());
  }

  for (const auto &state : policy.stateTrajectory_) {
    pinocchio::forwardKinematics(model, data, state);
    pinocchio::updateFramePlacements(model, data);

    const auto &tf_left = data.oMf[ee_left_id];
    geometry_msgs::msg::Pose left_pose;
    left_pose.position = ocs2::ros_msg_helpers::getPointMsg(tf_left.translation());
    left_pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond(tf_left.rotation()));
    ee_left_trajectory.push_back(left_pose.position);
    pose_array.poses.push_back(left_pose);

    if (has_right_ee) {
      const auto &tf_right = data.oMf[ee_right_id];
      geometry_msgs::msg::Pose right_pose;
      right_pose.position = ocs2::ros_msg_helpers::getPointMsg(tf_right.translation());
      right_pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond(tf_right.rotation()));
      ee_right_trajectory.push_back(right_pose.position);
      pose_array.poses.push_back(right_pose);
    }
  }

  marker_array.markers.emplace_back(
      ocs2::ros_msg_helpers::getLineMsg(std::move(base_trajectory), kBaseTrajectoryColor, kTrajectoryLineWidth));
  marker_array.markers.back().ns = "Base Trajectory";

  marker_array.markers.emplace_back(
      ocs2::ros_msg_helpers::getLineMsg(std::move(ee_left_trajectory), kEeTrajectoryColorLeft, kTrajectoryLineWidth));
  marker_array.markers.back().ns = "EE Trajectory";

  if (has_right_ee) {
    marker_array.markers.emplace_back(
        ocs2::ros_msg_helpers::getLineMsg(std::move(ee_right_trajectory), kEeTrajectoryColorRight, kTrajectoryLineWidth));
    marker_array.markers.back().ns = "EE1 Trajectory";
  }

  pose_array.header = ocs2::ros_msg_helpers::getHeaderMsg(global_frame_, time_stamp);
  assignHeader(marker_array.markers.begin(), marker_array.markers.end(), pose_array.header);
  assignIncreasingId(marker_array.markers.begin(), marker_array.markers.end());

  if (optimized_state_markers_pub_) {
    optimized_state_markers_pub_->publish(marker_array);
  }
  if (optimized_pose_pub_) {
    optimized_pose_pub_->publish(pose_array);
  }
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
  if (env_collision_visualization_) {
    env_collision_visualization_->publish(current_state, stamp);
  }
}

} // namespace mpc_controller
