#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h>

namespace mpc_controller {

class MobileManipulatorRosReferenceManager final : public ocs2::ReferenceManagerDecorator {
 public:
  MobileManipulatorRosReferenceManager(const rclcpp::Node::SharedPtr& node,
                                       std::shared_ptr<ocs2::ReferenceManagerInterface> referenceManager,
                                       std::string topicPrefix,
                                       const ocs2::PinocchioInterface& pinocchioInterface,
                                       const ocs2::mobile_manipulator::ManipulatorModelInfo& modelInfo,
                                       const rclcpp::QoS& qos = rclcpp::QoS(10));

  void setHoldTargetsFromObservation(const ocs2::SystemObservation& obs,
                                     const ocs2::TargetTrajectories& eeHold);

 private:
  void modeScheduleCallback(const ocs2_msgs::msg::ModeSchedule::SharedPtr msg);
  void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg);

  void eeTargetCallback(const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg);
  void jointTargetCallback(const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg);
  void baseTargetCallback(const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg);

  ocs2::TargetTrajectories makeEeHoldFromObservation(const ocs2::SystemObservation& obs);

  bool hasRecentEeTarget() const;
  bool hasRecentJointTarget() const;

  rclcpp::Node::SharedPtr node_;
  const std::string topicPrefix_;

  // Pinocchio (private copy)
  ocs2::PinocchioInterface pinocchioInterface_;
  ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping pinocchioMapping_;
  std::size_t eeFrameId_{0};

  // Latest observation cache
  mutable std::mutex obsMutex_;
  ocs2::SystemObservation latestObs_;
  bool hasObs_{false};

  // Target freshness tracking
  rclcpp::Time lastEeTargetStamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastJointTargetStamp_{0, 0, RCL_ROS_TIME};
  double targetTimeoutSec_{0.3};  // if no update > timeout => treat as "no target"

  // Mode change detection
  int lastMode_{-1};

  // Convention (your task.info):
  //  mode 0: joint-only
  //  mode 1: EE-only
  int jointOnlyMode_{0};
  int eeOnlyMode_{1};

  // Subs
  rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr modeScheduleSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr eeTargetSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr jointTargetSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr baseTargetSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr legacyEeTargetSub_;
};

}  // namespace mpc_controller
