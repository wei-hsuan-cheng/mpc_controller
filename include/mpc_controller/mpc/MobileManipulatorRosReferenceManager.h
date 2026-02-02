#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <mobile_manipulator_mpc/ManipulatorModelInfo.h>
#include <mobile_manipulator_mpc/MobileManipulatorPinocchioMapping.h>

namespace mpc_controller {

class MobileManipulatorRosReferenceManager final : public ocs2::ReferenceManagerDecorator {
 public:
  MobileManipulatorRosReferenceManager(
      const rclcpp::Node::SharedPtr& node,
      std::shared_ptr<ocs2::ReferenceManagerInterface> referenceManager,
      std::string topicPrefix,
      const ocs2::PinocchioInterface& pinocchioInterface,
      const ocs2::mobile_manipulator_mpc::ManipulatorModelInfo& modelInfo,
      const rclcpp::QoS& qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  /** Inject the latest observation from the MPC node (observation subscriber lives in MPC_ROS_Interface). */
  void setLatestObservation(const ocs2::SystemObservation& obs);

 private:
  void modeScheduleCallback(const ocs2_msgs::msg::ModeSchedule::SharedPtr msg);

  void eeTargetCallback(const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg);
  void jointTargetCallback(const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg);
  void baseTargetCallback(const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg);

  bool hasRecentEeTarget() const;
  bool hasRecentJointTarget() const;

  ocs2::TargetTrajectories makeEeHoldFromObservation(const ocs2::SystemObservation& obs);

  void setHoldTargetsFromObservation(const ocs2::SystemObservation& obs, const ocs2::TargetTrajectories& eeHold);

 private:
  rclcpp::Node::SharedPtr node_;
  std::string topicPrefix_;

  // Pinocchio for FK
  ocs2::PinocchioInterface pinocchioInterface_;  // local copy
  ocs2::mobile_manipulator_mpc::ManipulatorModelInfo modelInfo_;
  ocs2::mobile_manipulator_mpc::MobileManipulatorPinocchioMapping pinocchioMapping_;
  std::size_t eeFrameId_{0};

  // Subs
  rclcpp::Subscription<ocs2_msgs::msg::ModeSchedule>::SharedPtr modeScheduleSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr eeTargetSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr jointTargetSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr baseTargetSub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr legacyEeTargetSub_;

  // Latest observation cache (injected)
  mutable std::mutex obsMutex_;
  ocs2::SystemObservation latestObs_;
  std::atomic_bool hasObs_{false};

  // Target freshness
  double targetTimeoutSec_{0.5};
  rclcpp::Time lastEeTargetStamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastJointTargetStamp_{0, 0, RCL_ROS_TIME};

  // Mode id
  int jointOnlyMode_{0};
  int eeOnlyMode_{1};
  int lastMode_{-1};
};

}  // namespace mpc_controller
