#include "mpc_controller/mpc/MobileManipulatorRosReferenceManager.h"

#include <utility>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_mobile_manipulator/reference/MobileManipulatorReferenceManager.h>

// pinocchio FK
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace mpc_controller {

namespace {

// State layout: baseDim / qArmOffset / armDim
inline void computeStateLayout(const ocs2::mobile_manipulator::ManipulatorModelInfo& info,
                               int& baseDim, int& qArmOffset, int& armDim) {
  armDim = static_cast<int>(info.armDim);

  using MT = ocs2::mobile_manipulator::ManipulatorModelType;
  switch (info.manipulatorModelType) {
    case MT::DefaultManipulator:
      baseDim = 0;
      qArmOffset = 0;
      break;
    case MT::WheelBasedMobileManipulator:
      baseDim = 3;   // x,y,yaw
      qArmOffset = 3;
      break;
    case MT::FloatingArmManipulator:
    case MT::FullyActuatedFloatingArmManipulator:
      baseDim = 6;   // (pos+ori) or (xyz+rpy) depends on mapping, but always 6-dim
      qArmOffset = 6;
      break;
    default:
      // conservative fallback
      baseDim = 0;
      qArmOffset = 0;
      break;
  }
}

inline bool isValidSegment(int start, int length, int total) {
  return (start >= 0) && (length >= 0) && (start + length <= total);
}

}  // namespace

MobileManipulatorRosReferenceManager::MobileManipulatorRosReferenceManager(
    const rclcpp::Node::SharedPtr& node,
    std::shared_ptr<ocs2::ReferenceManagerInterface> referenceManager,
    std::string topicPrefix,
    const ocs2::PinocchioInterface& pinocchioInterface,
    const ocs2::mobile_manipulator::ManipulatorModelInfo& modelInfo,
    const rclcpp::QoS& qos)
    : ocs2::ReferenceManagerDecorator(std::move(referenceManager)),
      node_(node),
      topicPrefix_(std::move(topicPrefix)),
      pinocchioInterface_(pinocchioInterface),  // copy
      modelInfo_(modelInfo),
      pinocchioMapping_(modelInfo) {

  // Resolve EE frame id once
  eeFrameId_ = pinocchioInterface_.getModel().getFrameId(modelInfo_.eeFrame);

  // Optional param
  if (node_->has_parameter("target_timeout_sec")) {
    targetTimeoutSec_ = node_->get_parameter("target_timeout_sec").as_double();
  } else {
    node_->declare_parameter<double>("target_timeout_sec", targetTimeoutSec_);
    targetTimeoutSec_ = node_->get_parameter("target_timeout_sec").as_double();
  }

  // ---- Subscriptions ----
  modeScheduleSub_ = node_->create_subscription<ocs2_msgs::msg::ModeSchedule>(
      topicPrefix_ + "_mode_schedule", qos,
      std::bind(&MobileManipulatorRosReferenceManager::modeScheduleCallback, this, std::placeholders::_1));

  observationSub_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
      topicPrefix_ + "_mpc_observation", rclcpp::QoS(1).reliable(),
      std::bind(&MobileManipulatorRosReferenceManager::observationCallback, this, std::placeholders::_1));

  eeTargetSub_ = node_->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
      topicPrefix_ + "_mpc_ee_target", qos,
      std::bind(&MobileManipulatorRosReferenceManager::eeTargetCallback, this, std::placeholders::_1));

  jointTargetSub_ = node_->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
      topicPrefix_ + "_mpc_joint_target", qos,
      std::bind(&MobileManipulatorRosReferenceManager::jointTargetCallback, this, std::placeholders::_1));

  baseTargetSub_ = node_->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
      topicPrefix_ + "_mpc_base_target", qos,
      std::bind(&MobileManipulatorRosReferenceManager::baseTargetCallback, this, std::placeholders::_1));

  // Legacy topic (some nodes publish here)
  legacyEeTargetSub_ = node_->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
      topicPrefix_ + "_mpc_target", qos,
      std::bind(&MobileManipulatorRosReferenceManager::eeTargetCallback, this, std::placeholders::_1));
}

void MobileManipulatorRosReferenceManager::observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
  const auto obs = ocs2::ros_msg_conversions::readObservationMsg(*msg);
  std::lock_guard<std::mutex> lock(obsMutex_);
  latestObs_ = obs;
  hasObs_.store(true, std::memory_order_release);
}

bool MobileManipulatorRosReferenceManager::hasRecentEeTarget() const {
  const auto now = node_->now();
  if (lastEeTargetStamp_.nanoseconds() == 0) return false;
  return (now - lastEeTargetStamp_).seconds() <= targetTimeoutSec_;
}

bool MobileManipulatorRosReferenceManager::hasRecentJointTarget() const {
  const auto now = node_->now();
  if (lastJointTargetStamp_.nanoseconds() == 0) return false;
  return (now - lastJointTargetStamp_).seconds() <= targetTimeoutSec_;
}

void MobileManipulatorRosReferenceManager::modeScheduleCallback(const ocs2_msgs::msg::ModeSchedule::SharedPtr msg) {
  // Always forward mode schedule to the underlying reference manager
  auto modeSchedule = ocs2::ros_msg_conversions::readModeScheduleMsg(*msg);
  referenceManagerPtr_->setModeSchedule(std::move(modeSchedule));

  if (!hasObs_.load(std::memory_order_acquire)) return;
  if (msg->mode_sequence.empty()) return;

  // Current mode is the first one (for event_times empty: constant mode)
  const int mode = static_cast<int>(msg->mode_sequence.front());
  if (mode == lastMode_) return;
  lastMode_ = mode;

  // Snapshot latest observation
  ocs2::SystemObservation obs;
  {
    std::lock_guard<std::mutex> lock(obsMutex_);
    obs = latestObs_;
  }

  auto* mm =
      dynamic_cast<ocs2::mobile_manipulator::MobileManipulatorReferenceManager*>(referenceManagerPtr_.get());
  if (mm == nullptr) return;

  // ================================
  // (A) mode 0: joint-only
  // If no joint target recently => hold joint (and base)
  // This prevents "returning to task.info jointTracking.reference"
  // ================================
  if (mode == jointOnlyMode_) {
    if (!hasRecentJointTarget()) {
      // build a minimal eeHold just to feed setHoldTargetsFromObservation (EE alpha is 0 in joint-only anyway)
      const auto eeHold = makeEeHoldFromObservation(obs);
      setHoldTargetsFromObservation(obs, eeHold);
    }
    return;
  }

  // ================================
  // (B) mode 1: EE-only
  // If no EE target recently => auto EE-hold (plus base/joint hold as safety)
  // ================================
  if (mode == eeOnlyMode_) {
    if (!hasRecentEeTarget()) {
      const auto eeHold = makeEeHoldFromObservation(obs);
      setHoldTargetsFromObservation(obs, eeHold);
    }
    return;
  }
}

ocs2::TargetTrajectories MobileManipulatorRosReferenceManager::makeEeHoldFromObservation(
    const ocs2::SystemObservation& obs) {

  // Map OCS2 state -> pinocchio generalized coordinates
  const auto qPinRaw = pinocchioMapping_.getPinocchioJointPosition(obs.state);

  auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  // IMPORTANT:
  // Use pinocchio::neutral(model) then assign, ensures correct type/size for Pinocchio overloads.
  pinocchio::Model::ConfigVectorType q = pinocchio::neutral(model);
  q = qPinRaw;

  // pinocchio updates data
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  const auto& oMf = data.oMf[eeFrameId_];
  const Eigen::Vector3d p = oMf.translation();
  const Eigen::Quaterniond quat(oMf.rotation());  // coeffs(): (x,y,z,w)

  ocs2::vector_t x(7);
  x.head<3>() = p;
  x.tail<4>() = quat.coeffs();

  ocs2::TargetTrajectories eeHold;
  eeHold.timeTrajectory = {obs.time};
  eeHold.stateTrajectory = {x};
  eeHold.inputTrajectory = {};
  return eeHold;
}

void MobileManipulatorRosReferenceManager::eeTargetCallback(
    const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg) {
  auto target = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
  if (target.timeTrajectory.empty() || target.stateTrajectory.empty()) {
    return;
  }

  lastEeTargetStamp_ = node_->now();

  auto* mm =
      dynamic_cast<ocs2::mobile_manipulator::MobileManipulatorReferenceManager*>(referenceManagerPtr_.get());
  if (mm != nullptr) {
    mm->setEeTargetTrajectories(std::move(target));
  }
}

void MobileManipulatorRosReferenceManager::jointTargetCallback(
    const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg) {
  auto target = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
  if (target.timeTrajectory.empty() || target.stateTrajectory.empty()) {
    return;
  }

  lastJointTargetStamp_ = node_->now();

  auto* mm =
      dynamic_cast<ocs2::mobile_manipulator::MobileManipulatorReferenceManager*>(referenceManagerPtr_.get());
  if (mm != nullptr) {
    mm->setJointTargetTrajectories(std::move(target));
  }
}

void MobileManipulatorRosReferenceManager::baseTargetCallback(
    const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg) {
  auto target = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
  if (target.timeTrajectory.empty() || target.stateTrajectory.empty()) {
    return;
  }

  auto* mm =
      dynamic_cast<ocs2::mobile_manipulator::MobileManipulatorReferenceManager*>(referenceManagerPtr_.get());
  if (mm != nullptr) {
    mm->setBaseTargetTrajectories(std::move(target));
  }
}

void MobileManipulatorRosReferenceManager::setHoldTargetsFromObservation(
    const ocs2::SystemObservation& obs,
    const ocs2::TargetTrajectories& eeHold) {

  auto* mm =
      dynamic_cast<ocs2::mobile_manipulator::MobileManipulatorReferenceManager*>(referenceManagerPtr_.get());
  if (mm == nullptr) return;

  // 1) EE hold
  mm->setEeTargetTrajectories(eeHold);

  // state layout: baseDim / qArmOffset / armDim
  int baseDim = 0;
  int qArmOffset = 0;
  int armDim = 0;
  computeStateLayout(modelInfo_, baseDim, qArmOffset, armDim);

  const int stateDim = static_cast<int>(obs.state.size());

  // 2) Base hold (only meaningful if baseDim > 0; skip for fixed-base)
  if (baseDim > 0) {
    if (!isValidSegment(0, baseDim, stateDim)) {
      RCLCPP_WARN(node_->get_logger(),
                  "[RosRefManager][hold] base segment invalid: baseDim=%d stateDim=%d (modelType=%d). Skip base hold.",
                  baseDim, stateDim, static_cast<int>(modelInfo_.manipulatorModelType));
    } else {
      ocs2::TargetTrajectories baseHold;
      baseHold.timeTrajectory = {obs.time};
      baseHold.stateTrajectory = {obs.state.head(baseDim)};
      baseHold.inputTrajectory = {};
      mm->setBaseTargetTrajectories(std::move(baseHold));
    }
  }

  // 3) Joint hold (armDim is from modelInfo.armDim; offset depends on modelType)
  if (armDim > 0) {
    if (!isValidSegment(qArmOffset, armDim, stateDim)) {
      RCLCPP_WARN(node_->get_logger(),
                  "[RosRefManager][hold] arm segment invalid: qArmOffset=%d armDim=%d stateDim=%d (modelType=%d). "
                  "Fallback: use tail(armDim) if possible.",
                  qArmOffset, armDim, stateDim, static_cast<int>(modelInfo_.manipulatorModelType));

      if (armDim <= stateDim) {
        ocs2::TargetTrajectories jointHold;
        jointHold.timeTrajectory = {obs.time};
        jointHold.stateTrajectory = {obs.state.tail(armDim)};
        jointHold.inputTrajectory = {};
        mm->setJointTargetTrajectories(std::move(jointHold));
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[RosRefManager][hold] armDim > stateDim (armDim=%d stateDim=%d). Skip joint hold.", armDim,
                    stateDim);
      }
    } else {
      ocs2::TargetTrajectories jointHold;
      jointHold.timeTrajectory = {obs.time};
      jointHold.stateTrajectory = {obs.state.segment(qArmOffset, armDim)};
      jointHold.inputTrajectory = {};
      mm->setJointTargetTrajectories(std::move(jointHold));
    }
  }
}

}  // namespace mpc_controller
