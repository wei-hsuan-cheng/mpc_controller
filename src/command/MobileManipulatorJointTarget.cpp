/**
 * MobileManipulatorJointTarget
 *
 * Publishes arm joint TargetTrajectories on "<robot>_joint_mpc_target".
 * The desired arm configuration is centered either at:
 *   1. an offset from the arm configuration captured at the first observation, or
 *   2. an absolute arm joint target,
 * and can optionally add a sinusoidal offset to each joint.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/ManipulatorModelInfo.h>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

using namespace ocs2;
using namespace ocs2::mobile_manipulator_mpc;

namespace {

struct Params {
  double publish_rate_hz = 20.0;
  double horizon_T = 2.0;
  double dt = 0.05;
  bool relative_to_current = true;
  bool sinusoid_enabled = true;
  double sinusoid_amplitude = 0.03;
  double sinusoid_frequency_hz = 0.10;
  double sinusoid_phase_increment = 0.35;
  std::vector<double> joint_offset;
  std::vector<double> joint_target;
};

vector_t stdVectorToEigen(const std::vector<double>& values) {
  vector_t out(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    out(static_cast<Eigen::Index>(i)) = values[i];
  }
  return out;
}

}  // namespace

class JointTargetNode : public rclcpp::Node {
 public:
  explicit JointTargetNode(const rclcpp::NodeOptions& options)
      : Node("mobile_manipulator_joint_target", options) {
    this->declare_parameter<std::string>("taskFile", "");
    this->declare_parameter<std::string>("urdfFile", "");
    this->declare_parameter<std::string>("libFolder", "");
    this->declare_parameter<std::string>("robotName", std::string("mobile_manipulator"));

    this->declare_parameter<double>("publishRate", params_.publish_rate_hz);
    this->declare_parameter<double>("horizon", params_.horizon_T);
    this->declare_parameter<double>("dt", params_.dt);
    this->declare_parameter<bool>("relativeToCurrent", params_.relative_to_current);
    this->declare_parameter<bool>("sinusoidEnabled", params_.sinusoid_enabled);
    this->declare_parameter<double>("sinusoidAmplitude", params_.sinusoid_amplitude);
    this->declare_parameter<double>("sinusoidFrequency", params_.sinusoid_frequency_hz);
    this->declare_parameter<double>("sinusoidPhaseIncrement", params_.sinusoid_phase_increment);
    this->declare_parameter<std::vector<double>>("jointOffset", params_.joint_offset);
    this->declare_parameter<std::vector<double>>("jointTarget", params_.joint_target);

    taskFile_ = this->get_parameter("taskFile").as_string();
    urdfFile_ = this->get_parameter("urdfFile").as_string();
    libFolder_ = this->get_parameter("libFolder").as_string();
    robotName_ = this->get_parameter("robotName").as_string();

    params_.publish_rate_hz = this->get_parameter("publishRate").as_double();
    params_.horizon_T = this->get_parameter("horizon").as_double();
    params_.dt = this->get_parameter("dt").as_double();
    params_.relative_to_current = this->get_parameter("relativeToCurrent").as_bool();
    params_.sinusoid_enabled = this->get_parameter("sinusoidEnabled").as_bool();
    params_.sinusoid_amplitude = this->get_parameter("sinusoidAmplitude").as_double();
    params_.sinusoid_frequency_hz = this->get_parameter("sinusoidFrequency").as_double();
    params_.sinusoid_phase_increment = this->get_parameter("sinusoidPhaseIncrement").as_double();
    params_.joint_offset = this->get_parameter("jointOffset").as_double_array();
    params_.joint_target = this->get_parameter("jointTarget").as_double_array();

    if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
      try {
        interface_ = std::make_unique<MobileManipulatorInterface>(taskFile_, libFolder_, urdfFile_);
        const auto& info = interface_->getManipulatorModelInfo();
        armDim_ = info.armDim;
        baseStateDim_ = info.stateDim >= info.armDim ? (info.stateDim - info.armDim) : 0;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
      }
    }

    jointTargetPub_ = this->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
        robotName_ + std::string("_joint_mpc_target"), 1);

    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + std::string("_mpc_observation"), 1,
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(obsMtx_);
          latestObs_ = ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, params_.publish_rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&JointTargetNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Joint target node started. Publishing to %s_joint_mpc_target at %.1f Hz",
                robotName_.c_str(), params_.publish_rate_hz);
  }

 private:
  bool initializeFromObservation(const SystemObservation& obs) {
    if (initialized_) {
      return true;
    }

    if (armDim_ == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Arm dimension is unknown. Provide taskFile/libFolder/urdfFile for joint target publishing.");
      return false;
    }

    const Eigen::Index state_offset = static_cast<Eigen::Index>(baseStateDim_);
    const Eigen::Index arm_dim = static_cast<Eigen::Index>(armDim_);
    if (obs.state.size() < state_offset + arm_dim) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Observation state dimension (%ld) is smaller than baseStateDim + armDim (%ld).",
                           static_cast<long>(obs.state.size()), static_cast<long>(state_offset + arm_dim));
      return false;
    }

    capturedArmState_ = obs.state.segment(state_offset, arm_dim);
    startTime_ = obs.time;

    if (!params_.joint_offset.empty() && params_.joint_offset.size() != armDim_) {
      RCLCPP_ERROR(this->get_logger(), "jointOffset size (%zu) must match armDim (%zu).",
                   params_.joint_offset.size(), armDim_);
      return false;
    }
    if (!params_.joint_target.empty() && params_.joint_target.size() != armDim_) {
      RCLCPP_ERROR(this->get_logger(), "jointTarget size (%zu) must match armDim (%zu).",
                   params_.joint_target.size(), armDim_);
      return false;
    }

    if (!params_.joint_offset.empty()) {
      jointOffset_ = stdVectorToEigen(params_.joint_offset);
    } else {
      jointOffset_ = vector_t::Zero(arm_dim);
    }

    if (!params_.joint_target.empty()) {
      absoluteJointTarget_ = stdVectorToEigen(params_.joint_target);
    } else {
      absoluteJointTarget_ = capturedArmState_;
    }

    initialized_ = true;

    const vector_t desired = desiredJointState();
    std::ostringstream desired_ss;
    desired_ss << desired.transpose();
    RCLCPP_INFO(this->get_logger(),
                "Initialized joint target with armDim=%zu, reference=%s, sinusoid=%d, amplitude=%.4f, frequency=%.4f Hz.",
                armDim_, desired_ss.str().c_str(), static_cast<int>(params_.sinusoid_enabled),
                params_.sinusoid_amplitude, params_.sinusoid_frequency_hz);
    return true;
  }

  vector_t desiredJointState() const {
    if (params_.relative_to_current) {
      return capturedArmState_ + jointOffset_;
    }
    return absoluteJointTarget_;
  }

  vector_t desiredJointStateAtTime(double t) const {
    vector_t desired = desiredJointState();
    if (!params_.sinusoid_enabled || std::abs(params_.sinusoid_amplitude) <= 1e-12 ||
        std::abs(params_.sinusoid_frequency_hz) <= 1e-12) {
      return desired;
    }

    const double omega = 2.0 * M_PI * params_.sinusoid_frequency_hz;
    const double tau = t - startTime_;
    for (Eigen::Index i = 0; i < desired.size(); ++i) {
      const double phase = static_cast<double>(i) * params_.sinusoid_phase_increment;
      desired(i) += params_.sinusoid_amplitude * std::sin(omega * tau + phase);
    }
    return desired;
  }

  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(obsMtx_);
      if (!haveObs_) {
        return;
      }
      obs = latestObs_;
    }

    if (!initializeFromObservation(obs)) {
      return;
    }

    const int N = std::max(1, static_cast<int>(std::ceil(params_.horizon_T / std::max(1e-6, params_.dt))));

    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    vector_array_t inputTraj;
    timeTraj.reserve(N + 1);
    stateTraj.reserve(N + 1);
    inputTraj.reserve(N + 1);

    const vector_t zeroInput = vector_t::Zero(0);
    for (int k = 0; k <= N; ++k) {
      const double tk = obs.time + k * params_.dt;
      timeTraj.push_back(tk);
      stateTraj.push_back(desiredJointStateAtTime(tk));
      inputTraj.push_back(zeroInput);
    }

    TargetTrajectories traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
    const auto msg = ros_msg_conversions::createTargetTrajectoriesMsg(traj);
    jointTargetPub_->publish(msg);
  }

  std::string taskFile_;
  std::string urdfFile_;
  std::string libFolder_;
  std::string robotName_;
  Params params_;

  std::unique_ptr<MobileManipulatorInterface> interface_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr jointTargetPub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex obsMtx_;
  SystemObservation latestObs_;
  bool haveObs_{false};

  bool initialized_{false};
  size_t armDim_{0};
  size_t baseStateDim_{0};
  double startTime_{0.0};
  vector_t capturedArmState_;
  vector_t jointOffset_;
  vector_t absoluteJointTarget_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<JointTargetNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
