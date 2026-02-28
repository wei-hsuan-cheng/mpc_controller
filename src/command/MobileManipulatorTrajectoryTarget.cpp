/**
 * MobileManipulatorTrajectoryTarget
 *
 * Publishes a time-parameterized TargetTrajectories following an "eight" shape
 * in XY, centered at the initial end-effector position. Orientation remains
 * constant (initial FK orientation). Uses the latest MPC observation time as
 * anchor and advances the phase relative to the first observation time.
 *
 * In addition to the end-effector target, this node also publishes an
 * independent base target on "<robot>_base_mpc_target" using the same time
 * horizon and a sinusoidal back-and-forth line segment. For wheel-based
 * manipulators the base state target is [x, y, yaw].
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/ManipulatorModelInfo.h>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace ocs2;
using namespace ocs2::mobile_manipulator_mpc;

namespace {
struct Params {
  double publish_rate_hz = 20.0;
  double horizon_T = 2.0;
  double dt = 0.05;
  double amplitude = 0.2;
  double frequency_hz = 0.1;
  double base_amplitude = 0.2;
  double base_frequency_hz = 0.1;
  double base_yaw = 0.0;
  double base_slope = 0.0;
};

inline void orthonormalBasisFromAxis(const Eigen::Vector3d& axis_unit, Eigen::Vector3d& u, Eigen::Vector3d& v) {
  Eigen::Vector3d ref = (std::abs(axis_unit.z()) < 0.9) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
  u = axis_unit.cross(ref);
  double n = u.norm();
  if (n < 1e-9) {
    ref = Eigen::Vector3d::UnitY();
    u = axis_unit.cross(ref);
    n = u.norm();
  }
  u /= n;
  v = axis_unit.cross(u);
}

inline void eightShapeTrajectory(const Eigen::Vector3d& center_p, const Eigen::Quaterniond& q0,
                                 const Eigen::Vector3d& plane_axis_unit, double start_time, double t0, int N,
                                 double dt, double amp, double freq_hz, scalar_array_t& timeTraj,
                                 vector_array_t& stateTraj) {
  const double omega = 2.0 * M_PI * freq_hz;
  Eigen::Vector3d u, v;
  orthonormalBasisFromAxis(plane_axis_unit, u, v);
  timeTraj.reserve(N + 1);
  stateTraj.reserve(N + 1);
  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * dt;
    const double tau = tk - start_time;
    const double x = amp * std::sin(omega * tau);
    const double y = 0.5 * amp * std::sin(2.0 * omega * tau);
    Eigen::Vector3d p = center_p + u * x + v * y;

    vector_t target(7);
    target << p, Eigen::Vector4d(q0.x(), q0.y(), q0.z(), q0.w());

    timeTraj.push_back(tk);
    stateTraj.push_back(std::move(target));
  }
}

inline void sinusoidalLineBaseTrajectory(const Eigen::Vector3d& center_p, double yaw, double slope,
                                       size_t base_pose_dim, double start_time, double t0, int N, double dt,
                                       double amp, double freq_hz, scalar_array_t& timeTraj,
                                       vector_array_t& stateTraj) {
  if (base_pose_dim != 3 && base_pose_dim != 6) {
    return;
  }

  const double omega = 2.0 * M_PI * freq_hz;
  Eigen::Vector2d direction(1.0, slope);
  if (direction.norm() < 1e-9) {
    direction = Eigen::Vector2d::UnitX();
  }
  direction.normalize();

  timeTraj.reserve(N + 1);
  stateTraj.reserve(N + 1);

  for (int k = 0; k <= N; ++k) {
    const double tk = t0 + k * dt;
    const double tau = tk - start_time;
    const double displacement = amp * std::sin(omega * tau);
    const double x = center_p.x() + displacement * direction.x();
    const double y = center_p.y() + displacement * direction.y();

    vector_t target(base_pose_dim);
    if (base_pose_dim == 3) {
      target << x, y, yaw;
    } else {
      target << x, y, center_p.z(), yaw, 0.0, 0.0;
    }

    timeTraj.push_back(tk);
    stateTraj.push_back(std::move(target));
  }
}
}  // namespace

class TrajectoryTargetNode : public rclcpp::Node {
 public:
  explicit TrajectoryTargetNode(const rclcpp::NodeOptions& options)
      : Node("mobile_manipulator_trajectory_target", options) {
    this->declare_parameter<std::string>("taskFile", "");
    this->declare_parameter<std::string>("urdfFile", "");
    this->declare_parameter<std::string>("libFolder", "");
    this->declare_parameter<std::string>("robotName", std::string("mobile_manipulator"));

    this->declare_parameter<double>("publishRate", params_.publish_rate_hz);
    this->declare_parameter<double>("horizon", params_.horizon_T);
    this->declare_parameter<double>("dt", params_.dt);
    this->declare_parameter<double>("amplitude", params_.amplitude);
    this->declare_parameter<double>("frequency", params_.frequency_hz);
    this->declare_parameter<double>("baseAmplitude", params_.base_amplitude);
    this->declare_parameter<double>("baseFrequency", params_.base_frequency_hz);
    this->declare_parameter<double>("baseYaw", params_.base_yaw);
    this->declare_parameter<double>("baseSlope", params_.base_slope);
    this->declare_parameter<double>("axisX", 0.0);
    this->declare_parameter<double>("axisY", 0.0);
    this->declare_parameter<double>("axisZ", 1.0);
    this->declare_parameter<std::string>("trajectoryGlobalFrame", std::string("world"));

    taskFile_ = this->get_parameter("taskFile").as_string();
    urdfFile_ = this->get_parameter("urdfFile").as_string();
    libFolder_ = this->get_parameter("libFolder").as_string();
    robotName_ = this->get_parameter("robotName").as_string();

    params_.publish_rate_hz = this->get_parameter("publishRate").as_double();
    params_.horizon_T = this->get_parameter("horizon").as_double();
    params_.dt = this->get_parameter("dt").as_double();
    params_.amplitude = this->get_parameter("amplitude").as_double();
    params_.frequency_hz = this->get_parameter("frequency").as_double();
    params_.base_amplitude = this->get_parameter("baseAmplitude").as_double();
    params_.base_frequency_hz = this->get_parameter("baseFrequency").as_double();
    params_.base_yaw = this->get_parameter("baseYaw").as_double();
    params_.base_slope = this->get_parameter("baseSlope").as_double();
    axis_.x() = this->get_parameter("axisX").as_double();
    axis_.y() = this->get_parameter("axisY").as_double();
    axis_.z() = this->get_parameter("axisZ").as_double();
    if (axis_.norm() < 1e-9) axis_ = Eigen::Vector3d::UnitZ();
    axis_.normalize();
    trajectory_global_frame_ = this->get_parameter("trajectoryGlobalFrame").as_string();

    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    if (!taskFile_.empty() && !urdfFile_.empty() && !libFolder_.empty()) {
      try {
        interface_ = std::make_unique<MobileManipulatorInterface>(taskFile_, libFolder_, urdfFile_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
      }
    }

    targetPub_ = this->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(robotName_ + std::string("_mpc_target"), 1);
    baseTargetPub_ =
        this->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(robotName_ + std::string("_base_mpc_target"), 1);
    targetMarkerPub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/mobile_manipulator/targetStateTrajectory", 1);
    targetPosePub_ =
        this->create_publisher<geometry_msgs::msg::PoseArray>("/mobile_manipulator/targetPoseTrajectory", 1);

    obsSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName_ + std::string("_mpc_observation"), 1,
        [&](const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mtx_);
          latestObs_ = ros_msg_conversions::readObservationMsg(*msg);
          haveObs_ = true;
        });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, params_.publish_rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&TrajectoryTargetNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "Trajectory target node started. Publishing to %s_mpc_target and %s_base_mpc_target at %.1f Hz",
                robotName_.c_str(), robotName_.c_str(), params_.publish_rate_hz);
  }

 private:
  void onTimer() {
    SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!haveObs_) return;
      obs = latestObs_;
    }

    if (!initialized_) {
      if (interface_) {
        const auto& info = interface_->getManipulatorModelInfo();
        switch (info.manipulatorModelType) {
          case ManipulatorModelType::WheelBasedMobileManipulator:
            basePoseDim_ = 3;
            break;
          case ManipulatorModelType::FloatingArmManipulator:
          case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
            basePoseDim_ = 6;
            break;
          case ManipulatorModelType::DefaultManipulator:
          default:
            basePoseDim_ = 0;
            break;
        }
        baseInputDim_ = info.inputDim >= info.armDim ? (info.inputDim - info.armDim) : 0;
      } else if (obs.state.size() >= 3) {
        basePoseDim_ = 3;
        baseInputDim_ = obs.input.size() >= 3 ? 3 : (obs.input.size() >= 2 ? 2 : 0);
      }

      if (basePoseDim_ >= 3 && obs.state.size() >= static_cast<Eigen::Index>(basePoseDim_)) {
        base_center_p_.x() = obs.state(0);
        base_center_p_.y() = obs.state(1);
        if (basePoseDim_ == 6) {
          base_center_p_.z() = obs.state(2);
        } else {
          base_center_p_.z() = 0.0;
        }
      }

      if (interface_) {
        try {
          const auto& pin = interface_->getPinocchioInterface();
          const auto& model = pin.getModel();
          auto data = pin.getData();
          pinocchio::forwardKinematics(model, data, obs.state);
          pinocchio::updateFramePlacements(model, data);
          const auto& info = interface_->getManipulatorModelInfo();
          const auto ee_id = model.getFrameId(info.eeFrame);
          const auto& ee = data.oMf[ee_id];
          center_p_ = ee.translation();
          q0_ = Eigen::Quaterniond(ee.rotation());
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "FK failed: %s", e.what());
          center_p_.setZero();
          q0_ = Eigen::Quaterniond::Identity();
        }
      }
      start_time_ = obs.time;
      initialized_ = true;
    }

    const int N = std::max(1, static_cast<int>(std::ceil(params_.horizon_T / std::max(1e-6, params_.dt))));

    scalar_array_t timeTraj;
    vector_array_t stateTraj;
    eightShapeTrajectory(center_p_, q0_.normalized(), axis_, start_time_, obs.time, N, params_.dt, params_.amplitude,
                         params_.frequency_hz, timeTraj, stateTraj);

    vector_array_t inputTraj;
    inputTraj.resize(timeTraj.size(), vector_t::Zero(obs.input.size()));

    TargetTrajectories traj(std::move(timeTraj), std::move(stateTraj), std::move(inputTraj));
    const auto targetMsg = ros_msg_conversions::createTargetTrajectoriesMsg(traj);
    targetPub_->publish(targetMsg);

    if (baseTargetPub_ && basePoseDim_ > 0) {
      scalar_array_t baseTimeTraj;
      vector_array_t baseStateTraj;
      sinusoidalLineBaseTrajectory(base_center_p_, params_.base_yaw, params_.base_slope, basePoseDim_,
                                 start_time_, obs.time, N, params_.dt, params_.base_amplitude,
                                 params_.base_frequency_hz, baseTimeTraj, baseStateTraj);

      vector_array_t baseInputTraj;
      baseInputTraj.resize(baseTimeTraj.size(), vector_t::Zero(static_cast<Eigen::Index>(baseInputDim_)));

      TargetTrajectories baseTraj(std::move(baseTimeTraj), std::move(baseStateTraj), std::move(baseInputTraj));
      const auto baseTargetMsg = ros_msg_conversions::createTargetTrajectoriesMsg(baseTraj);
      baseTargetPub_->publish(baseTargetMsg);
    }

    // Broadcast "command" TF as the *current* command point (front)
    if (tfBroadcaster_ && !traj.stateTrajectory.empty()) {
      const auto& x = traj.stateTrajectory.front();  // [px py pz qx qy qz qw]
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = this->now();
      tf.header.frame_id = trajectory_global_frame_;
      tf.child_frame_id = "command";
      tf.transform.translation.x = x(0);
      tf.transform.translation.y = x(1);
      tf.transform.translation.z = x(2);

      Eigen::Quaterniond q(x(6), x(3), x(4), x(5));  // w, x, y, z
      q.normalize();
      tf.transform.rotation = ros_msg_helpers::getOrientationMsg(q);
      tfBroadcaster_->sendTransform(tf);
    }

    visualization_msgs::msg::MarkerArray markerArray;
    markerArray.markers.reserve(traj.size());
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = trajectory_global_frame_;
    poseArray.header.stamp = this->now();
    poseArray.poses.reserve(traj.size());

    for (size_t i = 0; i < traj.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = trajectory_global_frame_;
      marker.header.stamp = this->now();
      marker.ns = "target_state";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.0025;
      marker.color.r = 0.4660f;
      marker.color.g = 0.6740f;
      marker.color.b = 0.1880f;
      marker.color.a = 1.0f;

      marker.pose.position = ros_msg_helpers::getPointMsg(traj.stateTrajectory[i].head<3>());
      const auto quat_vec = traj.stateTrajectory[i].tail<4>();  // [qx qy qz qw]
      Eigen::Quaterniond quat(quat_vec[3], quat_vec[0], quat_vec[1], quat_vec[2]);
      marker.pose.orientation = ros_msg_helpers::getOrientationMsg(quat.normalized());
      markerArray.markers.push_back(marker);

      geometry_msgs::msg::Pose pose;
      pose.position = marker.pose.position;
      pose.orientation = marker.pose.orientation;
      poseArray.poses.push_back(pose);
    }

    targetMarkerPub_->publish(markerArray);
    targetPosePub_->publish(poseArray);
  }

  Params params_;
  Eigen::Vector3d axis_{0, 0, 1};
  std::string taskFile_, urdfFile_, libFolder_, robotName_;
  std::string trajectory_global_frame_{"world"};

  std::unique_ptr<MobileManipulatorInterface> interface_;

  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetPub_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr baseTargetPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targetMarkerPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr targetPosePub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr obsSub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::mutex mtx_;
  SystemObservation latestObs_;
  bool haveObs_{false};

  bool initialized_{false};
  double start_time_{0.0};
  Eigen::Vector3d center_p_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q0_ = Eigen::Quaterniond::Identity();
  size_t basePoseDim_{0};
  size_t baseInputDim_{0};
  Eigen::Vector3d base_center_p_ = Eigen::Vector3d::Zero();
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(false);
  auto node = std::make_shared<TrajectoryTargetNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
