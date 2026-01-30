#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

using namespace std::chrono_literals;

namespace {

std::vector<double> getDoubleArrayParam(rclcpp::Node& node, const std::string& name) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).as_double_array();
  }
  return {};
}

void resizeOrFill(std::vector<double>& v, size_t n, double fill) {
  if (v.empty()) {
    v.assign(n, fill);
    return;
  }
  if (v.size() < n) {
    v.resize(n, fill);
  } else if (v.size() > n) {
    v.resize(n);
  }
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("mobile_manipulator_joint_trajectory_target");

  // Required parameters
  node->declare_parameter<std::string>("robot_name", "mobile_manipulator");
  node->declare_parameter<std::string>("task_file", "");
  node->declare_parameter<std::string>("urdf_file", "");
  node->declare_parameter<std::string>("lib_folder", "");

  // Timing parameters
  node->declare_parameter<double>("publish_rate_hz", 100.0);
  node->declare_parameter<double>("horizon_s", 5.0);
  node->declare_parameter<double>("dt_s", 0.01);

  // Trajectory parameters (scalar)
  node->declare_parameter<double>("amplitude", 0.05);
  node->declare_parameter<double>("frequency_hz", 0.2);

  // Trajectory parameters (vectors)
  node->declare_parameter<std::vector<double>>("q_nominal", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("q_amplitude", std::vector<double>{});
  node->declare_parameter<std::vector<double>>("q_phase_rad", std::vector<double>{});

  const std::string robotName = node->get_parameter("robot_name").as_string();
  const std::string taskFile = node->get_parameter("task_file").as_string();
  const std::string urdfFile = node->get_parameter("urdf_file").as_string();
  const std::string libFolder = node->get_parameter("lib_folder").as_string();

  if (robotName.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter 'robot_name' must be set.");
    return 1;
  }
  if (taskFile.empty() || urdfFile.empty() || libFolder.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameters 'task_file', 'urdf_file', and 'lib_folder' must be set.");
    return 1;
  }

  const double publishRateHz = node->get_parameter("publish_rate_hz").as_double();
  const double horizonS = node->get_parameter("horizon_s").as_double();
  const double dtS = node->get_parameter("dt_s").as_double();

  if (publishRateHz <= 0.0 || horizonS <= 0.0 || dtS <= 0.0) {
    RCLCPP_FATAL(node->get_logger(), "Invalid timing params: publish_rate_hz=%.3f horizon_s=%.3f dt_s=%.3f",
                 publishRateHz, horizonS, dtS);
    return 1;
  }

  const double ampScalar = node->get_parameter("amplitude").as_double();
  const double freqHz = node->get_parameter("frequency_hz").as_double();

  // Load model and dimensions
  ocs2::mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);
  const auto& modelInfo = interface.getManipulatorModelInfo();
  const size_t armDim = modelInfo.armDim;
  const size_t inputDim = modelInfo.inputDim;

  if (armDim == 0) {
    RCLCPP_FATAL(node->get_logger(), "armDim is zero; this node publishes joint references.");
    return 1;
  }

  const int N = std::max(1, static_cast<int>(std::floor(horizonS / dtS)));

  // Read vectors
  std::vector<double> qNom = getDoubleArrayParam(*node, "q_nominal");
  std::vector<double> qAmp = getDoubleArrayParam(*node, "q_amplitude");
  std::vector<double> qPhase = getDoubleArrayParam(*node, "q_phase_rad");

  // Apply fallbacks and sizing
  resizeOrFill(qNom, armDim, 0.0);
  resizeOrFill(qAmp, armDim, ampScalar);
  resizeOrFill(qPhase, armDim, 0.0);

  // Observation cache
  ocs2::SystemObservation latestObs;
  bool hasObs = false;

  const std::string obsTopic = robotName + "_mpc_observation";
  const std::string jointTargetTopic = robotName + "_mpc_joint_target";

  auto obsSub = node->create_subscription<ocs2_msgs::msg::MpcObservation>(
      obsTopic,
      rclcpp::QoS(1).reliable(),
      [&](const ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
        latestObs = ocs2::ros_msg_conversions::readObservationMsg(*msg);
        hasObs = true;
      });

  auto targetPub = node->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      jointTargetTopic,
      rclcpp::QoS(1).reliable());

  const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publishRateHz));
  auto timer = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [&]() {
        if (!hasObs) {
          return;
        }

        const ocs2::scalar_t t0 = latestObs.time;

        ocs2::scalar_array_t timeTraj;
        ocs2::vector_array_t stateTraj;
        ocs2::vector_array_t inputTraj;
        timeTraj.reserve(static_cast<size_t>(N + 1));
        stateTraj.reserve(static_cast<size_t>(N + 1));
        inputTraj.reserve(static_cast<size_t>(N + 1));

        for (int k = 0; k <= N; ++k) {
          const double t = static_cast<double>(t0) + static_cast<double>(k) * dtS;

          ocs2::vector_t q = ocs2::vector_t::Zero(static_cast<long>(armDim));
          for (size_t i = 0; i < armDim; ++i) {
            const double w = 2.0 * M_PI * freqHz;
            q(static_cast<long>(i)) = qNom[i] + qAmp[i] * std::sin(w * t + qPhase[i]);
          }

          timeTraj.push_back(t);
          stateTraj.push_back(q);
          inputTraj.push_back(ocs2::vector_t::Zero(static_cast<long>(inputDim)));
        }

        ocs2::TargetTrajectories target(timeTraj, stateTraj, inputTraj);
        auto msg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(target);
        targetPub->publish(msg);
      });

  RCLCPP_INFO(node->get_logger(),
              "Publishing joint targets on '%s' (subscribing '%s'), armDim=%zu, rate=%.1f Hz, horizon=%.2f s, dt=%.3f s",
              jointTargetTopic.c_str(), obsTopic.c_str(), armDim, publishRateHz, horizonS, dtS);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
