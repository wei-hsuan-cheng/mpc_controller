#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <mobile_manipulator_mpc/EnvironmentCollisionConfig.h>

namespace mpc_controller {

class SphereSelfCollisionVisualization {
 public:
  SphereSelfCollisionVisualization(
      const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface,
      std::vector<ocs2::mobile_manipulator_mpc::RobotCollisionSphere> robotSpheres,
      std::vector<std::pair<size_t, size_t>> spherePairs, ocs2::scalar_t minimumDistance, std::string globalFrame);

  void publish(const ocs2::vector_t& state, const rclcpp::Time& stamp);

 private:
  struct EvaluatedSphere {
    ocs2::mobile_manipulator_mpc::RobotCollisionSphere sphere;
    ocs2::mobile_manipulator_mpc::vector3_t center = ocs2::mobile_manipulator_mpc::vector3_t::Zero();
  };

  std::vector<EvaluatedSphere> evaluateSpheres(const ocs2::vector_t& state);

  rclcpp::Node::SharedPtr node_;
  ocs2::PinocchioInterface pinocchioInterface_;
  std::vector<ocs2::mobile_manipulator_mpc::RobotCollisionSphere> robotSpheres_;
  std::vector<std::pair<size_t, size_t>> spherePairs_;
  ocs2::scalar_t minimumDistance_{0.0};
  std::string globalFrame_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
};

}  // namespace mpc_controller
