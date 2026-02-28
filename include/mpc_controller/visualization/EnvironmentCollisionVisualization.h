#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <mobile_manipulator_mpc/EnvironmentCollisionConfig.h>
#include <mobile_manipulator_mpc/reference/MobileManipulatorReferenceManager.h>

namespace mpc_controller {

class EnvironmentCollisionVisualization {
 public:
  EnvironmentCollisionVisualization(
      const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface,
      std::shared_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorReferenceManager> referenceManager,
      ocs2::mobile_manipulator_mpc::EnvironmentCollisionConfig config, std::string globalFrame);

  void publish(const ocs2::vector_t& state, const rclcpp::Time& stamp);

 private:
  struct EvaluatedRobotSphere {
    ocs2::mobile_manipulator_mpc::RobotCollisionSphere sphere;
    ocs2::mobile_manipulator_mpc::vector3_t center = ocs2::mobile_manipulator_mpc::vector3_t::Zero();
  };

  ocs2::mobile_manipulator_mpc::EnvironmentObstacleArray getActiveObstacles() const;
  std::vector<EvaluatedRobotSphere> evaluateRobotSpheres(const ocs2::vector_t& state);
  void publishSceneMarkers(const rclcpp::Time& stamp, const std::vector<EvaluatedRobotSphere>& robotSpheres,
                           const ocs2::mobile_manipulator_mpc::EnvironmentObstacleArray& obstacles);
  void publishDistanceMarkers(const rclcpp::Time& stamp, const std::vector<EvaluatedRobotSphere>& robotSpheres,
                              const ocs2::mobile_manipulator_mpc::EnvironmentObstacleArray& obstacles);

  rclcpp::Node::SharedPtr node_;
  ocs2::PinocchioInterface pinocchioInterface_;
  std::shared_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorReferenceManager> referenceManager_;
  ocs2::mobile_manipulator_mpc::EnvironmentCollisionConfig config_;
  std::string globalFrame_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sceneMarkerPub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr distanceMarkerPub_;
};

}  // namespace mpc_controller
