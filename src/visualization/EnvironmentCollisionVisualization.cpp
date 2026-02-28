#include "mpc_controller/visualization/EnvironmentCollisionVisualization.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

namespace mpc_controller {
namespace {

using ocs2::mobile_manipulator_mpc::EnvironmentObstacleArray;
using ocs2::mobile_manipulator_mpc::vector3_t;

constexpr double kDistanceEps = 1e-9;
constexpr double kLabelOffsetZ = 0.03;

constexpr std::array<double, 3> kRobotSphereColor{0.121, 0.466, 0.705};
constexpr std::array<double, 3> kStaticObstacleColor{0.850, 0.325, 0.098};
constexpr std::array<double, 3> kDynamicObstacleColor{0.890, 0.102, 0.110};
constexpr std::array<double, 3> kClearColor{0.173, 0.627, 0.173};
constexpr std::array<double, 3> kNearColor{1.000, 0.498, 0.055};
constexpr std::array<double, 3> kViolationColor{0.839, 0.153, 0.157};

visualization_msgs::msg::Marker makeDeleteAllMarker(const std::string& frameId, const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header = ocs2::ros_msg_helpers::getHeaderMsg(frameId, stamp);
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

std_msgs::msg::ColorRGBA getDistanceColor(double signedMargin) {
  if (signedMargin < 0.0) {
    return ocs2::ros_msg_helpers::getColor(kViolationColor, 1.0);
  }
  if (signedMargin < 0.05) {
    return ocs2::ros_msg_helpers::getColor(kNearColor, 1.0);
  }
  return ocs2::ros_msg_helpers::getColor(kClearColor, 1.0);
}

}  // namespace

EnvironmentCollisionVisualization::EnvironmentCollisionVisualization(
    const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface,
    std::shared_ptr<ocs2::mobile_manipulator_mpc::MobileManipulatorReferenceManager> referenceManager,
    ocs2::mobile_manipulator_mpc::EnvironmentCollisionConfig config, std::string globalFrame)
    : node_(node),
      pinocchioInterface_(std::move(pinocchioInterface)),
      referenceManager_(std::move(referenceManager)),
      config_(std::move(config)),
      globalFrame_(std::move(globalFrame)) {
  sceneMarkerPub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("/mobile_manipulator/environmentSceneMarkers", 1);
  distanceMarkerPub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("/mobile_manipulator/environmentDistanceMarkers", 1);
}

EnvironmentObstacleArray EnvironmentCollisionVisualization::getActiveObstacles() const {
  EnvironmentObstacleArray obstacles;
  obstacles.reserve(config_.staticObstacles.size() + (referenceManager_ ? referenceManager_->getEnvironmentObstacles().size() : 0));

  for (const auto& obstacle : config_.staticObstacles) {
    if (obstacle.active && obstacle.radius > 0.0) {
      obstacles.push_back(obstacle);
    }
  }
  if (referenceManager_) {
    for (const auto& obstacle : referenceManager_->getEnvironmentObstacles()) {
      if (obstacle.active && obstacle.radius > 0.0) {
        obstacles.push_back(obstacle);
      }
    }
  }
  return obstacles;
}

std::vector<EnvironmentCollisionVisualization::EvaluatedRobotSphere>
EnvironmentCollisionVisualization::evaluateRobotSpheres(const ocs2::vector_t& state) {
  std::vector<EvaluatedRobotSphere> robotSpheres;
  robotSpheres.reserve(config_.robotSpheres.size());

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);

  for (const auto& sphere : config_.robotSpheres) {
    if (!sphere.active || sphere.radius <= 0.0 || sphere.frameId >= model.frames.size()) {
      continue;
    }
    EvaluatedRobotSphere evaluated;
    evaluated.sphere = sphere;
    evaluated.center = data.oMf[sphere.frameId].translation();
    robotSpheres.push_back(std::move(evaluated));
  }

  return robotSpheres;
}

void EnvironmentCollisionVisualization::publishSceneMarkers(
    const rclcpp::Time& stamp, const std::vector<EvaluatedRobotSphere>& robotSpheres,
    const ocs2::mobile_manipulator_mpc::EnvironmentObstacleArray& obstacles) {
  if (!sceneMarkerPub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.push_back(makeDeleteAllMarker(globalFrame_, stamp));

  int markerId = 0;
  for (const auto& sphere : robotSpheres) {
    visualization_msgs::msg::Marker marker;
    marker.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
    marker.ns = "robot_spheres";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ocs2::ros_msg_helpers::getPointMsg(sphere.center);
    marker.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
    marker.scale.x = 2.0 * sphere.sphere.radius;
    marker.scale.y = 2.0 * sphere.sphere.radius;
    marker.scale.z = 2.0 * sphere.sphere.radius;
    marker.color = ocs2::ros_msg_helpers::getColor(kRobotSphereColor, 0.25);
    marker.text = sphere.sphere.frameName;
    markerArray.markers.push_back(std::move(marker));
  }

  const size_t staticObstacleCount = std::count_if(config_.staticObstacles.begin(), config_.staticObstacles.end(), [](const auto& obstacle) {
    return obstacle.active && obstacle.radius > 0.0;
  });
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obstacle = obstacles[i];
    const bool isStatic = i < staticObstacleCount;
    visualization_msgs::msg::Marker marker;
    marker.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
    marker.ns = isStatic ? "static_obstacles" : "dynamic_obstacles";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ocs2::ros_msg_helpers::getPointMsg(obstacle.center);
    marker.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
    marker.scale.x = 2.0 * obstacle.radius;
    marker.scale.y = 2.0 * obstacle.radius;
    marker.scale.z = 2.0 * obstacle.radius;
    marker.color = ocs2::ros_msg_helpers::getColor(isStatic ? kStaticObstacleColor : kDynamicObstacleColor, 0.45);
    markerArray.markers.push_back(std::move(marker));
  }

  sceneMarkerPub_->publish(markerArray);
}

void EnvironmentCollisionVisualization::publishDistanceMarkers(
    const rclcpp::Time& stamp, const std::vector<EvaluatedRobotSphere>& robotSpheres,
    const ocs2::mobile_manipulator_mpc::EnvironmentObstacleArray& obstacles) {
  if (!distanceMarkerPub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.push_back(makeDeleteAllMarker(globalFrame_, stamp));

  int markerId = 0;
  for (const auto& robotSphere : robotSpheres) {
    for (size_t obstacleIdx = 0; obstacleIdx < obstacles.size(); ++obstacleIdx) {
      const auto& obstacle = obstacles[obstacleIdx];
      const vector3_t separation = obstacle.center - robotSphere.center;
      const double centerDistance = separation.norm();
      vector3_t direction = vector3_t::UnitX();
      if (centerDistance > kDistanceEps) {
        direction = separation / centerDistance;
      }
      const vector3_t robotSurfacePoint = robotSphere.center + direction * robotSphere.sphere.radius;
      const vector3_t obstacleSurfacePoint = obstacle.center - direction * obstacle.radius;
      const double signedMargin = centerDistance - (robotSphere.sphere.radius + obstacle.radius + config_.minimumDistance);
      const auto color = getDistanceColor(signedMargin);

      visualization_msgs::msg::Marker arrow;
      arrow.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
      arrow.ns = "env_distance_arrow";
      arrow.id = markerId++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
      arrow.points.push_back(ocs2::ros_msg_helpers::getPointMsg(robotSurfacePoint));
      arrow.points.push_back(ocs2::ros_msg_helpers::getPointMsg(obstacleSurfacePoint));
      arrow.scale.x = 0.008;
      arrow.scale.y = 0.016;
      arrow.scale.z = 0.024;
      arrow.color = color;
      markerArray.markers.push_back(std::move(arrow));

      visualization_msgs::msg::Marker endpoints;
      endpoints.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
      endpoints.ns = "env_distance_endpoints";
      endpoints.id = markerId++;
      endpoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      endpoints.action = visualization_msgs::msg::Marker::ADD;
      endpoints.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
      endpoints.points.push_back(ocs2::ros_msg_helpers::getPointMsg(robotSurfacePoint));
      endpoints.points.push_back(ocs2::ros_msg_helpers::getPointMsg(obstacleSurfacePoint));
      endpoints.scale.x = 0.02;
      endpoints.scale.y = 0.02;
      endpoints.scale.z = 0.02;
      endpoints.color = color;
      markerArray.markers.push_back(std::move(endpoints));

      visualization_msgs::msg::Marker text;
      text.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
      text.ns = "env_distance_text";
      text.id = markerId++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position = ocs2::ros_msg_helpers::getPointMsg((robotSurfacePoint + obstacleSurfacePoint) * 0.5);
      text.pose.position.z += kLabelOffsetZ;
      text.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
      text.scale.z = 0.03;
      text.color = color;
      std::ostringstream stream;
      stream << robotSphere.sphere.frameName << "->obs" << obstacleIdx << " margin=" << std::fixed << std::setprecision(3)
             << signedMargin;
      text.text = stream.str();
      markerArray.markers.push_back(std::move(text));
    }
  }

  distanceMarkerPub_->publish(markerArray);
}

void EnvironmentCollisionVisualization::publish(const ocs2::vector_t& state, const rclcpp::Time& stamp) {
  const auto robotSpheres = evaluateRobotSpheres(state);
  const auto obstacles = getActiveObstacles();
  publishSceneMarkers(stamp, robotSpheres, obstacles);
  publishDistanceMarkers(stamp, robotSpheres, obstacles);
}

}  // namespace mpc_controller
