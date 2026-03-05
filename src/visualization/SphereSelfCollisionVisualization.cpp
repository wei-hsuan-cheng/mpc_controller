#include "mpc_controller/visualization/SphereSelfCollisionVisualization.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>

namespace mpc_controller {
namespace {

using ocs2::mobile_manipulator_mpc::vector3_t;

constexpr size_t kInvalidIndex = static_cast<size_t>(-1);
constexpr double kDistanceEps = 1e-9;
constexpr double kLabelOffsetZ = 0.03;
constexpr double kLabelLateralOffset = 0.015;

constexpr std::array<double, 3> kSphereColor{0.121, 0.466, 0.705};
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

SphereSelfCollisionVisualization::SphereSelfCollisionVisualization(
    const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface,
    std::vector<ocs2::mobile_manipulator_mpc::RobotCollisionSphere> robotSpheres,
    std::vector<std::pair<size_t, size_t>> spherePairs, const ocs2::scalar_t minimumDistance, std::string globalFrame)
    : node_(node),
      pinocchioInterface_(std::move(pinocchioInterface)),
      minimumDistance_(std::max<ocs2::scalar_t>(minimumDistance, 0.0)),
      globalFrame_(std::move(globalFrame)) {
  markerPub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>("/mobile_manipulator/sphereSelfCollisionMarkers", 1);

  std::vector<size_t> mapToCompact(robotSpheres.size(), kInvalidIndex);
  robotSpheres_.reserve(robotSpheres.size());

  const auto& model = pinocchioInterface_.getModel();
  for (size_t i = 0; i < robotSpheres.size(); ++i) {
    const auto& sphere = robotSpheres[i];
    if (!sphere.active || sphere.radius <= 0.0 || sphere.frameId >= model.frames.size()) {
      continue;
    }

    mapToCompact[i] = robotSpheres_.size();
    robotSpheres_.push_back(sphere);
  }

  if (spherePairs.empty()) {
    spherePairs.reserve((robotSpheres.size() * (robotSpheres.size() - 1)) / 2);
    for (size_t i = 0; i < robotSpheres.size(); ++i) {
      for (size_t j = i + 1; j < robotSpheres.size(); ++j) {
        spherePairs.emplace_back(i, j);
      }
    }
  }

  std::unordered_set<uint64_t> seen;
  spherePairs_.reserve(spherePairs.size());
  for (const auto& pair : spherePairs) {
    if (pair.first >= mapToCompact.size() || pair.second >= mapToCompact.size()) {
      continue;
    }

    const size_t first = mapToCompact[pair.first];
    const size_t second = mapToCompact[pair.second];
    if (first == kInvalidIndex || second == kInvalidIndex || first == second) {
      continue;
    }

    const std::pair<size_t, size_t> ordered{std::min(first, second), std::max(first, second)};
    const uint64_t key = (static_cast<uint64_t>(ordered.first) << 32u) | static_cast<uint64_t>(ordered.second);
    if (seen.insert(key).second) {
      spherePairs_.push_back(ordered);
    }
  }

  if (robotSpheres_.empty() || spherePairs_.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Sphere self-collision visualization has no valid data (spheres=%zu, pairs=%zu).",
                robotSpheres_.size(), spherePairs_.size());
  }
}

std::vector<SphereSelfCollisionVisualization::EvaluatedSphere> SphereSelfCollisionVisualization::evaluateSpheres(
    const ocs2::vector_t& state) {
  std::vector<EvaluatedSphere> evaluatedSpheres;
  evaluatedSpheres.reserve(robotSpheres_.size());

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);

  for (const auto& sphere : robotSpheres_) {
    EvaluatedSphere evaluated;
    evaluated.sphere = sphere;
    evaluated.center = data.oMf[sphere.frameId].translation();
    evaluatedSpheres.push_back(std::move(evaluated));
  }
  return evaluatedSpheres;
}

void SphereSelfCollisionVisualization::publish(const ocs2::vector_t& state, const rclcpp::Time& stamp) {
  if (!markerPub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.push_back(makeDeleteAllMarker(globalFrame_, stamp));

  if (robotSpheres_.empty() || spherePairs_.empty()) {
    markerPub_->publish(markerArray);
    return;
  }

  const auto spheres = evaluateSpheres(state);
  int markerId = 0;
  for (const auto& sphere : spheres) {
    visualization_msgs::msg::Marker marker;
    marker.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
    marker.ns = "self_collision_spheres";
    marker.id = markerId++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ocs2::ros_msg_helpers::getPointMsg(sphere.center);
    marker.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
    marker.scale.x = 2.0 * sphere.sphere.radius;
    marker.scale.y = 2.0 * sphere.sphere.radius;
    marker.scale.z = 2.0 * sphere.sphere.radius;
    marker.color = ocs2::ros_msg_helpers::getColor(kSphereColor, 0.20);
    markerArray.markers.push_back(std::move(marker));
  }

  for (size_t pairIdx = 0; pairIdx < spherePairs_.size(); ++pairIdx) {
    const auto [firstIdx, secondIdx] = spherePairs_[pairIdx];
    if (firstIdx >= spheres.size() || secondIdx >= spheres.size()) {
      continue;
    }

    const auto& first = spheres[firstIdx];
    const auto& second = spheres[secondIdx];
    const vector3_t separation = second.center - first.center;
    const double centerDistance = separation.norm();
    vector3_t direction = vector3_t::UnitX();
    if (centerDistance > kDistanceEps) {
      direction = separation / centerDistance;
    }

    const vector3_t firstSurface = first.center + direction * first.sphere.radius;
    const vector3_t secondSurface = second.center - direction * second.sphere.radius;
    const double signedMargin = centerDistance - (first.sphere.radius + second.sphere.radius + minimumDistance_);
    const auto color = getDistanceColor(signedMargin);

    visualization_msgs::msg::Marker arrow;
    arrow.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
    arrow.ns = "self_collision_pair_arrow";
    arrow.id = markerId++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
    arrow.points.push_back(ocs2::ros_msg_helpers::getPointMsg(firstSurface));
    arrow.points.push_back(ocs2::ros_msg_helpers::getPointMsg(secondSurface));
    arrow.scale.x = 0.008;
    arrow.scale.y = 0.016;
    arrow.scale.z = 0.024;
    arrow.color = color;
    markerArray.markers.push_back(std::move(arrow));

    visualization_msgs::msg::Marker endpoints;
    endpoints.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
    endpoints.ns = "self_collision_pair_endpoints";
    endpoints.id = markerId++;
    endpoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    endpoints.action = visualization_msgs::msg::Marker::ADD;
    endpoints.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
    endpoints.points.push_back(ocs2::ros_msg_helpers::getPointMsg(firstSurface));
    endpoints.points.push_back(ocs2::ros_msg_helpers::getPointMsg(secondSurface));
    endpoints.scale.x = 0.02;
    endpoints.scale.y = 0.02;
    endpoints.scale.z = 0.02;
    endpoints.color = color;
    markerArray.markers.push_back(std::move(endpoints));

    visualization_msgs::msg::Marker text;
    text.header = ocs2::ros_msg_helpers::getHeaderMsg(globalFrame_, stamp);
    text.ns = "self_collision_pair_text";
    text.id = markerId++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    vector3_t labelPosition = (firstSurface + secondSurface) * 0.5;
    vector3_t lateral = direction.cross(vector3_t::UnitZ());
    if (lateral.norm() < kDistanceEps) {
      lateral = direction.cross(vector3_t::UnitY());
    }
    if (lateral.norm() > kDistanceEps) {
      labelPosition += lateral.normalized() * kLabelLateralOffset;
    }
    text.pose.position = ocs2::ros_msg_helpers::getPointMsg(labelPosition);
    text.pose.position.z += kLabelOffsetZ;
    text.pose.orientation = ocs2::ros_msg_helpers::getOrientationMsg(Eigen::Quaterniond::Identity());
    text.scale.z = 0.03;
    text.color = color;

    std::ostringstream stream;
    stream << "margin=" << std::fixed << std::setprecision(3) << signedMargin;
    text.text = stream.str();
    markerArray.markers.push_back(std::move(text));
  }

  markerPub_->publish(markerArray);
}

}  // namespace mpc_controller
