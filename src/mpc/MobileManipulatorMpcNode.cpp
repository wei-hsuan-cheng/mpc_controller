/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <mobile_manipulator_mpc/reference/MobileManipulatorReferenceManager.h>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <utility>

#include "rclcpp/rclcpp.hpp"

using namespace ocs2;
using namespace ocs2::mobile_manipulator_mpc;

namespace {

EnvironmentObstacleArray readEnvironmentObstacles(const visualization_msgs::msg::MarkerArray& msg) {
  EnvironmentObstacleArray obstacles;
  obstacles.reserve(msg.markers.size());

  for (const auto& marker : msg.markers) {
    if (marker.type != visualization_msgs::msg::Marker::SPHERE ||
        marker.action != visualization_msgs::msg::Marker::ADD) {
      continue;
    }

    const double diameter = std::max({marker.scale.x, marker.scale.y, marker.scale.z});
    if (diameter <= 0.0) {
      continue;
    }

    SphericalObstacle obstacle;
    obstacle.center.x() = marker.pose.position.x;
    obstacle.center.y() = marker.pose.position.y;
    obstacle.center.z() = marker.pose.position.z;
    obstacle.radius = 0.5 * diameter;
    obstacles.push_back(std::move(obstacle));
  }

  return obstacles;
}

}  // namespace
int main(int argc, char **argv) {
  const std::string robotName = "mobile_manipulator";

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      robotName + "_mpc",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  std::string taskFile = node->get_parameter("taskFile").as_string();
  std::string libFolder = node->get_parameter("libFolder").as_string();
  std::string urdfFile = node->get_parameter("urdfFile").as_string();
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;

  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  auto rosReferenceManagerPtr =
      std::make_shared<RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node);

  auto mobileManipulatorReferenceManagerPtr =
      std::dynamic_pointer_cast<MobileManipulatorReferenceManager>(interface.getReferenceManagerPtr());
  rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr baseTargetSubscriber;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr envObstacleSubscriber;
  if (mobileManipulatorReferenceManagerPtr) {
    baseTargetSubscriber = node->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
        robotName + std::string("_base_mpc_target"), 1,
        [mobileManipulatorReferenceManagerPtr](const ocs2_msgs::msg::MpcTargetTrajectories& msg) {
          auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(msg);
          mobileManipulatorReferenceManagerPtr->setBaseTargetTrajectories(std::move(targetTrajectories));
        });
    envObstacleSubscriber = node->create_subscription<visualization_msgs::msg::MarkerArray>(
        robotName + std::string("_env_obstacles"), 1,
        [mobileManipulatorReferenceManagerPtr](const visualization_msgs::msg::MarkerArray& msg) {
          auto obstacles = readEnvironmentObstacles(msg);
          mobileManipulatorReferenceManagerPtr->setEnvironmentObstacles(std::move(obstacles));
        });
  }

  GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(),
                         interface.getOptimalControlProblem(), interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(node);

  (void)baseTargetSubscriber;
  (void)envObstacleSubscriber;
  return 0;
}
