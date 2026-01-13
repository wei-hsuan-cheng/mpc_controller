/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_ros_interfaces/command/UnifiedTargetTrajectoriesInteractiveMarker.h>
#include <ocs2_ros_interfaces/command/JoystickMarkerWrapper.h>
#include <ocs2_ros_interfaces/command/MarkerAutoPositionWrapper.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <yaml-cpp/yaml.h>

using namespace ocs2;
using namespace ocs2::mobile_manipulator;

namespace {

/* YAML helper */
vector_t loadInitialStateFromYaml(const std::string& path, const ManipulatorModelInfo& info,
                                  const vector_t& defaultState, const rclcpp::Logger& logger) {
  if (path.empty()) {
    return defaultState;
  }

  vector_t state = defaultState;
  try {
    const auto root = YAML::LoadFile(path);
    const auto stateSize = static_cast<size_t>(state.size());
    const size_t baseDim = stateSize > info.armDim ? stateSize - info.armDim : 0;

    if (baseDim > 0 && root["base_pose"]) {
      const auto base = root["base_pose"];
      auto assignBase = [&](size_t idx, double value) {
        if (idx < baseDim) {
          state(idx) = value;
        }
      };
      if (base.IsMap()) {
        assignBase(0, base["x"].as<double>(state(0)));
        assignBase(1, base["y"].as<double>(state(1)));
        assignBase(2, base["yaw"].as<double>(state(2)));
      } else if (base.IsSequence() && base.size() >= 3) {
        assignBase(0, base[0].as<double>());
        assignBase(1, base[1].as<double>());
        assignBase(2, base[2].as<double>());
      }
    }

    if (info.armDim > 0 && root["arm_joints"]) {
      const auto joints = root["arm_joints"];
      const size_t offset = state.size() - info.armDim;
      if (joints.IsMap()) {
        for (size_t i = 0; i < info.dofNames.size(); ++i) {
          const auto& name = info.dofNames[i];
          if (joints[name]) {
            state(offset + i) = joints[name].as<double>(state(offset + i));
          }
        }
      } else if (joints.IsSequence()) {
        for (size_t i = 0; i < std::min<size_t>(info.armDim, joints.size()); ++i) {
          state(offset + i) = joints[i].as<double>(state(offset + i));
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger, "Failed to load initial pose override from %s: %s", path.c_str(), e.what());
  }
  return state;
}

} // namespace

/* Helpers */
bool readDualArmModeFromTaskFile(const std::string& taskFile) {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    bool dualArmMode = false;
    loadData::loadPtreeValue(pt, dualArmMode, "endEffector.dualArmMode", false);
    if (!dualArmMode) {
      loadData::loadPtreeValue(pt, dualArmMode, "finalEndEffector.dualArmMode", false);
    }

    return dualArmMode;
  } catch (const std::exception& e) {
    std::cerr << "Error reading dualArmMode from task file: " << e.what() << std::endl;
    return false;
  }
}

std::string getMarkerFrameFromTaskFile(const std::string& taskFile) {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);

    size_t manipulatorModelType = 1;
    loadData::loadPtreeValue(pt, manipulatorModelType, "model_information.manipulatorModelType", false);

    std::string baseFrame = "base_link";
    loadData::loadPtreeValue(pt, baseFrame, "model_information.baseFrame", false);

    if (manipulatorModelType == 0) {
      return baseFrame;
    } else {
      return "world";
    }
  } catch (const std::exception& e) {
    std::cerr << "Error reading frame information from task file: " << e.what() << std::endl;
    return "world";
  }
}

TargetTrajectories goalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                const SystemObservation& observation) {
  const scalar_array_t timeTrajectory{observation.time};
  const vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
  const vector_array_t stateTrajectory{target};
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories dualArmGoalPoseToTargetTrajectories(const Eigen::Vector3d& leftPosition,
                                                       const Eigen::Quaterniond& leftOrientation,
                                                       const Eigen::Vector3d& rightPosition,
                                                       const Eigen::Quaterniond& rightOrientation,
                                                       const SystemObservation& observation) {
  const scalar_array_t timeTrajectory{observation.time};
  const vector_t target = (vector_t(14) << leftPosition, leftOrientation.coeffs(), rightPosition, rightOrientation.coeffs()).finished();
  const vector_array_t stateTrajectory{target};
  const vector_array_t inputTrajectory{vector_t::Zero(observation.input.size())};
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/* Main function */
int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      robotName + "_target",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  std::string taskFile = node->get_parameter("taskFile").as_string();
  std::string urdfFile = "";
  std::string libFolder = "";
  try { urdfFile = node->get_parameter("urdfFile").as_string(); } catch (...) {}
  try { libFolder = node->get_parameter("libFolder").as_string(); } catch (...) {}

  std::string initialPoseFile = "";
  try { initialPoseFile = node->get_parameter("initialPoseFile").as_string(); } catch (...) {}

  double markerPublishRate = 10.0;
  try { markerPublishRate = node->get_parameter("markerPublishRate").as_double(); } catch (...) {}

  bool dualArmMode = readDualArmModeFromTaskFile(taskFile);

  bool enableDynamicFrame = false;
  if (node->has_parameter("enableDynamicFrame"))
  {
      enableDynamicFrame = node->get_parameter("enableDynamicFrame").as_bool();
      RCLCPP_INFO(node->get_logger(), "enableDynamicFrame parameter found: %s", enableDynamicFrame ? "true" : "false");
  }
  else
  {
      RCLCPP_INFO(node->get_logger(), "enableDynamicFrame parameter not found, using default: false");
  }

  std::string markerGlobalFrame = "";
  try {
    markerGlobalFrame = node->get_parameter("markerGlobalFrame").as_string();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
    markerGlobalFrame = "world";
  }
  if (enableDynamicFrame) {
    markerGlobalFrame = getMarkerFrameFromTaskFile(taskFile);
  }
  RCLCPP_INFO(node->get_logger(), "Marker frame: %s", markerGlobalFrame.c_str());

  bool enableJoystick = false;
  try { enableJoystick = node->get_parameter("enableJoystick").as_bool(); }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException&) { enableJoystick = false; }

  bool enableAutoPosition = false;
  try { enableAutoPosition = node->get_parameter("enableAutoPosition").as_bool(); }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException&) { enableAutoPosition = false; }

  std::unique_ptr<MobileManipulatorInterface> interfacePtr;
  vector_t initialStateOverride;
  if (!urdfFile.empty() && !libFolder.empty()) {
    try {
      interfacePtr = std::make_unique<MobileManipulatorInterface>(taskFile, libFolder, urdfFile);
      initialStateOverride = loadInitialStateFromYaml(initialPoseFile, interfacePtr->getManipulatorModelInfo(),
                                                      interfacePtr->getInitialState(), node->get_logger());
    } catch (const std::exception& e) {
      RCLCPP_WARN(node->get_logger(), "Failed to create MobileManipulatorInterface: %s", e.what());
    }
  }

  RCLCPP_INFO(node->get_logger(), "Marker target node started. Publishing to %s_mpc_target at %.1f Hz",
              robotName.c_str(), markerPublishRate);

  auto getInitialStateForFk = [&]() -> vector_t {
    if (!interfacePtr) {
      return {};
    }
    if (static_cast<size_t>(initialStateOverride.size()) == interfacePtr->getManipulatorModelInfo().stateDim) {
      return initialStateOverride;
    }
    return interfacePtr->getInitialState();
  };

  if (dualArmMode) {
    SystemObservation latestObs;
    bool haveObs = false;
    bool markersInitialized = false;

    RCLCPP_INFO(node->get_logger(), "Dual arm mode enabled - creating dual arm interactive markers");
    UnifiedTargetTrajectoriesInteractiveMarker targetPoseCommand(
        node, robotName, &dualArmGoalPoseToTargetTrajectories, markerPublishRate, markerGlobalFrame);

    auto obsSub = node->create_subscription<ocs2_msgs::msg::MpcObservation>(
        robotName + std::string("_mpc_observation"), 1,
        [&](const ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
          latestObs = ros_msg_conversions::readObservationMsg(*msg);
          haveObs = true;

          if (!interfacePtr || markersInitialized) {
            return;
          }

          try {
            const auto& pin = interfacePtr->getPinocchioInterface();
            const auto& model = pin.getModel();
            auto data = pin.getData();
            pinocchio::forwardKinematics(model, data, latestObs.state);
            pinocchio::updateFramePlacements(model, data);

            const auto& info = interfacePtr->getManipulatorModelInfo();
            const auto left_id = model.getFrameId(info.eeFrame);
            const auto& left = data.oMf[left_id];
            Eigen::Vector3d lp = left.translation();
            Eigen::Quaterniond lq(left.rotation());
            targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::LEFT, lp, lq);
            targetPoseCommand.updateMarkerDisplay("LeftArmGoal", lp, lq);

            const auto right_id = model.getFrameId(info.eeFrame1);
            const auto& right = data.oMf[right_id];
            Eigen::Vector3d rp = right.translation();
            Eigen::Quaterniond rq(right.rotation());
            targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::RIGHT, rp, rq);
            targetPoseCommand.updateMarkerDisplay("RightArmGoal", rp, rq);
            markersInitialized = true;
            RCLCPP_INFO(node->get_logger(), "Initialized dual-arm markers from first MPC observation.");
          } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "FK init for markers from observation failed: %s", e.what());
          }
        });

    auto srv = node->create_service<std_srvs::srv::SetBool>(
        "toggle_mpc",
        [&](const std::shared_ptr<rmw_request_id_t> /*req_header*/, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          if (!req->data) {
            try {
              Eigen::Vector3d lp, rp;
              Eigen::Quaterniond lq, rq;
              if (interfacePtr && haveObs) {
                const auto& pin = interfacePtr->getPinocchioInterface();
                const auto& model = pin.getModel();
                auto data = pin.getData();
                pinocchio::forwardKinematics(model, data, latestObs.state);
                pinocchio::updateFramePlacements(model, data);
                const auto& info = interfacePtr->getManipulatorModelInfo();
                const auto left_id = model.getFrameId(info.eeFrame);
                const auto& left = data.oMf[left_id];
                lp = left.translation();
                lq = Eigen::Quaterniond(left.rotation());
                const auto right_id = model.getFrameId(info.eeFrame1);
                const auto& right = data.oMf[right_id];
                rp = right.translation();
                rq = Eigen::Quaterniond(right.rotation());
              } else {
                std::tie(lp, lq) = targetPoseCommand.getDualArmPose(ocs2::IMarkerControl::ArmType::LEFT);
                std::tie(rp, rq) = targetPoseCommand.getDualArmPose(ocs2::IMarkerControl::ArmType::RIGHT);
              }

              if (targetPoseCommand.isContinuousMode()) {
                targetPoseCommand.togglePublishMode();
              }
              targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::LEFT, lp, lq);
              targetPoseCommand.setDualArmPose(ocs2::IMarkerControl::ArmType::RIGHT, rp, rq);
              targetPoseCommand.updateMarkerDisplay("LeftArmGoal", lp, lq);
              targetPoseCommand.updateMarkerDisplay("RightArmGoal", rp, rq);
              targetPoseCommand.sendDualArmTrajectories();
              res->success = true;
              res->message = "MPC paused; holding current poses";
            } catch (const std::exception& e) {
              res->success = false;
              res->message = std::string("Hold failed: ") + e.what();
            }
          } else {
            if (!targetPoseCommand.isContinuousMode()) {
              targetPoseCommand.togglePublishMode();
            }
            res->success = true;
            res->message = "MPC started (continuous mode)";
          }
        });

    if (enableJoystick) {
      RCLCPP_INFO(node->get_logger(), "Joystick marker wrapper enabled");
      auto joystickControl = std::make_unique<JoystickMarkerWrapper>(node, &targetPoseCommand);
      (void)joystickControl;
    }

    if (enableAutoPosition) {
      RCLCPP_INFO(node->get_logger(), "Marker auto position wrapper enabled");
      auto autoPositionWrapper = std::make_unique<MarkerAutoPositionWrapper>(
          node, robotName, &targetPoseCommand, MarkerAutoPositionWrapper::UpdateMode::CONTINUOUS, dualArmMode);
      (void)autoPositionWrapper;
    }

    rclcpp::spin(node);
    return 0;
  }

  SystemObservation latestObs;
  bool haveObs = false;
  bool markerInitialized = false;

  RCLCPP_INFO(node->get_logger(), "Single arm mode enabled");
  UnifiedTargetTrajectoriesInteractiveMarker targetPoseCommand(
      node, robotName, &goalPoseToTargetTrajectories, markerPublishRate, markerGlobalFrame);

  auto obsSub = node->create_subscription<ocs2_msgs::msg::MpcObservation>(
      robotName + std::string("_mpc_observation"), 1,
      [&](const ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
        latestObs = ros_msg_conversions::readObservationMsg(*msg);
        haveObs = true;

        if (!interfacePtr || markerInitialized) {
          return;
        }

        try {
          const auto& pin = interfacePtr->getPinocchioInterface();
          const auto& model = pin.getModel();
          auto data = pin.getData();
          pinocchio::forwardKinematics(model, data, latestObs.state);
          pinocchio::updateFramePlacements(model, data);

          const auto& info = interfacePtr->getManipulatorModelInfo();
          const auto ee_id = model.getFrameId(info.eeFrame);
          const auto& ee = data.oMf[ee_id];
          Eigen::Vector3d p = ee.translation();
          Eigen::Quaterniond q(ee.rotation());
          targetPoseCommand.setSingleArmPose(p, q);
          targetPoseCommand.updateMarkerDisplay("Goal", p, q);
          markerInitialized = true;
          RCLCPP_INFO(node->get_logger(), "Initialized marker from first MPC observation.");
        } catch (const std::exception& e) {
          RCLCPP_WARN(node->get_logger(), "FK init for marker from observation failed: %s", e.what());
        }
      });

  auto srv = node->create_service<std_srvs::srv::SetBool>(
      "toggle_mpc",
      [&](const std::shared_ptr<rmw_request_id_t> /*req_header*/, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
          std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        if (!req->data) {
          try {
            Eigen::Vector3d p;
            Eigen::Quaterniond q;
            if (interfacePtr && haveObs) {
              const auto& pin = interfacePtr->getPinocchioInterface();
              const auto& model = pin.getModel();
              auto data = pin.getData();
              pinocchio::forwardKinematics(model, data, latestObs.state);
              pinocchio::updateFramePlacements(model, data);
              const auto& info = interfacePtr->getManipulatorModelInfo();
              const auto ee_id = model.getFrameId(info.eeFrame);
              const auto& ee = data.oMf[ee_id];
              p = ee.translation();
              q = Eigen::Quaterniond(ee.rotation());
            } else {
              std::tie(p, q) = targetPoseCommand.getSingleArmPose();
            }

            if (targetPoseCommand.isContinuousMode()) {
              targetPoseCommand.togglePublishMode();
            }
            targetPoseCommand.setSingleArmPose(p, q);
            targetPoseCommand.updateMarkerDisplay("Goal", p, q);
            targetPoseCommand.sendSingleArmTrajectories();
            res->success = true;
            res->message = "MPC paused; holding current pose";
          } catch (const std::exception& e) {
            res->success = false;
            res->message = std::string("Hold failed: ") + e.what();
          }
        } else {
          if (!targetPoseCommand.isContinuousMode()) {
            targetPoseCommand.togglePublishMode();
          }
          res->success = true;
          res->message = "MPC started (continuous mode)";
        }
      });

  if (enableJoystick) {
    RCLCPP_INFO(node->get_logger(), "Joystick marker wrapper enabled");
    auto joystickControl = std::make_unique<JoystickMarkerWrapper>(node, &targetPoseCommand);
    (void)joystickControl;
  }

  if (enableAutoPosition) {
    RCLCPP_INFO(node->get_logger(), "Marker auto position wrapper enabled");
    auto autoPositionWrapper = std::make_unique<MarkerAutoPositionWrapper>(
        node, robotName, &targetPoseCommand, MarkerAutoPositionWrapper::UpdateMode::CONTINUOUS, dualArmMode);
    (void)autoPositionWrapper;
  }

  rclcpp::spin(node);
  return 0;
}
