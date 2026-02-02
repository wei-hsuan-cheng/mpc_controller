#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <mobile_manipulator_mpc/MobileManipulatorInterface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include "mpc_controller/mpc/MobileManipulatorRosReferenceManager.h"

using ocs2::GaussNewtonDDP_MPC;
using ocs2::MPC_ROS_Interface;
using ocs2::SystemObservation;
using ocs2::mobile_manipulator_mpc::MobileManipulatorInterface;

namespace {

/**
 * Wrapper around MPC_ROS_Interface to inject the latest observation into a custom
 * RosReferenceManager without adding a second observation subscriber.
 *
 * Implementation:
 * - We re-implement launchNodes() to bind the observation subscription to our hook.
 * - The hook forwards observation to the reference manager, then calls base MPC_ROS_Interface callback.
 */
class MobileManipulatorMpcRosInterface final : public MPC_ROS_Interface {
 public:
  MobileManipulatorMpcRosInterface(ocs2::MPC_BASE& mpc,
                                  std::string topicPrefix,
                                  std::shared_ptr<mpc_controller::MobileManipulatorRosReferenceManager> rm)
      : MPC_ROS_Interface(mpc, std::move(topicPrefix)), rm_(std::move(rm)) {}

  void launchNodes(const rclcpp::Node::SharedPtr& node) {
    RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorMpcRosInterface"), "MPC node is setting up ...");
    node_ = node;

    // Observation subscriber (single subscriber for the whole system)
    mpcObservationSubscriber_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
        topicPrefix_ + "_mpc_observation",
        1,
        std::bind(&MobileManipulatorMpcRosInterface::observationHook, this, std::placeholders::_1));

    // MPC policy publisher
    mpcPolicyPublisher_ = node_->create_publisher<ocs2_msgs::msg::MpcFlattenedController>(
        topicPrefix_ + "_mpc_policy", 1);

    // Reset service
    mpcResetServiceServer_ = node_->create_service<ocs2_msgs::srv::Reset>(
        topicPrefix_ + "_mpc_reset",
        [this](const std::shared_ptr<ocs2_msgs::srv::Reset::Request>& request,
               const std::shared_ptr<ocs2_msgs::srv::Reset::Response>& response) {
          return resetMpcCallback(request, response);
        });

    RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorMpcRosInterface"), "MPC node is ready.");
    spin();
  }

 private:
  void observationHook(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
    // 1) Convert and inject observation into reference manager
    if (rm_) {
      const SystemObservation obs = ocs2::ros_msg_conversions::readObservationMsg(*msg);
      rm_->setLatestObservation(obs);
    }

    // 2) Run MPC using the original MPC_ROS_Interface logic
    MPC_ROS_Interface::mpcObservationCallback(msg);
  }

 private:
  std::shared_ptr<mpc_controller::MobileManipulatorRosReferenceManager> rm_;
};

}  // namespace

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      robotName + "_mpc",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  const std::string taskFile = node->get_parameter("taskFile").as_string();
  const std::string libFolder = node->get_parameter("libFolder").as_string();
  const std::string urdfFile = node->get_parameter("urdfFile").as_string();

  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;

  MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);

  // Custom reference manager (no observation subscription)
  auto rosReferenceManagerPtr =
      std::make_shared<mpc_controller::MobileManipulatorRosReferenceManager>(
          node,
          interface.getReferenceManagerPtr(),
          robotName,
          interface.getPinocchioInterface(),
          interface.getManipulatorModelInfo());

  GaussNewtonDDP_MPC mpc(interface.mpcSettings(),
                         interface.ddpSettings(),
                         interface.getRollout(),
                         interface.getOptimalControlProblem(),
                         interface.getInitializer());

  // Make sure MPC uses the decorated reference manager
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Use wrapper MPC ROS interface so we can inject observation without extra subscription
  MobileManipulatorMpcRosInterface mpcNode(mpc, robotName, rosReferenceManagerPtr);
  mpcNode.launchNodes(node);

  return 0;
}
