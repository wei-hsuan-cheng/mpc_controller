#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mpc_controller
{

class FakeBaseOdomNode final : public rclcpp::Node
{
public:
  FakeBaseOdomNode()
  : rclcpp::Node("fake_base_odom")
  {
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = this->declare_parameter<std::string>("child_frame_id", "base_link");

    const double publish_rate = this->declare_parameter<double>("publish_rate", 100.0);
    x_ = this->declare_parameter<double>("x0", 0.0);
    y_ = this->declare_parameter<double>("y0", 0.0);
    yaw_ = this->declare_parameter<double>("yaw0", 0.0);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cmd_ = *msg;
      });

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SystemDefaultsQoS());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_update_time_ = this->now();

    const auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate));
    timer_ = this->create_wall_timer(period, [this]() { this->updateAndPublish(); });

    RCLCPP_INFO(
      this->get_logger(),
      "[FakeBaseOdomNode] cmd_vel_topic=%s odom_topic=%s frame_id=%s child_frame_id=%s rate=%.2f", 
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), frame_id_.c_str(), child_frame_id_.c_str(), publish_rate);
  }

private:
  static double normalizeYaw(double yaw)
  {
    while (yaw > M_PI) {
      yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
      yaw += 2.0 * M_PI;
    }
    return yaw;
  }

  void updateAndPublish()
  {
    const auto now = this->now();
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    geometry_msgs::msg::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      cmd = last_cmd_;
    }

    const double v = cmd.linear.x;
    const double omega = cmd.angular.z;

    x_ += v * std::cos(yaw_) * dt;
    y_ += v * std::sin(yaw_) * dt;
    yaw_ = normalizeYaw(yaw_ + omega * dt);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = frame_id_;
    tf.child_frame_id = child_frame_id_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = std::sin(yaw_ * 0.5);
    tf.transform.rotation.w = std::cos(yaw_ * 0.5);
    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(tf);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

    odom.twist.twist = cmd;

    odom_pub_->publish(odom);
  }

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist last_cmd_;

  rclcpp::Time last_update_time_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpc_controller::FakeBaseOdomNode>());
  rclcpp::shutdown();
  return 0;
}
