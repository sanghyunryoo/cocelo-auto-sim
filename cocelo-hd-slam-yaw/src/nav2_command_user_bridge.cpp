#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "core/msg/command_user.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class Nav2CommandUserBridge : public rclcpp::Node {
 public:
  Nav2CommandUserBridge()
      : Node("nav2_command_user_bridge"), steady_clock_(RCL_STEADY_TIME) {
    input_topic_ = declare_parameter<std::string>("input_twist_topic", "/nav2/cmd_vel");
    output_topic_ =
        declare_parameter<std::string>("output_command_topic", "/control_command/user_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    publish_frequency_ = declare_parameter<double>("publish_frequency", 20.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.50);

    if (input_topic_.empty() || output_topic_.empty() || publish_frequency_ <= 0.0 ||
        command_timeout_ <= 0.0) {
      throw std::runtime_error("Invalid Nav2 CommandUser bridge parameters");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(16)).reliable().durability_volatile();
    command_publisher_ = create_publisher<core::msg::CommandUser>(output_topic_, qos);
    twist_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr message) { onTwist(*message); });

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { publishCommand(); });
    last_publish_time_ = steady_clock_.now();

    RCLCPP_INFO(
        get_logger(),
        "Nav2 command interface: %s (Twist) -> %s (core/msg/CommandUser), %.1f Hz, timeout %.2f s",
        input_topic_.c_str(), output_topic_.c_str(), publish_frequency_, command_timeout_);
  }

 private:
  static double finiteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

  void onTwist(const geometry_msgs::msg::Twist &message) {
    latest_twist_ = message;
    latest_twist_.linear.x = finiteOrZero(latest_twist_.linear.x);
    latest_twist_.linear.y = finiteOrZero(latest_twist_.linear.y);
    latest_twist_.linear.z = finiteOrZero(latest_twist_.linear.z);
    latest_twist_.angular.x = finiteOrZero(latest_twist_.angular.x);
    latest_twist_.angular.y = finiteOrZero(latest_twist_.angular.y);
    latest_twist_.angular.z = finiteOrZero(latest_twist_.angular.z);
    last_command_time_ = steady_clock_.now();
    command_received_ = true;
  }

  void publishCommand() {
    const auto now_steady = steady_clock_.now();
    const double publish_dt = std::clamp(
        (now_steady - last_publish_time_).seconds(), 0.0, 2.0 / publish_frequency_);
    last_publish_time_ = now_steady;

    geometry_msgs::msg::Twist command;
    bool command_active = false;
    if (command_received_) {
      const double age = (now_steady - last_command_time_).seconds();
      command_active = age <= command_timeout_;
      if (command_active) {
        command = latest_twist_;
      }
    }

    const double linear_x = command.linear.x;
    const double linear_y = command.linear.y;
    const double angular_z = command.angular.z;
    yaw_ += angular_z * publish_dt;
    position_x_ +=
        (linear_x * std::cos(yaw_) - linear_y * std::sin(yaw_)) * publish_dt;
    position_y_ +=
        (linear_x * std::sin(yaw_) + linear_y * std::cos(yaw_)) * publish_dt;

    core::msg::CommandUser output;
    output.odom.header.stamp = now();
    output.odom.header.frame_id = odom_frame_;
    output.odom.child_frame_id = child_frame_;
    output.odom.pose.pose.position.x = position_x_;
    output.odom.pose.pose.position.y = position_y_;
    output.odom.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    output.odom.pose.pose.orientation.w = std::cos(0.5 * yaw_);
    output.odom.twist.twist = command;
    output.event.estop = false;
    output.event.wake = false;
    output.event.sleep = false;
    output.event.rough_drive_toggle = false;
    command_publisher_->publish(output);

    if (!command_active && command_received_) {
      RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Nav2 Twist timed out; publishing zero CommandUser command");
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string child_frame_;
  double publish_frequency_{20.0};
  double command_timeout_{0.50};
  geometry_msgs::msg::Twist latest_twist_;
  bool command_received_{false};
  double position_x_{0.0};
  double position_y_{0.0};
  double yaw_{0.0};
  rclcpp::Clock steady_clock_;
  rclcpp::Time last_command_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
  rclcpp::Publisher<core::msg::CommandUser>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2CommandUserBridge>());
  rclcpp::shutdown();
  return 0;
}
