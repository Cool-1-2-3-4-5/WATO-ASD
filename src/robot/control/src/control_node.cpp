#include "control_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

ControlNode::ControlNode() : Node("control"), control_(this->get_logger()) {
  robot::ControlConfig config;
  config.lookahead_distance = this->declare_parameter<double>("lookahead_distance", config.lookahead_distance);
  config.linear_speed = this->declare_parameter<double>("linear_speed", config.linear_speed);
  config.min_linear_speed = this->declare_parameter<double>("min_linear_speed", config.min_linear_speed);
  config.max_angular_speed = this->declare_parameter<double>("max_angular_speed", config.max_angular_speed);
  config.goal_tolerance = this->declare_parameter<double>("goal_tolerance", config.goal_tolerance);
  config.slow_down_distance = this->declare_parameter<double>("slow_down_distance", config.slow_down_distance);
  config.rotate_in_place_angle =
    this->declare_parameter<double>("rotate_in_place_angle", config.rotate_in_place_angle);
  const double rate = this->declare_parameter<double>("control_rate", 10.0);
  control_.configure(config);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate), std::bind(&ControlNode::controlLoop, this));
  last_odom_time_ = this->now();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.empty()) {
    control_.clearPath();
  } else {
    control_.setPath(*msg);
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = msg;
  last_odom_time_ = this->now();
}

void ControlNode::stop() {
  if (moving_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    moving_ = false;
  }
}

void ControlNode::controlLoop() {
  if (!odom_ || !control_.hasPath()) {
    stop();
    return;
  }
  if ((this->now() - last_odom_time_).seconds() > 1.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odometry is stale, stopping");
    stop();
    return;
  }

  const auto& pose = odom_->pose.pose;
  geometry_msgs::msg::Twist cmd;
  if (!control_.computeCommand(pose.position.x, pose.position.y,
                               yawFromQuaternion(pose.orientation), cmd)) {
    stop();
    return;
  }
  cmd_pub_->publish(cmd);
  moving_ = true;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
