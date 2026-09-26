#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void path_callback(nav_msgs::msg::Path::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void control_loop();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path::SharedPtr path_msg_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;
};

#endif