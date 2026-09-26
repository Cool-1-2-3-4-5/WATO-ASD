#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goal_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();
  void plan_path();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  geometry_msgs::msg::PointStamped::SharedPtr current_goal_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;

  int state_; 
};

#endif