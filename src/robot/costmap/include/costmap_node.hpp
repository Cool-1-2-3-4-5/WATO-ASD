#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

// Subscribes to /lidar and publishes an inflated local costmap on /costmap.
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

#endif  // COSTMAP_NODE_HPP_
