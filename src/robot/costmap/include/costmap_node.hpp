#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    void lidarScanner(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  private:
    robot::CostmapCore costmap_;
    
    // Member variables matching your .cpp file
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr string_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};
 
#endif
