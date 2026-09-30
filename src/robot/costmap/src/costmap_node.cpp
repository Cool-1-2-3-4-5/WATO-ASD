#include "costmap_node.hpp"

#include <memory>

CostmapNode::CostmapNode() : Node("costmap"), costmap_(this->get_logger()) {
  robot::CostmapConfig config;
  config.resolution = this->declare_parameter<double>("resolution", config.resolution);
  config.size = this->declare_parameter<double>("size", config.size);
  config.inflation_radius = this->declare_parameter<double>("inflation_radius", config.inflation_radius);
  config.max_cost = this->declare_parameter<int>("max_cost", config.max_cost);
  config.max_range = this->declare_parameter<double>("max_range", config.max_range);
  costmap_.configure(config);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  nav_msgs::msg::OccupancyGrid grid;
  if (!costmap_.build(*msg, grid)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Ignoring lidar scan with no returns (sensor still starting up?)");
    return;
  }
  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
