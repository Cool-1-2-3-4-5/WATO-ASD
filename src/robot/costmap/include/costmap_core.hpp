#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

struct CostmapConfig {
  double resolution = 0.1;
  double size = 40.0;
  double inflation_radius = 2.0;
  int max_cost = 100;
  double max_range = 20.0;
};

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void configure(const CostmapConfig& config);

    bool build(const sensor_msgs::msg::LaserScan& scan, nav_msgs::msg::OccupancyGrid& out);

  private:
    void classifyCells(const sensor_msgs::msg::LaserScan& scan, double usable_max);
    void inflate();

    rclcpp::Logger logger_;
    CostmapConfig config_;

    int width_ = 0;
    double origin_ = 0.0;
    std::vector<int8_t> grid_;
    std::vector<int> obstacle_cells_;
    std::vector<double> inflation_kernel_;
    int kernel_radius_ = 0;
};

}

#endif
