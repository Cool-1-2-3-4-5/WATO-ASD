#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

// Tunable settings for building the local costmap.
struct CostmapConfig {
  double resolution = 0.1;        // metres per cell
  double size = 40.0;             // side length of the square grid (metres), centred on the sensor
  double inflation_radius = 2.0;  // metres around an obstacle that receive a cost
  int max_cost = 100;             // cost of an obstacle cell
  double max_range = 20.0;        // ignore returns further than this (metres)
};

// Converts a LaserScan (in the sensor frame) into an inflated costmap.
//
// Cell values follow nav_msgs/OccupancyGrid conventions:
//   -1      : unknown (never observed by any ray in this scan)
//    0      : observed free space
//   1..99   : observed free space, with inflation cost from a nearby obstacle
//   100     : obstacle
class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void configure(const CostmapConfig& config);

    // Builds a costmap from a scan. The resulting grid is expressed in the scan's frame,
    // with the sensor at the centre of the grid. Returns false (leaving `out` untouched) for scans
    // with no returns at all: the simulated lidar emits a few of these while it starts up, and
    // treating them as "free out to max range" would wipe obstacles from the map.
    bool build(const sensor_msgs::msg::LaserScan& scan, nav_msgs::msg::OccupancyGrid& out);

  private:
    void classifyCells(const sensor_msgs::msg::LaserScan& scan, double usable_max);
    void inflate();

    rclcpp::Logger logger_;
    CostmapConfig config_;

    int width_ = 0;
    double origin_ = 0.0;  // origin is (origin_, origin_) in the sensor frame
    std::vector<int8_t> grid_;
    std::vector<int> obstacle_cells_;  // flat indices of obstacle cells in the current scan
    std::vector<double> inflation_kernel_;  // precomputed cost multipliers for a (2r+1)^2 window
    int kernel_radius_ = 0;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
