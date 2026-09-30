#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

struct MapMemoryConfig {
  std::string frame_id = "sim_world";
  double resolution = 0.1;  // metres per cell
  double width = 40.0;      // metres
  double height = 40.0;     // metres
  double origin_x = -20.0;  // world position of cell (0, 0)
  double origin_y = -20.0;
};

// 2D pose of the costmap's frame expressed in the global map frame.
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Holds the global map and fuses local costmaps into it.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void configure(const MapMemoryConfig& config);

    // Fuses a local costmap whose frame sits at `pose` in the global frame.
    // Known cells in the costmap overwrite the global map; unknown cells keep the old value.
    void fuse(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& pose);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
