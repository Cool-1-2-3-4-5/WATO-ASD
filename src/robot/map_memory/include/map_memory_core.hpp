#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

struct MapMemoryConfig {
  std::string frame_id = "sim_world";
  double resolution = 0.1;
  double width = 40.0;
  double height = 40.0;
  double origin_x = -20.0;
  double origin_y = -20.0;
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void configure(const MapMemoryConfig& config);

    void fuse(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& pose);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}

#endif
