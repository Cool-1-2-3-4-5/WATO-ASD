#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

struct ControlConfig {
  double lookahead_distance = 1.5;
  double linear_speed = 0.8;
  double min_linear_speed = 0.15;
  double max_angular_speed = 1.2;
  double goal_tolerance = 0.3;
  double slow_down_distance = 1.5;
  double rotate_in_place_angle = 1.0;
};

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void configure(const ControlConfig& config);

    void setPath(const nav_msgs::msg::Path& path);
    void clearPath();
    bool hasPath() const { return !path_.poses.empty(); }

    bool computeCommand(double x, double y, double yaw, geometry_msgs::msg::Twist& cmd);

  private:
    size_t updateClosestIndex(double x, double y);
    size_t findLookaheadIndex(size_t from, double x, double y) const;

    rclcpp::Logger logger_;
    ControlConfig config_;
    nav_msgs::msg::Path path_;
    size_t closest_index_ = 0;
};

}

#endif
