#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cstddef>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

struct ControlConfig {
  double lookahead_distance = 1.5;  // metres ahead on the path to aim for
  double linear_speed = 0.8;        // cruise speed (m/s)
  double min_linear_speed = 0.15;   // floor while slowing down near the goal (m/s)
  double max_angular_speed = 1.2;   // rad/s
  double goal_tolerance = 0.3;      // stop when this close to the last pose (metres)
  double slow_down_distance = 1.5;  // start slowing down this far from the goal (metres)
  double rotate_in_place_angle = 1.0;  // turn on the spot if the target is further off-axis (rad)
};

// Pure Pursuit path follower for a differential-drive robot.
class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void configure(const ControlConfig& config);

    void setPath(const nav_msgs::msg::Path& path);
    void clearPath();
    bool hasPath() const { return !path_.poses.empty(); }

    // Computes a velocity command for the robot at (x, y, yaw). Returns false (and a zero command)
    // when there is nothing to follow, including once the goal has been reached.
    bool computeCommand(double x, double y, double yaw, geometry_msgs::msg::Twist& cmd);

  private:
    size_t updateClosestIndex(double x, double y);
    size_t findLookaheadIndex(size_t from, double x, double y) const;

    rclcpp::Logger logger_;
    ControlConfig config_;
    nav_msgs::msg::Path path_;
    size_t closest_index_ = 0;  // progress along the path; only ever moves forward
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
