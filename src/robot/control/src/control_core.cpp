#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

void ControlCore::configure(const ControlConfig& config) {
  config_ = config;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path) {
  path_ = path;
  closest_index_ = 0;
}

void ControlCore::clearPath() {
  path_.poses.clear();
  closest_index_ = 0;
}

// Advances the progress marker to the closest path pose. Searching only forward from the previous
// marker stops the robot from latching onto an earlier part of the path that happens to be near.
size_t ControlCore::updateClosestIndex(double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  size_t best_i = closest_index_;
  for (size_t i = closest_index_; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    const double d = std::hypot(p.x - x, p.y - y);
    if (d < best) {
      best = d;
      best_i = i;
    }
  }
  closest_index_ = best_i;
  return best_i;
}

size_t ControlCore::findLookaheadIndex(size_t from, double x, double y) const {
  for (size_t i = from; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    if (std::hypot(p.x - x, p.y - y) >= config_.lookahead_distance) {
      return i;
    }
  }
  return path_.poses.size() - 1;  // near the end: aim straight for the goal
}

bool ControlCore::computeCommand(double x, double y, double yaw, geometry_msgs::msg::Twist& cmd) {
  cmd = geometry_msgs::msg::Twist();
  if (path_.poses.empty()) {
    return false;
  }

  const auto& goal = path_.poses.back().pose.position;
  const double dist_to_goal = std::hypot(goal.x - x, goal.y - y);
  if (dist_to_goal < config_.goal_tolerance) {
    RCLCPP_INFO(logger_, "Reached end of path (%.2fm away), stopping", dist_to_goal);
    clearPath();
    return false;
  }

  const size_t closest = updateClosestIndex(x, y);
  const auto& target = path_.poses[findLookaheadIndex(closest, x, y)].pose.position;

  // Target point in the robot frame (x forward, y left).
  const double dx = target.x - x;
  const double dy = target.y - y;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double local_x = c * dx + s * dy;
  const double local_y = -s * dx + c * dy;
  const double alpha = std::atan2(local_y, local_x);  // heading error to the target

  // Large heading error (e.g. goal behind us): turn on the spot first instead of driving a huge arc.
  if (std::abs(alpha) > config_.rotate_in_place_angle) {
    const double w = std::clamp(1.5 * alpha, -config_.max_angular_speed, config_.max_angular_speed);
    cmd.angular.z = std::copysign(std::max(std::abs(w), 0.3), alpha);
    return true;
  }

  // Pure Pursuit: the arc through the robot and the target has curvature 2*y / L^2.
  const double L2 = local_x * local_x + local_y * local_y;
  const double curvature = L2 > 1e-6 ? 2.0 * local_y / L2 : 0.0;

  // Ease off as we approach the goal so we don't overshoot it.
  const double speed_scale = std::clamp(dist_to_goal / config_.slow_down_distance, 0.0, 1.0);
  double v = std::max(config_.min_linear_speed, config_.linear_speed * speed_scale);
  double w = v * curvature;

  // If the turn would exceed the angular limit, slow down instead so we still follow the same arc.
  if (std::abs(w) > config_.max_angular_speed) {
    w = std::copysign(config_.max_angular_speed, w);
    v = w / curvature;
  }

  cmd.linear.x = v;
  cmd.angular.z = w;
  return true;
}

}  // namespace robot
