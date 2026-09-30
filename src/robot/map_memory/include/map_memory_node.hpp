#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <deque>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

// Stitches local costmaps (/costmap) into a global map (/map) using odometry (/odom/filtered).
// The map is only re-fused once the robot has travelled `update_distance` metres, and the check
// runs on a timer so that fusion never happens more often than `update_period` seconds.
class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publishMap();

    robot::MapMemoryCore map_memory_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot pose at time `stamp`, interpolated from recent odometry.
    bool poseAt(const rclcpp::Time& stamp, robot::Pose2D& pose) const;

    // Latest costmap together with the robot pose at the time the scan was taken.
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    robot::Pose2D costmap_pose_;

    // Short odometry history so each scan can be matched with the pose it was taken from.
    // Using just the latest odom smears the map badly while the robot is turning.
    std::deque<std::pair<rclcpp::Time, robot::Pose2D>> odom_history_;

    bool has_fused_ = false;
    double last_fuse_x_ = 0.0;
    double last_fuse_y_ = 0.0;
    double update_distance_;
};

#endif  // MAP_MEMORY_NODE_HPP_
