#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(this->get_logger()) {
  robot::MapMemoryConfig config;
  config.frame_id = this->declare_parameter<std::string>("frame_id", config.frame_id);
  config.resolution = this->declare_parameter<double>("resolution", config.resolution);
  config.width = this->declare_parameter<double>("width", config.width);
  config.height = this->declare_parameter<double>("height", config.height);
  config.origin_x = this->declare_parameter<double>("origin_x", config.origin_x);
  config.origin_y = this->declare_parameter<double>("origin_y", config.origin_y);
  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);
  const double update_period = this->declare_parameter<double>("update_period", 1.0);
  map_memory_.configure(config);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable());
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(update_period), std::bind(&MapMemoryNode::timerCallback, this));

  publishMap();
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  robot::Pose2D pose;
  if (!poseAt(rclcpp::Time(msg->header.stamp), pose)) {
    return;
  }
  latest_costmap_ = msg;
  costmap_pose_ = pose;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot::Pose2D pose;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.yaw = yawFromQuaternion(msg->pose.pose.orientation);
  const rclcpp::Time stamp(msg->header.stamp);

  if (!odom_history_.empty() && stamp < odom_history_.back().first) {
    odom_history_.clear();
  }
  odom_history_.emplace_back(stamp, pose);
  while (odom_history_.size() > 1 &&
         (stamp - odom_history_.front().first).seconds() > 2.0) {
    odom_history_.pop_front();
  }
}

bool MapMemoryNode::poseAt(const rclcpp::Time& stamp, robot::Pose2D& pose) const {
  if (odom_history_.empty()) {
    return false;
  }
  if (stamp <= odom_history_.front().first) {
    pose = odom_history_.front().second;
    return true;
  }
  if (stamp >= odom_history_.back().first) {
    pose = odom_history_.back().second;
    return true;
  }
  for (size_t i = 1; i < odom_history_.size(); ++i) {
    const auto& [t1, p1] = odom_history_[i];
    if (t1 < stamp) {
      continue;
    }
    const auto& [t0, p0] = odom_history_[i - 1];
    const double span = (t1 - t0).seconds();
    const double a = span > 0.0 ? (stamp - t0).seconds() / span : 0.0;
    const double dyaw = std::atan2(std::sin(p1.yaw - p0.yaw), std::cos(p1.yaw - p0.yaw));
    pose.x = p0.x + a * (p1.x - p0.x);
    pose.y = p0.y + a * (p1.y - p0.y);
    pose.yaw = p0.yaw + a * dyaw;
    return true;
  }
  pose = odom_history_.back().second;
  return true;
}

void MapMemoryNode::timerCallback() {
  if (!latest_costmap_) {
    return;
  }

  const double moved = std::hypot(costmap_pose_.x - last_fuse_x_, costmap_pose_.y - last_fuse_y_);
  if (has_fused_ && moved < update_distance_) {
    return;
  }

  map_memory_.fuse(*latest_costmap_, costmap_pose_);
  last_fuse_x_ = costmap_pose_.x;
  last_fuse_y_ = costmap_pose_.y;
  has_fused_ = true;
  publishMap();
}

void MapMemoryNode::publishMap() {
  nav_msgs::msg::OccupancyGrid msg = map_memory_.map();
  msg.header.stamp = this->now();
  map_pub_->publish(msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
