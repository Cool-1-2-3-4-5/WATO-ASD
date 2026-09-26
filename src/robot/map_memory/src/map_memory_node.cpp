#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;
using namespace std::chrono_literals;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), has_initialized_pose_(false), last_update_x_(0.0), last_update_y_(0.0) {

  global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  initGlobalMap();

  timer_ = this->create_wall_timer(
    1s, std::bind(&MapMemoryNode::timerCallback, this)
  );
}

void MapMemoryNode::initGlobalMap() {
  global_map_.header.frame_id = "map";
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 500;
  global_map_.info.height = 500;
  
  global_map_.info.origin.position.x = -25.0; 
  global_map_.info.origin.position.y = -25.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  int total_cells = global_map_.info.width * global_map_.info.height;
  global_map_.data.assign(total_cells, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = msg;
}

void MapMemoryNode::timerCallback() {
  if (!latest_costmap_ || !latest_odom_) {
    return;
  }

  double current_x = latest_odom_->pose.pose.position.x;
  double current_y = latest_odom_->pose.pose.position.y;

  if (!has_initialized_pose_) {
    last_update_x_ = current_x;
    last_update_y_ = current_y;
    has_initialized_pose_ = true;
    fuseCostmap();
    return;
  }

  double distance = sqrt(pow(current_x - last_update_x_, 2) + pow(current_y - last_update_y_, 2));

  double update_thres = 1.5; 

  if (distance >= update_thres) {
    last_update_x_ = current_x;
    last_update_y_ = current_y;
    fuseCostmap();
  }
}

void MapMemoryNode::fuseCostmap() {
  double current_x = latest_odom_->pose.pose.position.x;
  double current_y = latest_odom_->pose.pose.position.y;
  double yaw = tf2::getYaw(latest_odom_->pose.pose.orientation);

  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);

  for (unsigned int ly = 0; ly < latest_costmap_->info.height; ++ly) {
    for (unsigned int lx = 0; lx < latest_costmap_->info.width; ++lx) {
    
      int local_index = ly * latest_costmap_->info.width + lx;
      int local_cost = latest_costmap_->data[local_index];
      
      if (local_cost != -1) {
        double local_px = (lx * latest_costmap_->info.resolution) + latest_costmap_->info.origin.position.x;
        double local_py = (ly * latest_costmap_->info.resolution) + latest_costmap_->info.origin.position.y;

        double global_px = (cos_yaw * local_px) - (sin_yaw * local_py) + current_x;
        double global_py = (sin_yaw * local_px) + (cos_yaw * local_py) + current_y;

        int gx = (global_px - global_map_.info.origin.position.x) / global_map_.info.resolution;
        int gy = (global_py - global_map_.info.origin.position.y) / global_map_.info.resolution;

        if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) && gy >= 0 && gy < static_cast<int>(global_map_.info.height)) {
            
            int global_index = gy * global_map_.info.width + gx;
            global_map_.data[global_index] = local_cost;
        }
      }
    }
  }
  global_map_.header.stamp = this->now();
  global_map_pub_->publish(global_map_);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}