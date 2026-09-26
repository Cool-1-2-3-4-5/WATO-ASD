#include "planner_node.hpp"
#include <cmath>
#include <queue>
#include <algorithm>

using namespace std::chrono_literals;

PlannerNode::PlannerNode() : Node("planner_node") {
  state_ = 0; 

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_callback(msg); });
      
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, [this](geometry_msgs::msg::PointStamped::SharedPtr msg) { goal_callback(msg); });
      
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  timer_ = this->create_wall_timer(
      500ms, [this]() { timer_callback(); });
}

void PlannerNode::map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = msg;
}

void PlannerNode::goal_callback(geometry_msgs::msg::PointStamped::SharedPtr msg) {
  current_goal_ = msg;
  state_ = 1;
  plan_path();
}

void PlannerNode::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = msg;
}

void PlannerNode::timer_callback() {
  if (state_ == 1 && current_goal_ && current_odom_) {
    double dx = current_goal_->point.x - current_odom_->pose.pose.position.x;
    double dy = current_goal_->point.y - current_odom_->pose.pose.position.y;
    double dist = std::sqrt((dx * dx) + (dy * dy));
    
    if (dist < 0.5) {
      state_ = 0; 
    } else {
      plan_path();
    }
  }
}

void PlannerNode::plan_path() {
  if (!current_map_ || !current_goal_ || !current_odom_) {
    return;
  }

  int width = current_map_->info.width;
  int height = current_map_->info.height;
  double res = current_map_->info.resolution;
  double origin_x = current_map_->info.origin.position.x;
  double origin_y = current_map_->info.origin.position.y;

  int start_x = std::round((current_odom_->pose.pose.position.x - origin_x) / res);
  int start_y = std::round((current_odom_->pose.pose.position.y - origin_y) / res);
  
  int goal_x = std::round((current_goal_->point.x - origin_x) / res);
  int goal_y = std::round((current_goal_->point.y - origin_y) / res);

  if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
      goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
    return;
  }

  int start_idx = (start_y * width) + start_x;
  int goal_idx = (goal_y * width) + goal_x;

  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> open_set;
  
  int total_cells = width * height;
  std::vector<double> g_score(total_cells, 999999.0);
  std::vector<int> came_from(total_cells, -1);

  g_score[start_idx] = 0.0;
  
  double start_dx = goal_x - start_x;
  double start_dy = goal_y - start_y;
  double start_h = std::sqrt((start_dx * start_dx) + (start_dy * start_dy));
  
  open_set.push({start_h, start_idx});

  int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
  double step_cost[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

  int current_idx = -1;

  while (!open_set.empty()) {
    current_idx = open_set.top().second;
    open_set.pop();

    if (current_idx == goal_idx) {
      break;
    }

    int cx = current_idx % width;
    int cy = current_idx / width;

    for (int i = 0; i < 8; i++) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];

      if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        int neighbor_idx = (ny * width) + nx;
        int map_val = current_map_->data[neighbor_idx];

        if (map_val >= 0 && map_val <= 50) {
          double penalty = map_val / 100.0;
          double tentative_g = g_score[current_idx] + step_cost[i] + penalty;

          if (tentative_g < g_score[neighbor_idx]) {
            came_from[neighbor_idx] = current_idx;
            g_score[neighbor_idx] = tentative_g;
            
            double hx = goal_x - nx;
            double hy = goal_y - ny;
            double h = std::sqrt((hx * hx) + (hy * hy));
            
            open_set.push({tentative_g + h, neighbor_idx});
          }
        }
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "map";

  if (current_idx == goal_idx) {
    int curr = goal_idx;
    while (curr != start_idx && curr != -1) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      
      int mx = curr % width;
      int my = curr / width;
      pose.pose.position.x = (mx * res) + origin_x;
      pose.pose.position.y = (my * res) + origin_y;
      
      path.poses.push_back(pose);
      curr = came_from[curr];
    }
    std::reverse(path.poses.begin(), path.poses.end());
  }

  path_pub_->publish(path);
}