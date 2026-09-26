#include <chrono>
#include <memory>
#include <cmath> 
#include <vector>
#include <utility>
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  string_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "/lidar", 10, std::bind(&CostmapNode::lidarScanner, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);
}

void CostmapNode::lidarScanner(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Initialize grid
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = msg->header;
  
  grid.info.resolution = 0.1; 
  grid.info.width = 100;
  grid.info.height = 100;

  grid.info.origin.position.x = -5;
  grid.info.origin.position.y = -5;
  grid.info.origin.position.z = 0;
  grid.info.origin.orientation.w = 1;

  int total_length = grid.info.width * grid.info.height;
  grid.data.assign(total_length, 0);
  
  std::vector<std::pair<int, int>> obstacle_cells;

  for (size_t i = 0; i < msg->ranges.size(); i++) {
    double range = msg->ranges[i];
    
    if (range >= msg->range_min && range <= msg->range_max) {
      double angle = msg->angle_min + (i * msg->angle_increment);
      double x = range * cos(angle);
      double y = range * sin(angle);
  
      int grid_x = (x - grid.info.origin.position.x) / grid.info.resolution;
      int grid_y = (y - grid.info.origin.position.y) / grid.info.resolution;
      
      int index = grid_y * grid.info.width + grid_x;
      
      if (grid_x >= 0 && grid_x < static_cast<int>(grid.info.width) && grid_y >= 0 && grid_y < static_cast<int>(grid.info.height)) {
        grid.data[index] = 100;
        obstacle_cells.push_back({grid_x, grid_y});
      }
    }
  }
  
  double inflation_radius_m = 1.0; 
  int max_cost = 100;
  
  int inflation_cells = ceil(inflation_radius_m / grid.info.resolution);
  
  for (const auto& obs : obstacle_cells) {
    int obs_x = obs.first;
    int obs_y = obs.second;
  
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        
        int nx = obs_x + dx;
        int ny = obs_y + dy;
        
        if (nx >= 0 && nx < static_cast<int>(grid.info.width) && 
            ny >= 0 && ny < static_cast<int>(grid.info.height)) {
            
          double distance_m = sqrt(dx*dx + dy*dy) * grid.info.resolution;
          
          if (distance_m <= inflation_radius_m) {
            int new_cost = max_cost * (1.0 - (distance_m / inflation_radius_m));
            
            int n_index = ny * grid.info.width + nx;
            
            if (new_cost > grid.data[n_index]) {
              grid.data[n_index] = new_cost;
            }
          }
        }
      }
    }
  }
  costmap_pub_->publish(grid);
}