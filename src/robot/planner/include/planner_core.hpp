#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

struct CellIndex {
  int x = 0;
  int y = 0;
  CellIndex() = default;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct PlannerConfig {
  int lethal_cost = 55;
  double cost_weight = 4.0;
  bool allow_unknown = true;
  double goal_search_radius = 3.0;
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(const PlannerConfig& config);

    bool plan(const nav_msgs::msg::OccupancyGrid& map, const Point2D& start, const Point2D& goal,
              std::vector<Point2D>& path, std::string& error);

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, const Point2D& p, CellIndex& c) const;
    Point2D gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const;

    bool traversable(int idx, bool allow_escape = true) const;
    double stepMultiplier(int idx) const;
    bool findNearestTraversable(const CellIndex& from, int max_cells, CellIndex& out) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    bool runAStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& cells);

    rclcpp::Logger logger_;
    PlannerConfig config_;

    const std::vector<int8_t>* data_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    int escape_cost_ = 0;

    std::vector<double> g_score_;
    std::vector<int> came_from_;
    std::vector<uint8_t> closed_;
};

}

#endif
