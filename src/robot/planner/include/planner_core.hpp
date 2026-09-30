#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// 2D grid index
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
  int lethal_cost = 55;          // cells with cost >= this are not traversable
  double cost_weight = 4.0;      // how strongly paths are pushed away from inflated areas
  bool allow_unknown = true;     // plan through unexplored (-1) cells
  double goal_search_radius = 3.0;  // metres to search for a free cell if the goal is blocked
};

// A* planner over an OccupancyGrid.
class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(const PlannerConfig& config);

    // Plans from `start` to `goal` (both in the map frame). On success fills `path` with world
    // coordinates, starting at the robot and ending at the goal. If the goal sits inside an
    // obstacle, the nearest free cell is used instead; path.back() is the goal actually planned to.
    bool plan(const nav_msgs::msg::OccupancyGrid& map, const Point2D& start, const Point2D& goal,
              std::vector<Point2D>& path, std::string& error);

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& map, const Point2D& p, CellIndex& c) const;
    Point2D gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const;

    // allow_escape also permits cells no worse than the robot's current cell (see escape_cost_).
    bool traversable(int idx, bool allow_escape = true) const;
    double stepMultiplier(int idx) const;
    bool findNearestTraversable(const CellIndex& from, int max_cells, CellIndex& out) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    bool runAStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& cells);

    rclcpp::Logger logger_;
    PlannerConfig config_;

    // Per-plan working data
    const std::vector<int8_t>* data_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    int escape_cost_ = 0;  // lets the robot plan its way out if it starts inside an inflated area

    std::vector<double> g_score_;
    std::vector<int> came_from_;
    std::vector<uint8_t> closed_;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
