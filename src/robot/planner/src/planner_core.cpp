#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot
{

namespace
{
constexpr double kSqrt2 = 1.41421356237;

struct AStarNode {
  int index;
  double f_score;
  double g_score;
};

struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const { return a.f_score > b.f_score; }
};

const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

void PlannerCore::configure(const PlannerConfig& config) {
  config_ = config;
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid& map, const Point2D& p,
                              CellIndex& c) const {
  c.x = static_cast<int>(std::floor((p.x - map.info.origin.position.x) / map.info.resolution));
  c.y = static_cast<int>(std::floor((p.y - map.info.origin.position.y) / map.info.resolution));
  return c.x >= 0 && c.y >= 0 && c.x < static_cast<int>(map.info.width) &&
         c.y < static_cast<int>(map.info.height);
}

Point2D PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const {
  return {map.info.origin.position.x + (c.x + 0.5) * map.info.resolution,
          map.info.origin.position.y + (c.y + 0.5) * map.info.resolution};
}

bool PlannerCore::traversable(int idx, bool allow_escape) const {
  const int v = (*data_)[idx];
  if (v < 0) {
    return config_.allow_unknown;
  }
  return v < config_.lethal_cost || (allow_escape && v <= escape_cost_);
}

double PlannerCore::stepMultiplier(int idx) const {
  const int v = std::max<int>(0, (*data_)[idx]);
  return 1.0 + config_.cost_weight * (v / 100.0);
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
  const int dx = std::abs(a.x - b.x);
  const int dy = std::abs(a.y - b.y);
  return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

bool PlannerCore::findNearestTraversable(const CellIndex& from, int max_cells, CellIndex& out) const {
  std::vector<uint8_t> seen(static_cast<size_t>(width_) * height_, 0);
  std::queue<CellIndex> q;
  q.push(from);
  seen[from.y * width_ + from.x] = 1;
  while (!q.empty()) {
    const CellIndex c = q.front();
    q.pop();
    if (std::max(std::abs(c.x - from.x), std::abs(c.y - from.y)) > max_cells) {
      continue;
    }
    const int v = (*data_)[c.y * width_ + c.x];
    if (v >= 0 && v < config_.lethal_cost) {
      out = c;
      return true;
    }
    for (int i = 0; i < 4; ++i) {
      const int nx = c.x + kDx[i];
      const int ny = c.y + kDy[i];
      if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
      const int n = ny * width_ + nx;
      if (seen[n]) continue;
      seen[n] = 1;
      q.push({nx, ny});
    }
  }
  return false;
}

bool PlannerCore::runAStar(const CellIndex& start, const CellIndex& goal,
                           std::vector<CellIndex>& cells) {
  const size_t n = static_cast<size_t>(width_) * height_;
  g_score_.assign(n, std::numeric_limits<double>::infinity());
  came_from_.assign(n, -1);
  closed_.assign(n, 0);

  const int start_idx = start.y * width_ + start.x;
  const int goal_idx = goal.y * width_ + goal.x;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  g_score_[start_idx] = 0.0;
  open.push({start_idx, heuristic(start, goal), 0.0});

  while (!open.empty()) {
    const AStarNode current = open.top();
    open.pop();

    if (closed_[current.index]) {
      continue;
    }
    closed_[current.index] = 1;

    if (current.index == goal_idx) {
      cells.clear();
      for (int i = goal_idx; i != -1; i = came_from_[i]) {
        cells.emplace_back(i % width_, i / width_);
      }
      std::reverse(cells.begin(), cells.end());
      return true;
    }

    const int cx = current.index % width_;
    const int cy = current.index / width_;

    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kDx[k];
      const int ny = cy + kDy[k];
      if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
      const int nidx = ny * width_ + nx;
      if (closed_[nidx] || !traversable(nidx)) continue;

      const bool diagonal = kDx[k] != 0 && kDy[k] != 0;
      if (diagonal) {
        if (!traversable(cy * width_ + nx) || !traversable(ny * width_ + cx)) continue;
      }

      const double step = (diagonal ? kSqrt2 : 1.0) * stepMultiplier(nidx);
      const double tentative_g = current.g_score + step;
      if (tentative_g < g_score_[nidx]) {
        g_score_[nidx] = tentative_g;
        came_from_[nidx] = current.index;
        open.push({nidx, tentative_g + heuristic({nx, ny}, goal), tentative_g});
      }
    }
  }
  return false;
}

bool PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map, const Point2D& start,
                       const Point2D& goal, std::vector<Point2D>& path, std::string& error) {
  path.clear();
  data_ = &map.data;
  width_ = static_cast<int>(map.info.width);
  height_ = static_cast<int>(map.info.height);

  if (map.data.size() != static_cast<size_t>(width_) * height_ || width_ == 0) {
    error = "map is empty or malformed";
    return false;
  }

  CellIndex start_cell, goal_cell;
  if (!worldToGrid(map, start, start_cell)) {
    error = "robot is outside the map";
    return false;
  }
  if (!worldToGrid(map, goal, goal_cell)) {
    error = "goal is outside the map";
    return false;
  }

  const int start_cost = map.data[start_cell.y * width_ + start_cell.x];
  escape_cost_ = std::min(99, std::max(-1, start_cost));

  const CellIndex requested_goal_cell = goal_cell;
  if (!traversable(goal_cell.y * width_ + goal_cell.x, false)) {
    const int radius_cells = static_cast<int>(config_.goal_search_radius / map.info.resolution);
    CellIndex adjusted;
    if (!findNearestTraversable(goal_cell, radius_cells, adjusted)) {
      error = "goal is inside an obstacle and no free cell is nearby";
      return false;
    }
    goal_cell = adjusted;
  }

  std::vector<CellIndex> cells;
  if (!runAStar(start_cell, goal_cell, cells)) {
    const int radius_cells = static_cast<int>(config_.goal_search_radius / map.info.resolution);
    CellIndex adjusted;
    if (goal_cell != requested_goal_cell ||
        !findNearestTraversable(requested_goal_cell, radius_cells, adjusted) ||
        !runAStar(start_cell, adjusted, cells)) {
      error = "no path exists";
      return false;
    }
  }

  path.reserve(cells.size());
  for (const auto& c : cells) {
    path.push_back(gridToWorld(map, c));
  }
  if (cells.back() == requested_goal_cell) {
    path.back() = goal;
  }
  return true;
}

}
