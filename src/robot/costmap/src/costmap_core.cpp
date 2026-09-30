#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
  configure(CostmapConfig{});
}

void CostmapCore::configure(const CostmapConfig& config) {
  config_ = config;
  width_ = static_cast<int>(std::ceil(config_.size / config_.resolution));
  origin_ = -0.5 * width_ * config_.resolution;
  grid_.assign(static_cast<size_t>(width_) * width_, -1);

  // Precompute the linear inflation falloff: cost = max_cost * (1 - d / r)
  kernel_radius_ = static_cast<int>(std::ceil(config_.inflation_radius / config_.resolution));
  const int k = 2 * kernel_radius_ + 1;
  inflation_kernel_.assign(static_cast<size_t>(k) * k, 0.0);
  for (int dy = -kernel_radius_; dy <= kernel_radius_; ++dy) {
    for (int dx = -kernel_radius_; dx <= kernel_radius_; ++dx) {
      const double d = std::hypot(dx, dy) * config_.resolution;
      const double factor = d <= config_.inflation_radius ? 1.0 - d / config_.inflation_radius : 0.0;
      inflation_kernel_[(dy + kernel_radius_) * k + (dx + kernel_radius_)] = factor;
    }
  }

  RCLCPP_INFO(logger_, "Costmap configured: %dx%d cells @ %.2fm, inflation %.2fm",
              width_, width_, config_.resolution, config_.inflation_radius);
}

// Classifies every cell against the beam pointing at it. Compared to tracing individual rays this
// leaves no unobserved wedges between beams at long range, so free space and obstacle outlines are
// continuous. Cells behind a return (e.g. inside an obstacle) stay unknown.
void CostmapCore::classifyCells(const sensor_msgs::msg::LaserScan& scan, double usable_max) {
  const int n = static_cast<int>(scan.ranges.size());
  const double res = config_.resolution;
  const double hit_band = 0.75 * res;  // how close to a return a cell must be to count as occupied

  for (int cy = 0; cy < width_; ++cy) {
    const double y = origin_ + (cy + 0.5) * res;
    for (int cx = 0; cx < width_; ++cx) {
      const double x = origin_ + (cx + 0.5) * res;
      const double d = std::hypot(x, y);
      if (d > usable_max) {
        continue;
      }
      const int idx = cy * width_ + cx;
      if (d < res) {
        grid_[idx] = 0;  // the sensor's own cell
        continue;
      }

      const int beam = static_cast<int>(std::lround((std::atan2(y, x) - scan.angle_min) / scan.angle_increment));
      if (beam < 0 || beam >= n) {
        continue;  // outside the scanner's field of view
      }
      const double r = scan.ranges[beam];
      if (std::isnan(r) || r < scan.range_min) {
        continue;  // invalid return: we learn nothing along this beam
      }

      const bool hit = std::isfinite(r) && r <= usable_max;
      const double beam_range = hit ? r : usable_max;
      if (d < beam_range - hit_band) {
        grid_[idx] = 0;
      } else if (hit && d <= beam_range + hit_band) {
        grid_[idx] = static_cast<int8_t>(config_.max_cost);
        obstacle_cells_.push_back(idx);
      }
    }
  }
}

void CostmapCore::inflate() {
  const int k = 2 * kernel_radius_ + 1;
  for (int idx : obstacle_cells_) {
    const int ox = idx % width_;
    const int oy = idx / width_;
    for (int dy = -kernel_radius_; dy <= kernel_radius_; ++dy) {
      const int ny = oy + dy;
      if (ny < 0 || ny >= width_) continue;
      for (int dx = -kernel_radius_; dx <= kernel_radius_; ++dx) {
        const int nx = ox + dx;
        if (nx < 0 || nx >= width_) continue;
        const double factor = inflation_kernel_[(dy + kernel_radius_) * k + (dx + kernel_radius_)];
        if (factor <= 0.0) continue;
        const int8_t cost = static_cast<int8_t>(config_.max_cost * factor);
        int8_t& cell = grid_[ny * width_ + nx];
        // Only raise the cost of cells this scan actually observed. Unobserved cells stay unknown
        // (-1) so map memory never overwrites what it learned earlier with guesses.
        if (cell >= 0 && cost > cell) {
          cell = cost;
        }
      }
    }
  }
}

bool CostmapCore::build(const sensor_msgs::msg::LaserScan& scan, nav_msgs::msg::OccupancyGrid& out) {
  std::fill(grid_.begin(), grid_.end(), static_cast<int8_t>(-1));
  obstacle_cells_.clear();

  if (scan.ranges.empty() || scan.angle_increment == 0.0f) {
    return false;
  }
  const double usable_max = std::min(static_cast<double>(scan.range_max), config_.max_range);

  // Also mark each return's exact cell, in case it falls between cell centres.
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > usable_max) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const int cx = static_cast<int>(std::floor((r * std::cos(angle) - origin_) / config_.resolution));
    const int cy = static_cast<int>(std::floor((r * std::sin(angle) - origin_) / config_.resolution));
    if (cx >= 0 && cy >= 0 && cx < width_ && cy < width_) {
      obstacle_cells_.push_back(cy * width_ + cx);
    }
  }
  if (obstacle_cells_.empty()) {
    return false;
  }

  classifyCells(scan, usable_max);
  for (int idx : obstacle_cells_) {
    grid_[idx] = static_cast<int8_t>(config_.max_cost);
  }
  inflate();

  out.header = scan.header;
  out.info.resolution = static_cast<float>(config_.resolution);
  out.info.width = static_cast<uint32_t>(width_);
  out.info.height = static_cast<uint32_t>(width_);
  out.info.origin = geometry_msgs::msg::Pose();
  out.info.origin.position.x = origin_;
  out.info.origin.position.y = origin_;
  out.info.origin.orientation.w = 1.0;
  out.data = grid_;
  return true;
}

}  // namespace robot
