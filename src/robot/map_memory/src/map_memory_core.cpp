#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {
  configure(MapMemoryConfig{});
}

void MapMemoryCore::configure(const MapMemoryConfig& config) {
  map_ = nav_msgs::msg::OccupancyGrid();
  map_.header.frame_id = config.frame_id;
  map_.info.resolution = static_cast<float>(config.resolution);
  map_.info.width = static_cast<uint32_t>(std::ceil(config.width / config.resolution));
  map_.info.height = static_cast<uint32_t>(std::ceil(config.height / config.resolution));
  map_.info.origin.position.x = config.origin_x;
  map_.info.origin.position.y = config.origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(static_cast<size_t>(map_.info.width) * map_.info.height, -1);

  RCLCPP_INFO(logger_, "Global map configured: %ux%u cells @ %.2fm in '%s'",
              map_.info.width, map_.info.height, config.resolution, config.frame_id.c_str());
}

// Rather than pushing every local cell into the global grid (which leaves holes once the local
// grid is rotated), we walk every global cell covered by the costmap and sample the local cell
// underneath it. This gives a gap-free result regardless of the relative resolutions.
void MapMemoryCore::fuse(const nav_msgs::msg::OccupancyGrid& costmap, const Pose2D& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);

  const double l_res = costmap.info.resolution;
  const double l_ox = costmap.info.origin.position.x;
  const double l_oy = costmap.info.origin.position.y;
  const int l_w = static_cast<int>(costmap.info.width);
  const int l_h = static_cast<int>(costmap.info.height);

  const double g_res = map_.info.resolution;
  const double g_ox = map_.info.origin.position.x;
  const double g_oy = map_.info.origin.position.y;
  const int g_w = static_cast<int>(map_.info.width);
  const int g_h = static_cast<int>(map_.info.height);

  // Global-frame bounding box of the rotated costmap.
  const double corners[4][2] = {
    {l_ox, l_oy}, {l_ox + l_w * l_res, l_oy},
    {l_ox, l_oy + l_h * l_res}, {l_ox + l_w * l_res, l_oy + l_h * l_res}};
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  for (const auto& p : corners) {
    const double gx = pose.x + c * p[0] - s * p[1];
    const double gy = pose.y + s * p[0] + c * p[1];
    min_x = std::min(min_x, gx); max_x = std::max(max_x, gx);
    min_y = std::min(min_y, gy); max_y = std::max(max_y, gy);
  }

  const int gx0 = std::max(0, static_cast<int>(std::floor((min_x - g_ox) / g_res)));
  const int gy0 = std::max(0, static_cast<int>(std::floor((min_y - g_oy) / g_res)));
  const int gx1 = std::min(g_w - 1, static_cast<int>(std::ceil((max_x - g_ox) / g_res)));
  const int gy1 = std::min(g_h - 1, static_cast<int>(std::ceil((max_y - g_oy) / g_res)));

  for (int gy = gy0; gy <= gy1; ++gy) {
    const double wy = g_oy + (gy + 0.5) * g_res;
    for (int gx = gx0; gx <= gx1; ++gx) {
      const double wx = g_ox + (gx + 0.5) * g_res;

      // Global -> local (inverse rigid transform).
      const double dx = wx - pose.x;
      const double dy = wy - pose.y;
      const double lx = c * dx + s * dy;
      const double ly = -s * dx + c * dy;

      const int cx = static_cast<int>(std::floor((lx - l_ox) / l_res));
      const int cy = static_cast<int>(std::floor((ly - l_oy) / l_res));
      if (cx < 0 || cy < 0 || cx >= l_w || cy >= l_h) {
        continue;
      }

      const int8_t value = costmap.data[cy * l_w + cx];
      if (value >= 0) {
        map_.data[gy * g_w + gx] = value;  // newest observation wins
      }
    }
  }
}

}  // namespace robot
