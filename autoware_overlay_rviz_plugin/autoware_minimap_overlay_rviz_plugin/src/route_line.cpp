// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "route_line.hpp"

#include <boost/archive/binary_iarchive.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware_minimap_overlay_rviz_plugin
{

namespace
{
constexpr double look_ahead_m = 50.0;
constexpr double far_from_line_m = 20.0;

double distance(const QPointF & a, const QPointF & b)
{
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

// Ratio of the point of the segment a-b that is nearest to the position
double nearest_ratio(const QPointF & a, const QPointF & b, const QPointF & position)
{
  const QPointF ab = b - a;
  const double length_squared = QPointF::dotProduct(ab, ab);
  if (length_squared <= 0.0) {
    return 0.0;
  }
  return std::clamp(QPointF::dotProduct(position - a, ab) / length_squared, 0.0, 1.0);
}

struct Nearest
{
  LineProgress progress;
  double distance = std::numeric_limits<double>::max();
};

// Nearest point of the line to the position, on the segments from `first` on.
// The search stops when the segments are longer than max_length in total.
Nearest find_nearest(
  const std::vector<QPointF> & line, const LineProgress & first, const QPointF & position,
  const double max_length)
{
  Nearest nearest;
  double length = 0.0;
  for (std::size_t i = first.segment; i + 1 < line.size() && length <= max_length; ++i) {
    double ratio = nearest_ratio(line[i], line[i + 1], position);
    if (i == first.segment) {
      ratio = std::max(ratio, first.ratio);
    }
    const double d = distance(line[i] + (line[i + 1] - line[i]) * ratio, position);
    if (d < nearest.distance) {
      nearest = {{i, ratio}, d};
    }
    length += distance(line[i], line[i + 1]);
  }
  return nearest;
}

std::size_t nearest_index(const lanelet::ConstLineString3d & line, const double x, const double y)
{
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < line.size(); ++i) {
    const double d = std::hypot(line[i].x() - x, line[i].y() - y);
    if (d < nearest_distance) {
      nearest = i;
      nearest_distance = d;
    }
  }
  return nearest;
}
}  // namespace

lanelet::LaneletMapConstPtr to_lanelet_map(const autoware_map_msgs::msg::LaneletMapBin & msg)
{
  // Same as from_autoware_map_msgs() of autoware_lanelet2_utils, which is not a stable interface
  std::stringstream stream;
  stream << std::string(msg.data.begin(), msg.data.end());
  boost::archive::binary_iarchive archive(stream);

  auto map = std::make_shared<lanelet::LaneletMap>();
  archive >> *map;
  lanelet::Id id_counter = 0;
  archive >> id_counter;
  lanelet::utils::registerId(id_counter);
  return map;
}

std::vector<geometry_msgs::msg::Point> route_centerline(
  const lanelet::LaneletMap & map, const autoware_adapi_v1_msgs::msg::RouteData & route)
{
  std::vector<geometry_msgs::msg::Point> line;
  const std::size_t segment_count = route.segments.size();

  for (std::size_t i = 0; i < segment_count; ++i) {
    const auto lanelet = map.laneletLayer.find(route.segments[i].preferred.id);
    if (lanelet == map.laneletLayer.end()) {
      return {};
    }
    const lanelet::ConstLineString3d centerline = lanelet->centerline();

    // The route starts and ends inside its first and last lanelets
    const std::size_t first =
      i == 0 ? nearest_index(centerline, route.start.position.x, route.start.position.y) : 0;
    const std::size_t last =
      i + 1 == segment_count
        ? nearest_index(centerline, route.goal.position.x, route.goal.position.y)
        : centerline.size() - 1;

    for (std::size_t j = first; j <= last && j < centerline.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = centerline[j].x();
      point.y = centerline[j].y();
      point.z = centerline[j].z();
      line.push_back(point);
    }
  }

  if (!line.empty()) {
    line.push_back(route.goal.position);
  }
  return line;
}

LineProgress advance_progress(
  const std::vector<QPointF> & line, const LineProgress & progress, const QPointF & position)
{
  if (line.size() < 2 || progress.segment + 1 >= line.size()) {
    return progress;
  }
  const Nearest ahead = find_nearest(line, progress, position, look_ahead_m);
  if (ahead.distance <= far_from_line_m) {
    return ahead.progress;
  }
  // For example, RViz started when the vehicle was already on the route
  const Nearest anywhere =
    find_nearest(line, LineProgress(), position, std::numeric_limits<double>::max());
  return anywhere.distance <= far_from_line_m ? anywhere.progress : progress;
}

std::vector<QPointF> remaining_line(
  const std::vector<QPointF> & line, const LineProgress & progress)
{
  if (progress.segment + 1 >= line.size()) {
    return {};
  }
  std::vector<QPointF> remaining;
  remaining.reserve(line.size() - progress.segment);
  const QPointF & a = line[progress.segment];
  const QPointF & b = line[progress.segment + 1];
  remaining.push_back(a + (b - a) * progress.ratio);
  remaining.insert(remaining.end(), line.begin() + progress.segment + 1, line.end());
  return remaining;
}

}  // namespace autoware_minimap_overlay_rviz_plugin
