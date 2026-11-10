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

#ifndef ROUTE_LINE_HPP_
#define ROUTE_LINE_HPP_

#include <QPointF>

#include <autoware_adapi_v1_msgs/msg/route_data.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <vector>

namespace autoware_minimap_overlay_rviz_plugin
{

// Position on a line: between the points `segment` and `segment + 1`.
// The ratio is 0 at the first point of the segment and 1 at the second point.
struct LineProgress
{
  std::size_t segment = 0;
  double ratio = 0.0;
};

lanelet::LaneletMapConstPtr to_lanelet_map(const autoware_map_msgs::msg::LaneletMapBin & msg);

// Line along the centerlines of the preferred lanelets of the route, in the map frame.
// It starts near the start pose of the route and ends at the goal.
// It is empty if the map does not have a lanelet of the route.
std::vector<geometry_msgs::msg::Point> route_centerline(
  const lanelet::LaneletMap & map, const autoware_adapi_v1_msgs::msg::RouteData & route);

// Moves the progress forward to the point of the line that is nearest to the position.
// It looks only a limited distance ahead, so that it does not jump to a later part of a route
// that crosses itself. If the position is far from that part, it searches the whole line.
LineProgress advance_progress(
  const std::vector<QPointF> & line, const LineProgress & progress, const QPointF & position);

// Part of the line from the progress to the end
std::vector<QPointF> remaining_line(
  const std::vector<QPointF> & line, const LineProgress & progress);

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // ROUTE_LINE_HPP_
