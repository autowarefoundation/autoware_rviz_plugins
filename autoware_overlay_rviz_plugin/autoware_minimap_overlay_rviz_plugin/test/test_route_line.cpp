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

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <vector>

using autoware_adapi_v1_msgs::msg::RouteData;
using autoware_adapi_v1_msgs::msg::RouteSegment;
using autoware_minimap_overlay_rviz_plugin::advance_progress;
using autoware_minimap_overlay_rviz_plugin::LineProgress;
using autoware_minimap_overlay_rviz_plugin::remaining_line;
using autoware_minimap_overlay_rviz_plugin::route_centerline;

namespace
{
// Lanelet of 2 m width along the x axis, with a point at each 10 m. Its centerline is at y = 0.
lanelet::Lanelet make_lanelet(const lanelet::Id id, const double x_begin, const double x_end)
{
  lanelet::LineString3d left(lanelet::utils::getId());
  lanelet::LineString3d right(lanelet::utils::getId());
  for (double x = x_begin; x <= x_end; x += 10.0) {
    left.push_back(lanelet::Point3d(lanelet::utils::getId(), x, 1.0, 0.0));
    right.push_back(lanelet::Point3d(lanelet::utils::getId(), x, -1.0, 0.0));
  }
  return lanelet::Lanelet(id, left, right);
}

RouteData make_route(
  const std::vector<lanelet::Id> & ids, const double start_x, const double goal_x)
{
  RouteData route;
  route.start.position.x = start_x;
  route.goal.position.x = goal_x;
  for (const lanelet::Id id : ids) {
    RouteSegment segment;
    segment.preferred.id = id;
    route.segments.push_back(segment);
  }
  return route;
}

const std::vector<QPointF> straight_line = {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}, {30.0, 0.0}};
}  // namespace

TEST(RouteCenterline, FollowsTheLaneletsFromTheStartToTheGoal)
{
  const auto map =
    lanelet::utils::createMap({make_lanelet(1, 0.0, 30.0), make_lanelet(2, 30.0, 60.0)});

  const auto line = route_centerline(*map, make_route({1, 2}, 11.0, 47.0));

  // The centerline of lanelet2 has a point at each 5 m.
  // The line starts at the point nearest to the start, and ends at the goal.
  ASSERT_GE(line.size(), 3u);
  EXPECT_DOUBLE_EQ(line.front().x, 10.0);
  EXPECT_DOUBLE_EQ(line[line.size() - 2].x, 45.0);
  EXPECT_DOUBLE_EQ(line.back().x, 47.0);
  for (std::size_t i = 0; i < line.size(); ++i) {
    EXPECT_NEAR(line[i].y, 0.0, 1e-9);
    if (i > 0) {
      EXPECT_GE(line[i].x, line[i - 1].x);
      EXPECT_LE(line[i].x - line[i - 1].x, 5.0);
    }
  }
}

TEST(RouteCenterline, RouteInOneLanelet)
{
  const auto map = lanelet::utils::createMap({make_lanelet(1, 0.0, 30.0)});

  const auto line = route_centerline(*map, make_route({1}, 9.0, 21.0));

  std::vector<double> x;
  for (const auto & point : line) {
    x.push_back(point.x);
  }
  EXPECT_EQ(x, (std::vector<double>{10.0, 15.0, 20.0, 21.0}));
}

TEST(RouteCenterline, IsEmptyWhenTheMapDoesNotHaveALanelet)
{
  const auto map = lanelet::utils::createMap({make_lanelet(1, 0.0, 30.0)});

  EXPECT_TRUE(route_centerline(*map, make_route({1, 99}, 0.0, 30.0)).empty());
  EXPECT_TRUE(route_centerline(*map, make_route({}, 0.0, 30.0)).empty());
}

TEST(LineProgress, MovesToThePointNearestToThePosition)
{
  const LineProgress progress = advance_progress(straight_line, LineProgress(), QPointF(14.0, 3.0));
  EXPECT_EQ(progress.segment, 1u);
  EXPECT_NEAR(progress.ratio, 0.4, 1e-9);
}

TEST(LineProgress, DoesNotMoveBack)
{
  const LineProgress progress = advance_progress(straight_line, {2, 0.5}, QPointF(22.0, 0.0));
  EXPECT_EQ(progress.segment, 2u);
  EXPECT_NEAR(progress.ratio, 0.5, 1e-9);
}

TEST(LineProgress, StaysOnTheFirstPassOfALineThatCrossesItself)
{
  // The line passes the point (50, 0) a second time, 200 m later along the line
  const std::vector<QPointF> line = {{0.0, 0.0},    {50.0, 0.0},  {100.0, 0.0},
                                     {100.0, 50.0}, {50.0, 50.0}, {50.0, -10.0}};
  const LineProgress progress = advance_progress(line, LineProgress(), QPointF(49.0, 0.5));
  EXPECT_EQ(progress.segment, 0u);
  EXPECT_NEAR(progress.ratio, 0.98, 1e-9);
}

TEST(LineProgress, SearchesTheWholeLineWhenThePositionIsFarAhead)
{
  std::vector<QPointF> line;
  for (int i = 0; i <= 100; ++i) {
    line.emplace_back(10.0 * i, 0.0);
  }
  const LineProgress progress = advance_progress(line, LineProgress(), QPointF(705.0, 1.0));
  EXPECT_EQ(progress.segment, 70u);
  EXPECT_NEAR(progress.ratio, 0.5, 1e-9);
}

TEST(LineProgress, StaysWhenThePositionIsFarFromTheLine)
{
  const LineProgress progress = advance_progress(straight_line, {1, 0.25}, QPointF(20.0, 500.0));
  EXPECT_EQ(progress.segment, 1u);
  EXPECT_NEAR(progress.ratio, 0.25, 1e-9);
}

TEST(LineProgress, RemainingLineStartsAtTheProgress)
{
  const std::vector<QPointF> remaining = remaining_line(straight_line, {1, 0.4});
  EXPECT_EQ(remaining, (std::vector<QPointF>{{14.0, 0.0}, {20.0, 0.0}, {30.0, 0.0}}));
  EXPECT_EQ(remaining_line(straight_line, LineProgress()), straight_line);
}

TEST(LineProgress, RemainingLineOfAnEmptyLineIsEmpty)
{
  EXPECT_TRUE(remaining_line({}, LineProgress()).empty());
  EXPECT_TRUE(remaining_line({QPointF(1.0, 1.0)}, LineProgress()).empty());
  EXPECT_TRUE(remaining_line(straight_line, {3, 0.0}).empty());
}
