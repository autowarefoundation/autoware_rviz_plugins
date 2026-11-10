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

#include "web_mercator.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware_minimap_overlay_rviz_plugin::map_size_px;
using autoware_minimap_overlay_rviz_plugin::to_mercator;

TEST(WebMercator, MapSizeDoublesWithEachZoomLevel)
{
  EXPECT_DOUBLE_EQ(map_size_px(0), 256.0);
  EXPECT_DOUBLE_EQ(map_size_px(1), 512.0);
  EXPECT_DOUBLE_EQ(map_size_px(19), 256.0 * 524288.0);
}

TEST(WebMercator, OriginIsAtMapCenter)
{
  const QPointF position = to_mercator(0.0, 0.0);
  EXPECT_NEAR(position.x(), 0.5, 1e-12);
  EXPECT_NEAR(position.y(), 0.5, 1e-12);
}

TEST(WebMercator, MapCornersAreAtTheLatitudeLimit)
{
  const QPointF north_west = to_mercator(85.0511287798, -180.0);
  EXPECT_NEAR(north_west.x(), 0.0, 1e-9);
  EXPECT_NEAR(north_west.y(), 0.0, 1e-9);

  const QPointF south_east = to_mercator(-85.0511287798, 180.0);
  EXPECT_NEAR(south_east.x(), 1.0, 1e-9);
  EXPECT_NEAR(south_east.y(), 1.0, 1e-9);
}

TEST(WebMercator, LatitudeBeyondTheLimitIsClamped)
{
  EXPECT_DOUBLE_EQ(to_mercator(90.0, 0.0).y(), to_mercator(85.0511287798, 0.0).y());
  EXPECT_DOUBLE_EQ(to_mercator(-90.0, 0.0).y(), to_mercator(-85.0511287798, 0.0).y());
}

TEST(WebMercator, NorthEastIsUpRight)
{
  const QPointF vehicle = to_mercator(35.9, 139.93);
  const QPointF point = to_mercator(35.901, 139.931);
  EXPECT_GT(point.x(), vehicle.x());
  EXPECT_LT(point.y(), vehicle.y());
}

TEST(WebMercator, KnownTileOfTheSampleMap)
{
  // Kashiwanoha, the place of the Autoware sample map, is in the tile 17/116483/51513
  const QPointF position = to_mercator(35.9024281, 139.9326469) * map_size_px(17);
  EXPECT_EQ(static_cast<int>(std::floor(position.x() / 256.0)), 116483);
  EXPECT_EQ(static_cast<int>(std::floor(position.y() / 256.0)), 51513);
}

TEST(WebMercator, PixelDistanceMatchesGroundResolution)
{
  // 1 km east of the vehicle is many tiles away at zoom level 17
  const double latitude = 35.9;
  const double longitude = 139.93;
  const double meters_east = 1000.0;
  const double meters_per_degree = 111319.49 * std::cos(latitude * M_PI / 180.0);
  const int zoom = 17;
  const double meters_per_pixel =
    156543.03392 * std::cos(latitude * M_PI / 180.0) / std::pow(2.0, zoom);

  const QPointF offset_px = (to_mercator(latitude, longitude + meters_east / meters_per_degree) -
                             to_mercator(latitude, longitude)) *
                            map_size_px(zoom);

  EXPECT_NEAR(offset_px.x(), meters_east / meters_per_pixel, 1.0);
  EXPECT_NEAR(offset_px.y(), 0.0, 1e-6);
  EXPECT_GT(offset_px.x(), 256.0);
}
