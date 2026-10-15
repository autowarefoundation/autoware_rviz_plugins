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

#include "../src/include/web_mercator.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware_minimap_overlay_rviz_plugin::lat_lon_to_overlay_pixel;
using autoware_minimap_overlay_rviz_plugin::lat_lon_to_world_pixel;

TEST(WebMercator, WorldPixelOfOriginIsMapCenter)
{
  for (int zoom = 15; zoom <= 18; ++zoom) {
    const double half_map_size_px = 256.0 * std::pow(2.0, zoom) / 2.0;
    const QPointF pixel = lat_lon_to_world_pixel(0.0, 0.0, zoom);
    EXPECT_NEAR(pixel.x(), half_map_size_px, 1e-6);
    EXPECT_NEAR(pixel.y(), half_map_size_px, 1e-6);
  }
}

TEST(WebMercator, VehicleIsAtOverlayCenter)
{
  const QRectF overlay_rect(0.0, 0.0, 256.0, 200.0);
  const QPointF pixel = lat_lon_to_overlay_pixel(35.9, 139.93, 35.9, 139.93, 17, overlay_rect);
  EXPECT_NEAR(pixel.x(), 128.0, 1e-6);
  EXPECT_NEAR(pixel.y(), 100.0, 1e-6);
}

TEST(WebMercator, NorthEastIsUpRight)
{
  const QRectF overlay_rect(0.0, 0.0, 256.0, 256.0);
  const QPointF pixel = lat_lon_to_overlay_pixel(35.901, 139.931, 35.9, 139.93, 17, overlay_rect);
  EXPECT_GT(pixel.x(), 128.0);
  EXPECT_LT(pixel.y(), 128.0);
}

TEST(WebMercator, DistanceAcrossTileBorderMatchesGroundResolution)
{
  // About 1 km east of the vehicle: many tiles away at zoom 17
  const double lat = 35.9;
  const double vehicle_lon = 139.93;
  const double meters_east = 1000.0;
  const double meters_per_degree_lon = 111319.49 * std::cos(lat * M_PI / 180.0);
  const double lon = vehicle_lon + meters_east / meters_per_degree_lon;

  const int zoom = 17;
  const double meters_per_pixel = 156543.03392 * std::cos(lat * M_PI / 180.0) / std::pow(2.0, zoom);

  const QRectF overlay_rect(0.0, 0.0, 256.0, 256.0);
  const QPointF pixel = lat_lon_to_overlay_pixel(lat, lon, lat, vehicle_lon, zoom, overlay_rect);

  EXPECT_NEAR(pixel.x() - 128.0, meters_east / meters_per_pixel, 1.0);
  EXPECT_NEAR(pixel.y(), 128.0, 1e-6);
  // More than one tile away, so a per-tile offset would be wrong
  EXPECT_GT(pixel.x() - 128.0, 256.0);
}
