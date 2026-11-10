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

#include "map_style.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware_minimap_overlay_rviz_plugin::apply_map_style;
using autoware_minimap_overlay_rviz_plugin::map_background;
using autoware_minimap_overlay_rviz_plugin::MapStyle;

namespace
{
// Colors of the standard OpenStreetMap style
const QColor osm_land(242, 239, 233);
const QColor osm_small_road(255, 255, 255);
const QColor osm_primary_road(252, 214, 164);
const QColor osm_motorway(232, 146, 162);
const QColor osm_building(217, 208, 201);
const QColor osm_water(170, 211, 223);
const QColor osm_park(200, 250, 204);
const QColor osm_school_area(255, 255, 229);
const QColor osm_text(0, 0, 0);

QColor styled(const QColor & color, const MapStyle style)
{
  QImage tile(2, 2, QImage::Format_RGB32);
  tile.fill(color);
  return apply_map_style(tile, style).pixelColor(1, 1);
}

// Sum of the differences of the color channels
int difference(const QColor & a, const QColor & b)
{
  return std::abs(a.red() - b.red()) + std::abs(a.green() - b.green()) +
         std::abs(a.blue() - b.blue());
}
}  // namespace

TEST(MapStyle, StandardStyleKeepsTheColors)
{
  for (const QColor & color : {osm_land, osm_small_road, osm_motorway, osm_water, osm_text}) {
    EXPECT_EQ(styled(color, MapStyle::Standard), color);
  }
  EXPECT_EQ(map_background(MapStyle::Standard), osm_land);
}

TEST(MapStyle, LandAndParksBecomeTheBackground)
{
  for (const MapStyle style : {MapStyle::NavigationDark, MapStyle::NavigationLight}) {
    EXPECT_EQ(styled(osm_land, style), map_background(style));
    EXPECT_EQ(styled(osm_park, style), map_background(style));
    EXPECT_EQ(styled(osm_school_area, style), map_background(style));
  }
}

TEST(MapStyle, DarkStyleIsDarkAndLightStyleIsLight)
{
  EXPECT_LT(map_background(MapStyle::NavigationDark).lightness(), 50);
  EXPECT_GT(map_background(MapStyle::NavigationLight).lightness(), 220);
}

TEST(MapStyle, RoadsStandOutFromTheLand)
{
  for (const MapStyle style : {MapStyle::NavigationDark, MapStyle::NavigationLight}) {
    const QColor land = map_background(style);
    const QColor small_road = styled(osm_small_road, style);
    const QColor primary_road = styled(osm_primary_road, style);
    EXPECT_GT(difference(small_road, land), 40);
    EXPECT_GT(difference(primary_road, land), 100);
    // All the large roads have the same color
    EXPECT_EQ(styled(osm_motorway, style), primary_road);
    // The buildings are faint: much nearer to the land than the roads
    EXPECT_LT(difference(styled(osm_building, style), land), difference(small_road, land));
    EXPECT_NE(styled(osm_building, style), land);
  }
}

TEST(MapStyle, WaterIsBlue)
{
  for (const MapStyle style : {MapStyle::NavigationDark, MapStyle::NavigationLight}) {
    const QColor water = styled(osm_water, style);
    EXPECT_NE(water, map_background(style));
    EXPECT_GT(water.blue(), water.red());
  }
}

TEST(MapStyle, TextIsLightOnDarkAndDarkOnLight)
{
  EXPECT_GT(styled(osm_text, MapStyle::NavigationDark).lightness(), 220);
  EXPECT_LT(styled(osm_text, MapStyle::NavigationLight).lightness(), 40);
}
