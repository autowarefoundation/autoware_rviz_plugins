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

#ifndef WEB_MERCATOR_HPP_
#define WEB_MERCATOR_HPP_

#include <QPointF>

#include <algorithm>
#include <cmath>

namespace autoware_minimap_overlay_rviz_plugin
{

constexpr int tile_size_px = 256;
// The Web Mercator map ends at this latitude
constexpr double max_latitude_deg = 85.0511287798;

// Position on the Web Mercator map as a fraction of the map size.
// x grows to the east and y grows to the south. Both are in [0, 1].
inline QPointF to_mercator(const double latitude_deg, const double longitude_deg)
{
  const double latitude_rad =
    std::clamp(latitude_deg, -max_latitude_deg, max_latitude_deg) * M_PI / 180.0;
  return {(longitude_deg + 180.0) / 360.0, (1.0 - std::asinh(std::tan(latitude_rad)) / M_PI) / 2.0};
}

// Width and height of the whole map in pixels at a zoom level
inline double map_size_px(const int zoom)
{
  return std::ldexp(static_cast<double>(tile_size_px), zoom);
}

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // WEB_MERCATOR_HPP_
