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
#include <QRectF>

#include <cmath>

namespace autoware_minimap_overlay_rviz_plugin
{

constexpr double tile_size_px = 256.0;

// Position in pixels on the whole Web Mercator map at the given zoom level
inline QPointF lat_lon_to_world_pixel(double lat, double lon, int zoom)
{
  const double map_size_px = tile_size_px * std::pow(2.0, zoom);
  const double lat_rad = lat * M_PI / 180.0;
  return {
    (lon + 180.0) / 360.0 * map_size_px,
    (1.0 - std::asinh(std::tan(lat_rad)) / M_PI) / 2.0 * map_size_px};
}

// Position in pixels on the overlay, where the vehicle is at the center of the overlay
inline QPointF lat_lon_to_overlay_pixel(
  double lat, double lon, double vehicle_lat, double vehicle_lon, int zoom,
  const QRectF & overlay_rect)
{
  return overlay_rect.center() + lat_lon_to_world_pixel(lat, lon, zoom) -
         lat_lon_to_world_pixel(vehicle_lat, vehicle_lon, zoom);
}

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // WEB_MERCATOR_HPP_
