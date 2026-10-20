// Copyright 2024 The Autoware Contributors
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
#include "include/path_overlay.hpp"

#include "include/web_mercator.hpp"

#include <QPainterPath>

#include <utility>
#include <vector>

PathOverlay::PathOverlay() : vehicle_lat_(0.0), vehicle_lon_(0.0)
{
}

void PathOverlay::setVehiclePosition(double lat, double lon)
{
  vehicle_lat_ = lat;
  vehicle_lon_ = lon;
}

void PathOverlay::setPathPoints(std::vector<std::pair<double, double>> path_points_geo)
{
  path_points_geo_ = std::move(path_points_geo);
}

void PathOverlay::draw(QPainter & painter, const QRectF & backgroundRect, int zoom) const
{
  if (path_points_geo_.size() < 2) {
    return;
  }

  QPainterPath path;
  for (size_t i = 0; i < path_points_geo_.size(); ++i) {
    const QPointF point = autoware_minimap_overlay_rviz_plugin::lat_lon_to_overlay_pixel(
      path_points_geo_[i].first, path_points_geo_[i].second, vehicle_lat_, vehicle_lon_, zoom,
      backgroundRect);
    if (i == 0) {
      path.moveTo(point);
    } else {
      path.lineTo(point);
    }
  }

  const int line_width = zoom == 15 ? 1 : zoom == 16 ? 2 : 3;

  painter.save();
  painter.setPen(QPen(QColor("#00E678"), line_width));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(path);
  painter.restore();
}
