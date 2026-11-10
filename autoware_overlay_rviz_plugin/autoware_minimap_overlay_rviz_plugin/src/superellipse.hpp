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

#ifndef SUPERELLIPSE_HPP_
#define SUPERELLIPSE_HPP_

#include <QPainterPath>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware_minimap_overlay_rviz_plugin
{

// Shape with the outline |x / a|^n + |y / b|^n = 1 in a rectangle, where a and b are half of the
// width and the height. The exponent n gives the shape: 2 is an ellipse, 4 is a squircle
// (a square with smooth corners), and infinity is the rectangle itself.
struct Superellipse
{
  QRectF rect;
  double exponent = 4.0;

  // 0 at the center, 1 on the outline, and more than 1 outside
  double norm(const QPointF & point) const
  {
    const double u = std::abs(point.x() - rect.center().x()) / (rect.width() / 2.0);
    const double v = std::abs(point.y() - rect.center().y()) / (rect.height() / 2.0);
    if (std::isinf(exponent)) {
      return std::max(u, v);
    }
    return std::pow(std::pow(u, exponent) + std::pow(v, exponent), 1.0 / exponent);
  }

  bool contains(const QPointF & point) const { return norm(point) <= 1.0; }

  // Returns the point if it is inside the shape. If not, returns the point where the line from
  // the center to the point crosses the outline.
  QPointF clamp(const QPointF & point) const
  {
    const double n = norm(point);
    return n <= 1.0 ? point : rect.center() + (point - rect.center()) / n;
  }

  // Half of the width of the shape at a distance above its lowest point
  double halfWidthAboveBottom(const double distance) const
  {
    const double v = std::clamp(1.0 - distance / (rect.height() / 2.0), 0.0, 1.0);
    if (std::isinf(exponent)) {
      return rect.width() / 2.0;
    }
    return rect.width() / 2.0 * std::pow(1.0 - std::pow(v, exponent), 1.0 / exponent);
  }

  // The same shape, smaller by the margin on each side
  Superellipse shrunk(const double margin) const
  {
    return {rect.adjusted(margin, margin, -margin, -margin), exponent};
  }

  QPainterPath path() const
  {
    QPainterPath outline;
    if (std::isinf(exponent)) {
      outline.addRect(rect);
      return outline;
    }
    constexpr int point_count = 180;
    QPolygonF polygon;
    for (int i = 0; i < point_count; ++i) {
      const double angle = 2.0 * M_PI * i / point_count;
      const double c = std::cos(angle);
      const double s = std::sin(angle);
      polygon << rect.center() +
                   QPointF(
                     std::copysign(std::pow(std::abs(c), 2.0 / exponent), c) * rect.width() / 2.0,
                     std::copysign(std::pow(std::abs(s), 2.0 / exponent), s) * rect.height() / 2.0);
    }
    outline.addPolygon(polygon);
    outline.closeSubpath();
    return outline;
  }
};

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // SUPERELLIPSE_HPP_
