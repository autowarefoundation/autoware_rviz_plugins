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

#include "minimap_painter.hpp"

#include "web_mercator.hpp"

#include <QFont>
#include <QFontMetricsF>
#include <QLinearGradient>
#include <QPainterPath>
#include <QPen>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware_minimap_overlay_rviz_plugin
{

namespace
{
// Brand colors of Autoware, from https://docs.autoware.org/main/home/media-kit/
const QColor navy(0, 0, 42);
const QColor indigo(63, 46, 227);
const QColor cyan(0, 200, 255);
const QColor mist(242, 245, 255);
const QColor grey_on_dark(154, 157, 187);

// The sizes are for a minimap of 256 pixels, see designScale()
constexpr double design_size_px = 256.0;
// Highest zoom level of the tile server
constexpr int max_tile_zoom = 19;
constexpr double border_width_px = 2.0;
constexpr double vehicle_icon_size_px = 40.0;
constexpr double goal_icon_size_px = 30.0;
constexpr double goal_direction_icon_size_px = 22.0;
constexpr double button_size_px = 26.0;
constexpr double button_margin_px = 5.0;
constexpr double recenter_bottom_gap_px = 50.0;
constexpr double route_width_px = 4.0;
constexpr double route_casing_width_px = 7.0;
constexpr int map_oversampling = 2;

// The 3D view looks at the map plane with this angle from the vertical
constexpr double tilt_deg = 45.0;
// Distance of the viewer of the 3D view from the view center, as a ratio of the minimap height
constexpr double viewer_distance_ratio = 1.4;
// Height of the view center of the 3D view on the minimap, as a ratio of the minimap height
constexpr double anchor_height_ratio = 0.7;
// The 3D view does not draw the part of the map plane that is nearer to the viewer than this
// ratio of the viewer distance
constexpr double near_ratio = 0.2;
// The 3D view shows the map up to this distance from the view center, as a ratio of the longer
// side of the minimap. This limits the number of tiles. The fade at the top hides the limit.
constexpr double max_view_radius_ratio = 1.6;

QRectF centered_square(const QPointF & center, const double size)
{
  return {center.x() - size / 2.0, center.y() - size / 2.0, size, size};
}

// The rectangle in the units of the design
QRectF to_design(const QRectF & rect, const double scale)
{
  return {rect.topLeft() / scale, rect.size() / scale};
}
}  // namespace

MinimapPainter::Icons::Icons(const QString & directory)
: vehicle(directory + "/vehicle.svg"),
  goal(directory + "/goal.svg"),
  goal_direction(directory + "/goal_direction.svg"),
  layers(directory + "/layers.svg"),
  compass(directory + "/compass.svg"),
  recenter(directory + "/recenter.svg"),
  zoom_in(directory + "/zoom_in.svg"),
  zoom_out(directory + "/zoom_out.svg")
{
}

bool MinimapPainter::Icons::isValid() const
{
  return vehicle.isValid() && goal.isValid() && goal_direction.isValid() && layers.isValid() &&
         compass.isValid() && recenter.isValid() && zoom_in.isValid() && zoom_out.isValid();
}

MinimapPainter::MinimapPainter(const QString & icon_directory)
: dark_icons_(icon_directory + "/dark"), light_icons_(icon_directory + "/light")
{
}

bool MinimapPainter::isValid() const
{
  return dark_icons_.isValid() && light_icons_.isValid();
}

const MinimapPainter::Colors & MinimapPainter::colors(const MapStyle map)
{
  // Cyan is the accent on dark backgrounds, and indigo is the accent on light backgrounds
  static const Colors dark{grey_on_dark, cyan, navy, navy, mist};
  static const Colors light{navy, indigo, Qt::white, Qt::white, navy};
  return map == MapStyle::NavigationDark ? dark : light;
}

MinimapPainter::Icons & MinimapPainter::icons(const MapStyle map)
{
  return map == MapStyle::NavigationDark ? dark_icons_ : light_icons_;
}

Superellipse MinimapPainter::outline(const QRectF & rect, const MinimapShape shape)
{
  switch (shape) {
    case MinimapShape::Circle:
      return {rect, 2.0};
    case MinimapShape::Rectangle:
      return {rect, std::numeric_limits<double>::infinity()};
    default:
      return {rect, 4.0};
  }
}

double MinimapPainter::mapRotation(const MinimapStyle & style, const MinimapScene & scene)
{
  return style.view == MinimapView::NorthUp ? 0.0 : -scene.vehicle_heading_deg;
}

MinimapPainter::Projection::Projection(
  const QRectF & rect, const MinimapView view, const double map_rotation_deg)
: anchor(rect.center())
{
  rotation.rotate(map_rotation_deg);
  if (view == MinimapView::Perspective) {
    anchor.setY(rect.top() + anchor_height_ratio * rect.height());
    tilt_sin = std::sin(tilt_deg * M_PI / 180.0);
    tilt_cos = std::cos(tilt_deg * M_PI / 180.0);
    distance = viewer_distance_ratio * rect.height();
  }

  // The same projection as project(), as a matrix. See project() for the formula.
  const QTransform tilt(
    1.0, 0.0, 0.0, -anchor.x() * tilt_sin / distance, tilt_cos - anchor.y() * tilt_sin / distance,
    -tilt_sin / distance, anchor.x(), anchor.y(), 1.0);
  tile_transform =
    QTransform::fromTranslate(-rect.center().x(), -rect.center().y()) * rotation * tilt;
}

QPointF MinimapPainter::Projection::projectFlat(const QPointF & offset) const
{
  return anchor + rotation.map(offset);
}

std::optional<QPointF> MinimapPainter::Projection::project(const QPointF & offset) const
{
  // The map plane turns around the horizontal line through the view center. A point that is
  // `rotated.y()` below the view center comes nearer to the viewer, and one above goes away.
  const QPointF rotated = rotation.map(offset);
  const double depth_ratio = 1.0 - tilt_sin * rotated.y() / distance;
  if (depth_ratio < near_ratio) {
    return std::nullopt;
  }
  return anchor + QPointF(rotated.x(), rotated.y() * tilt_cos) / depth_ratio;
}

double MinimapPainter::designScale(const QRectF & rect)
{
  return std::min(rect.width(), rect.height()) / design_size_px;
}

int MinimapPainter::tileZoom(const QRectF & rect, const int zoom)
{
  // Each level more has tiles with 2 times the detail
  const int detail = std::max(0, static_cast<int>(std::lround(std::log2(designScale(rect)))));
  return std::max(zoom, std::min(zoom + detail, max_tile_zoom));
}

QSizeF MinimapPainter::tileViewSize(
  const QRectF & device_rect, const MinimapStyle & style, const int zoom)
{
  const QRectF rect = to_design(device_rect, designScale(device_rect));
  // The tiles must reach the place on the map of each corner of the minimap, at each rotation
  const Projection projection(rect, style.view, 0.0);
  const QTransform to_map = projection.tile_transform.inverted();
  double radius = 0.0;
  for (const QPointF & corner :
       {rect.topLeft(), rect.topRight(), rect.bottomLeft(), rect.bottomRight()}) {
    const QPointF on_map = to_map.map(corner) - rect.center();
    radius = std::max(radius, std::hypot(on_map.x(), on_map.y()));
  }
  radius = std::min(radius, max_view_radius_ratio * std::max(rect.width(), rect.height()));
  // The radius is in map pixels of the zoom level. The tiles can have a higher zoom level.
  radius = std::ldexp(radius, tileZoom(device_rect, zoom) - zoom);
  return {2.0 * radius, 2.0 * radius};
}

QPointF MinimapPainter::buttonCenter(
  const MinimapButton button, const QRectF & device_rect, const MinimapShape shape)
{
  const double scale = designScale(device_rect);
  const QRectF rect = to_design(device_rect, scale);
  if (button == MinimapButton::Recenter) {
    // At the bottom center, above the attribution text
    return QPointF(rect.center().x(), rect.bottom() - recenter_bottom_gap_px) * scale;
  }
  // Each of the other buttons is in one corner of the minimap
  const bool left = button == MinimapButton::Layers || button == MinimapButton::ZoomOut;
  const bool top = button == MinimapButton::Layers || button == MinimapButton::Compass;
  const QPointF corner(left ? rect.left() : rect.right(), top ? rect.top() : rect.bottom());
  const double margin = border_width_px + button_margin_px + button_size_px / 2.0;
  return outline(rect, shape).shrunk(margin).clamp(corner) * scale;
}

std::optional<MinimapButton> MinimapPainter::buttonAt(
  const QPointF & position, const QRectF & rect, const MinimapStyle & style,
  const bool has_recenter_button)
{
  for (const MinimapButton button :
       {MinimapButton::Layers, MinimapButton::Compass, MinimapButton::ZoomOut,
        MinimapButton::ZoomIn, MinimapButton::Recenter}) {
    if (button == MinimapButton::Recenter && !has_recenter_button) {
      continue;
    }
    const QPointF offset = position - buttonCenter(button, rect, style.shape);
    if (std::hypot(offset.x(), offset.y()) <= button_size_px / 2.0 * designScale(rect)) {
      return button;
    }
  }
  return std::nullopt;
}

void MinimapPainter::paint(
  QPainter & painter, const QRectF & device_rect, const MinimapStyle & style,
  const MinimapScene & scene, TileLoader * tiles)
{
  // From here on, all sizes and positions are in the units of the design
  const double scale = designScale(device_rect);
  const QRectF rect = to_design(device_rect, scale);
  const Colors & theme_colors = colors(style.map);
  Icons & theme_icons = icons(style.map);
  const QColor background = map_background(style.map);
  const Superellipse shape = outline(rect, style.shape);
  const QPainterPath shape_path = shape.path();

  painter.save();
  painter.scale(scale, scale);
  painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
  painter.setClipPath(shape_path);

  if (!scene.vehicle) {
    painter.fillPath(shape_path, navy);
    paintMessage(painter, rect, "Waiting for the\nvehicle position");
  } else {
    painter.fillPath(shape_path, background);

    const double map_rotation_deg = mapRotation(style, scene);
    const Projection projection(rect, style.view, map_rotation_deg);
    const QPointF view_center = scene.view_center.value_or(*scene.vehicle);
    const double map_size = map_size_px(scene.zoom);
    // Offset from the view center in map pixels, with north up
    const auto to_offset = [&](const QPointF & position) {
      return (position - view_center) * map_size;
    };

    // The tiles can have a higher zoom level than the map. Then they are drawn smaller.
    const int tile_zoom = tileZoom(device_rect, scene.zoom);
    const double tile_scale = std::ldexp(1.0, scene.zoom - tile_zoom);
    const QTransform tile_transform =
      QTransform::fromTranslate(-rect.center().x(), -rect.center().y()) *
      QTransform::fromScale(tile_scale, tile_scale) *
      QTransform::fromTranslate(rect.center().x(), rect.center().y()) * projection.tile_transform;

    if (tiles && style.view == MinimapView::NorthUp && scale * tile_scale == 1.0) {
      // Each pixel of a tile is one pixel of the minimap
      tiles->paint(painter, rect, true);
    } else if (tiles) {
      // A scaled, rotated or tilted tile image has rough edges. Draw it larger and scale it down,
      // so that each pixel of the minimap is the mean of some pixels of the map.
      const QSize layer_size = (device_rect.size() * map_oversampling).toSize();
      if (map_layer_.size() != layer_size) {
        map_layer_ = QImage(layer_size, QImage::Format_ARGB32_Premultiplied);
      }
      map_layer_.fill(Qt::transparent);
      QPainter layer_painter(&map_layer_);
      layer_painter.setRenderHint(QPainter::SmoothPixmapTransform);
      layer_painter.scale(map_oversampling * scale, map_oversampling * scale);
      layer_painter.translate(-rect.topLeft());
      layer_painter.setTransform(tile_transform, true);
      tiles->paint(layer_painter, rect, false);
      layer_painter.end();
      painter.drawImage(rect, map_layer_);
    }
    if (style.view == MinimapView::Perspective) {
      // The far part of the map fades out
      QLinearGradient fade(rect.topLeft(), rect.topLeft() + QPointF(0.0, 0.3 * rect.height()));
      QColor transparent = background;
      transparent.setAlpha(0);
      fade.setColorAt(0.0, background);
      fade.setColorAt(1.0, transparent);
      painter.fillRect(rect, fade);
    }

    std::vector<QPointF> route;
    route.reserve(scene.route.size());
    for (const QPointF & position : scene.route) {
      route.push_back(to_offset(position));
    }
    paintRoute(painter, route, projection, theme_colors);

    if (scene.goal) {
      // Area for the center of an icon at the edge, so that the icon stays inside the minimap
      const double margin = border_width_px + goal_direction_icon_size_px / 2.0 + 2.0;
      paintGoal(painter, shape.shrunk(margin), to_offset(*scene.goal), projection, theme_icons);
    }
    paintVehicle(
      painter, shape, to_offset(*scene.vehicle), scene.vehicle_heading_deg + map_rotation_deg,
      projection, theme_icons);
    paintAttribution(painter, shape, theme_colors);

    const auto button_rect = [&](const MinimapButton button) {
      return centered_square(buttonCenter(button, rect, style.shape), button_size_px);
    };
    theme_icons.layers.render(&painter, button_rect(MinimapButton::Layers));
    theme_icons.zoom_out.render(&painter, button_rect(MinimapButton::ZoomOut));
    theme_icons.zoom_in.render(&painter, button_rect(MinimapButton::ZoomIn));
    if (scene.view_center) {
      theme_icons.recenter.render(&painter, button_rect(MinimapButton::Recenter));
    }
    // The red tip of the compass points to north
    painter.save();
    painter.translate(buttonCenter(MinimapButton::Compass, rect, style.shape));
    painter.rotate(map_rotation_deg);
    theme_icons.compass.render(&painter, centered_square({}, button_size_px));
    painter.restore();
  }

  // Half of the pen is outside the outline. The clip removes it.
  painter.setPen(QPen(theme_colors.border, 2.0 * border_width_px));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(shape_path);
  painter.restore();
}

void MinimapPainter::paintMessage(
  QPainter & painter, const QRectF & rect, const QString & text) const
{
  QFont font = painter.font();
  font.setPixelSize(12);
  painter.setFont(font);
  painter.setPen(mist);
  painter.drawText(rect, Qt::AlignCenter, text);
}

void MinimapPainter::paintRoute(
  QPainter & painter, const std::vector<QPointF> & offsets, const Projection & projection,
  const Colors & colors) const
{
  // The 3D view cannot show the points that are behind the viewer. Find where each segment
  // crosses the limit, by bisection, and draw the part in front of it.
  const auto visible_end = [&projection](QPointF visible, QPointF hidden) {
    for (int i = 0; i < 16; ++i) {
      const QPointF middle = (visible + hidden) / 2.0;
      (projection.project(middle) ? visible : hidden) = middle;
    }
    return *projection.project(visible);
  };

  QPainterPath line;
  for (std::size_t i = 0; i + 1 < offsets.size(); ++i) {
    const auto a = projection.project(offsets[i]);
    const auto b = projection.project(offsets[i + 1]);
    if (!a && !b) {
      continue;
    }
    const QPointF begin = a ? *a : visible_end(offsets[i + 1], offsets[i]);
    const QPointF end = b ? *b : visible_end(offsets[i], offsets[i + 1]);
    if (line.elementCount() == 0 || line.currentPosition() != begin) {
      line.moveTo(begin);
    }
    line.lineTo(end);
  }
  if (line.isEmpty()) {
    return;
  }

  painter.setBrush(Qt::NoBrush);
  // The casing keeps the line apart from the roads and the buildings of the map
  painter.setPen(
    QPen(colors.route_casing, route_casing_width_px, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawPath(line);
  painter.setPen(QPen(colors.route, route_width_px, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawPath(line);
}

void MinimapPainter::paintGoal(
  QPainter & painter, const Superellipse & area, const QPointF & offset,
  const Projection & projection, Icons & icons)
{
  const auto goal = projection.project(offset);
  // The pin shows only when its tip and its head are inside the minimap
  const QPointF tip_to_head(0.0, -0.6 * goal_icon_size_px);
  if (goal && area.contains(*goal) && area.contains(*goal + tip_to_head)) {
    // The tip of the pin is at the bottom center of the icon
    icons.goal.render(
      &painter, QRectF(
                  goal->x() - goal_icon_size_px / 2.0, goal->y() - goal_icon_size_px,
                  goal_icon_size_px, goal_icon_size_px));
    return;
  }

  // The goal is outside the minimap: show its direction at the edge.
  // A goal behind the viewer of the 3D view has no position, so use its position without tilt.
  const QPointF target = goal ? *goal : projection.projectFlat(offset);
  const QPointF direction = target - area.rect.center();
  // A far point in the direction is outside of the area for sure
  const double length = std::max(std::hypot(direction.x(), direction.y()), 1e-9);
  const double far = area.rect.width() + area.rect.height();
  painter.save();
  painter.translate(area.clamp(area.rect.center() + direction / length * far));
  // The icon points up
  painter.rotate(std::atan2(direction.x(), -direction.y()) * 180.0 / M_PI);
  icons.goal_direction.render(&painter, centered_square({}, goal_direction_icon_size_px));
  painter.restore();
}

void MinimapPainter::paintVehicle(
  QPainter & painter, const Superellipse & shape, const QPointF & offset, const double rotation_deg,
  const Projection & projection, Icons & icons)
{
  const auto position = projection.project(offset);
  if (!position || !shape.contains(*position)) {
    // The user moved the map away from the vehicle
    return;
  }
  painter.save();
  painter.translate(*position);
  // In the 3D view, the icon lies flat on the map
  painter.scale(1.0, projection.tilt_cos + 0.25 * projection.tilt_sin);
  // The icon points up
  painter.rotate(rotation_deg);
  icons.vehicle.render(&painter, centered_square({}, vehicle_icon_size_px));
  painter.restore();
}

void MinimapPainter::paintAttribution(
  QPainter & painter, const Superellipse & shape, const Colors & colors) const
{
  // The OpenStreetMap license asks for this text on the map
  QFont font = painter.font();
  font.setPixelSize(9);
  painter.setFont(font);
  const QFontMetricsF metrics(font);
  const QRectF & rect = shape.rect;

  for (const char * text : {"© OpenStreetMap contributors", "© OpenStreetMap", "© OSM"}) {
    const QSizeF size(metrics.horizontalAdvance(text) + 8.0, metrics.height() + 2.0);
    // Move the text up until the shape is wide enough for it
    double bottom_gap = border_width_px + 2.0;
    while (bottom_gap < rect.height() / 4.0 &&
           shape.halfWidthAboveBottom(bottom_gap) - border_width_px - 2.0 < size.width() / 2.0) {
      bottom_gap += 1.0;
    }
    if (bottom_gap >= rect.height() / 4.0) {
      // This text does not fit in the lower part of the minimap. Try the shorter text.
      continue;
    }

    const QRectF text_rect(
      rect.center().x() - size.width() / 2.0, rect.bottom() - bottom_gap - size.height(),
      size.width(), size.height());
    QColor background = colors.label_background;
    background.setAlpha(200);
    painter.setPen(Qt::NoPen);
    painter.setBrush(background);
    painter.drawRoundedRect(text_rect, text_rect.height() / 2.0, text_rect.height() / 2.0);
    painter.setPen(colors.label_text);
    painter.drawText(text_rect, Qt::AlignCenter, text);
    return;
  }
}

}  // namespace autoware_minimap_overlay_rviz_plugin
