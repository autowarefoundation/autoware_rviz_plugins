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

#ifndef MINIMAP_PAINTER_HPP_
#define MINIMAP_PAINTER_HPP_

#include "superellipse.hpp"
#include "tile_loader.hpp"
#include "web_mercator.hpp"

#include <QColor>
#include <QImage>
#include <QPainter>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QSvgRenderer>
#include <QTransform>

#include <optional>
#include <vector>

namespace autoware_minimap_overlay_rviz_plugin
{

// What the minimap shows. All positions are on the Web Mercator map, as returned by to_mercator().
struct MinimapScene
{
  int zoom = 17;
  // Without a vehicle position, the minimap shows a message in place of the map
  std::optional<QPointF> vehicle;
  // Text of that message
  QString waiting_message = "Waiting for the\nvehicle position";
  // Clockwise from north
  double vehicle_heading_deg = 0.0;
  // Map position at the center of the view. Without it, the view follows the vehicle.
  std::optional<QPointF> view_center;
  std::optional<QPointF> goal;
  // Line from the vehicle to the goal
  std::vector<QPointF> route;
};

enum class MinimapShape { Squircle, Circle, Rectangle };
// NorthUp: north is at the top. HeadingUp: the heading of the vehicle is at the top.
// Perspective: as HeadingUp, and the map is tilted away from the viewer.
enum class MinimapView { Perspective, HeadingUp, NorthUp };
enum class MinimapButton { Layers, Compass, ZoomOut, ZoomIn, Recenter };

// How the minimap looks
struct MinimapStyle
{
  MinimapShape shape = MinimapShape::Squircle;
  MapStyle map = MapStyle::NavigationDark;
  MinimapView view = MinimapView::Perspective;
};

// Draws the minimap, and tells where its parts are
class MinimapPainter
{
public:
  // icon_directory holds the directories `dark` and `light` with the SVG files of the icons
  explicit MinimapPainter(const QString & icon_directory);

  // False if an icon file did not load
  bool isValid() const;

  // Draws the minimap in the rectangle, and makes the part of the rectangle outside of the shape
  // transparent. The design is for a minimap of 256 pixels. A larger or
  // smaller rectangle shows the same design and the same part of the map, larger or smaller.
  // tiles can be null. Then the map background stays empty.
  // The tiles must have the view of tileZoom() and tileViewSize(), and the map style of the style.
  void paint(
    QPainter & painter, const QRectF & rect, const MinimapStyle & style, const MinimapScene & scene,
    TileLoader * tiles);

  // Size of the design, as a ratio of the design for 256 pixels
  static double designScale(const QRectF & rect);
  // Zoom level of the tiles for a minimap at a zoom level. A large minimap uses the tiles of a
  // higher zoom level, so that they stay sharp. A minimap that shows a very large part of the map,
  // such as a long and narrow one, uses the tiles of a lower zoom level. This keeps the view of
  // the tiles below max_tile_view_px.
  static int tileZoom(const QRectF & rect, const MinimapStyle & style, int zoom);
  // Size of the square of map tiles that the minimap needs, in pixels of the tiles of tileZoom()
  static QSizeF tileViewSize(const QRectF & rect, const MinimapStyle & style, int zoom);
  // Upper limit of tileViewSize(). One view then needs at most 8 x 8 tiles.
  static constexpr double max_tile_view_px = 7.0 * tile_size_px;

  // Outline of the minimap in the rectangle
  static Superellipse outline(const QRectF & rect, MinimapShape shape);

  // Button at a position. The layers, compass and zoom buttons are in the corners. The recenter
  // button is at the bottom, and only there when the view does not follow the vehicle.
  static std::optional<MinimapButton> buttonAt(
    const QPointF & position, const QRectF & rect, const MinimapStyle & style,
    bool has_recenter_button);
  static QPointF buttonCenter(MinimapButton button, const QRectF & rect, MinimapShape shape);

  // Rotation of the map on the minimap, clockwise
  static double mapRotation(const MinimapStyle & style, const MinimapScene & scene);

private:
  struct Icons
  {
    explicit Icons(const QString & directory);
    bool isValid() const;
    QSvgRenderer vehicle;
    QSvgRenderer goal;
    QSvgRenderer goal_direction;
    QSvgRenderer layers;
    QSvgRenderer compass;
    QSvgRenderer recenter;
    QSvgRenderer zoom_in;
    QSvgRenderer zoom_out;
  };

  struct Colors
  {
    QColor route;
    QColor route_casing;
    QColor label_background;
    QColor label_text;
    // Background of the minimap when it has no map to show
    QColor panel;
  };

  // Places the map on the minimap: rotation around the view center, and the tilt of the 3D view
  struct Projection
  {
    Projection(const QRectF & rect, MinimapView view, double map_rotation_deg);

    // Position on the minimap of an offset from the view center, in map pixels with north up.
    // No value if the offset is behind the viewer of the 3D view.
    std::optional<QPointF> project(const QPointF & offset) const;
    // Position of the offset on a minimap that is rotated but not tilted
    QPointF projectFlat(const QPointF & offset) const;

    // Position of the view center on the minimap
    QPointF anchor;
    QTransform rotation;
    double tilt_sin = 0.0;
    double tilt_cos = 1.0;
    // Distance from the viewer to the view center, in map pixels
    double distance = 1.0;
    // For the painter of the tiles: from the minimap with north up and the view center at the
    // center of the rectangle, to the minimap as it shows
    QTransform tile_transform;
  };

  // Distance from the view center that the tiles must cover, in map pixels of the zoom level of
  // the minimap
  static double viewRadius(const QRectF & device_rect, const MinimapStyle & style);
  static const Colors & colors(MapStyle map);
  Icons & icons(MapStyle map);

  void paintMessage(QPainter & painter, const QRectF & rect, const QString & text) const;
  void paintRoute(
    QPainter & painter, const std::vector<QPointF> & offsets, const Projection & projection,
    const Colors & colors) const;
  // `area` is where the center of an icon at the edge of the minimap can be
  void paintGoal(
    QPainter & painter, const Superellipse & area, const QPointF & offset,
    const Projection & projection, Icons & icons);
  void paintVehicle(
    QPainter & painter, const Superellipse & shape, const QPointF & offset, double rotation_deg,
    const Projection & projection, Icons & icons);
  // `scale` is the size of one unit of the painter in pixels
  void paintAttribution(
    QPainter & painter, const Superellipse & shape, const Colors & colors, double scale) const;

  Icons dark_icons_;
  Icons light_icons_;
  // Buffer for the rotated or tilted map
  QImage map_layer_;
};

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // MINIMAP_PAINTER_HPP_
