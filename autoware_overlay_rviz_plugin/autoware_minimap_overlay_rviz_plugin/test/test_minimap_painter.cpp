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
#include "tile_directory.hpp"
#include "web_mercator.hpp"

#include <QImage>
#include <QPainter>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

using autoware_minimap_overlay_rviz_plugin::map_size_px;
using autoware_minimap_overlay_rviz_plugin::MapStyle;
using autoware_minimap_overlay_rviz_plugin::MinimapButton;
using autoware_minimap_overlay_rviz_plugin::MinimapPainter;
using autoware_minimap_overlay_rviz_plugin::MinimapScene;
using autoware_minimap_overlay_rviz_plugin::MinimapShape;
using autoware_minimap_overlay_rviz_plugin::MinimapStyle;
using autoware_minimap_overlay_rviz_plugin::MinimapView;
using autoware_minimap_overlay_rviz_plugin::TileLoader;
using autoware_minimap_overlay_rviz_plugin::to_mercator;

namespace
{
constexpr int size_px = 256;
constexpr int zoom = 17;
const QRectF minimap_rect(0.0, 0.0, size_px, size_px);
const QPoint image_center(size_px / 2, size_px / 2);
const QColor navy(0, 0, 42);
const QColor charcoal(23, 28, 31);
const QColor indigo(63, 46, 227);
const QColor green(0, 230, 120);
// Land color of the standard OpenStreetMap style
const QColor tile_color(242, 239, 233);

// Position that is the given number of map pixels east and south of the vehicle
QPointF offset_px(const QPointF & vehicle, const double east, const double south)
{
  return vehicle + QPointF(east, south) / map_size_px(zoom);
}

// True if the color is in the square of 7 x 7 pixels around the position
bool has_color_near(const QImage & image, const QPoint & position, const QColor & color)
{
  for (int x = position.x() - 3; x <= position.x() + 3; ++x) {
    for (int y = position.y() - 3; y <= position.y() + 3; ++y) {
      if (image.rect().contains(x, y) && image.pixelColor(x, y) == color) {
        return true;
      }
    }
  }
  return false;
}

// The dark body of the vehicle icon of the light theme, with its smooth edge
bool is_dark(const QColor & color)
{
  return color.lightness() < 100;
}

class MinimapPainterTest : public testing::Test
{
protected:
  void SetUp() override
  {
    style_.shape = MinimapShape::Circle;
    style_.map = MapStyle::Standard;
    style_.view = MinimapView::NorthUp;
    scene_.zoom = zoom;
    scene_.vehicle = to_mercator(35.9024281, 139.9326469);

    // The vehicle is in the tile 116483, 51513. The 3D view reaches some tiles more.
    for (int x = 116483 - 3; x <= 116483 + 3; ++x) {
      for (int y = 51513 - 3; y <= 51513 + 3; ++y) {
        directory_.addTile(zoom, x, y, tile_color);
      }
    }
    // A large minimap uses the tiles of the next zoom level
    for (int x = 2 * 116483 - 2; x <= 2 * 116483 + 3; ++x) {
      for (int y = 2 * 51513 - 2; y <= 2 * 51513 + 3; ++y) {
        directory_.addTile(zoom + 1, x, y, tile_color);
      }
    }
    tiles_ = std::make_unique<TileLoader>(directory_.urlTemplate());
  }

  QImage paint()
  {
    // Load the tiles of the view, and wait until no more tile arrives
    int loaded = 0;
    const auto connection =
      QObject::connect(tiles_.get(), &TileLoader::tileLoaded, [&loaded]() { ++loaded; });
    tiles_->setStyle(style_.map);
    tiles_->setView(
      scene_.view_center.value_or(*scene_.vehicle), MinimapPainter::tileZoom(rect_, zoom),
      MinimapPainter::tileViewSize(rect_, style_, zoom));
    wait_for([&loaded, last = -1]() mutable {
      const bool settled = loaded == last;
      last = loaded;
      return settled;
    });
    QObject::disconnect(connection);

    QImage image(rect_.size().toSize(), QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    QPainter painter(&image);
    painter_.paint(painter, rect_, style_, scene_, tiles_.get());
    painter.end();

    // Set this variable to look at the images of the tests
    const QString output_directory = qEnvironmentVariable("MINIMAP_TEST_OUTPUT_DIRECTORY");
    if (!output_directory.isEmpty()) {
      const auto * test = testing::UnitTest::GetInstance()->current_test_info();
      image.save(QString("%1/%2_%3.png").arg(output_directory, test->name()).arg(image_count_++));
    }
    return image;
  }

  MinimapPainter painter_{ICON_DIRECTORY};
  MinimapStyle style_;
  MinimapScene scene_;
  QRectF rect_ = minimap_rect;
  TileDirectory directory_;
  std::unique_ptr<TileLoader> tiles_;
  int image_count_ = 0;
};
}  // namespace

TEST_F(MinimapPainterTest, IconFilesLoad)
{
  EXPECT_TRUE(painter_.isValid());
  EXPECT_FALSE(MinimapPainter("/directory/that/does/not/exist").isValid());
}

TEST_F(MinimapPainterTest, ShapeGivesTheOutline)
{
  // A point near the corner is outside the circle, but inside the squircle and the rectangle
  const QPoint near_corner(size_px - 30, size_px - 30);
  const QPoint corner(size_px - 3, size_px - 3);

  style_.shape = MinimapShape::Circle;
  const QImage circle = paint();
  EXPECT_EQ(circle.pixelColor(near_corner).alpha(), 0);
  EXPECT_EQ(circle.pixelColor(corner).alpha(), 0);
  // Border at the left end of the circle, and the map beside it
  EXPECT_EQ(circle.pixelColor(1, size_px / 2), charcoal);
  EXPECT_EQ(circle.pixelColor(5, size_px / 2), tile_color);

  style_.shape = MinimapShape::Squircle;
  const QImage squircle = paint();
  EXPECT_EQ(squircle.pixelColor(near_corner).alpha(), 255);
  EXPECT_EQ(squircle.pixelColor(corner).alpha(), 0);

  style_.shape = MinimapShape::Rectangle;
  const QImage rectangle = paint();
  EXPECT_EQ(rectangle.pixelColor(corner).alpha(), 255);
  EXPECT_EQ(rectangle.pixelColor(size_px - 1, size_px - 1), charcoal);
}

TEST_F(MinimapPainterTest, StandardStyleShowsTheTilesAsTheyAre)
{
  EXPECT_EQ(paint().pixelColor(60, 150), tile_color);
}

TEST_F(MinimapPainterTest, NavigationStylesChangeTheColorsOfTheTiles)
{
  style_.map = MapStyle::NavigationDark;
  EXPECT_LT(paint().pixelColor(60, 150).lightness(), 80);
  style_.map = MapStyle::NavigationLight;
  const QColor light = paint().pixelColor(60, 150);
  EXPECT_GT(light.lightness(), 200);
  EXPECT_NE(light, tile_color);
}

TEST_F(MinimapPainterTest, VehicleIconTurnsWithTheHeadingWhenNorthIsUp)
{
  // The icon is narrow: 6 pixels ahead of the image_center are inside it, 6 pixels to the side are
  // not
  const QPoint north_of_center = image_center + QPoint(1, -6);
  const QPoint east_of_center = image_center + QPoint(6, 1);

  scene_.vehicle_heading_deg = 0.0;
  const QImage north = paint();
  EXPECT_TRUE(is_dark(north.pixelColor(north_of_center)));
  EXPECT_FALSE(is_dark(north.pixelColor(east_of_center)));

  scene_.vehicle_heading_deg = 90.0;
  const QImage east = paint();
  EXPECT_TRUE(is_dark(east.pixelColor(east_of_center)));
  EXPECT_FALSE(is_dark(east.pixelColor(north_of_center)));
}

TEST_F(MinimapPainterTest, HeadingUpTurnsTheMapAndNotTheVehicleIcon)
{
  // The vehicle drives east, and the route goes 60 pixels east on the map
  style_.view = MinimapView::HeadingUp;
  scene_.vehicle_heading_deg = 90.0;
  scene_.route = {*scene_.vehicle, offset_px(*scene_.vehicle, 60.0, 0.0)};
  const QImage image = paint();

  // The icon points up, and east is up
  EXPECT_TRUE(is_dark(image.pixelColor(image_center + QPoint(1, -6))));
  EXPECT_EQ(image.pixelColor(image_center + QPoint(0, -45)), indigo);
  EXPECT_NE(image.pixelColor(image_center + QPoint(45, 0)), indigo);
}

TEST_F(MinimapPainterTest, RouteIsDrawnRelativeToTheVehicle)
{
  // A line from the vehicle to 80 pixels east and 40 pixels north
  scene_.route = {*scene_.vehicle, offset_px(*scene_.vehicle, 80.0, -40.0)};
  const QImage image = paint();
  EXPECT_EQ(image.pixelColor(image_center + QPoint(60, -30)), indigo);
  EXPECT_EQ(image.pixelColor(image_center + QPoint(60, 30)), tile_color);
}

TEST_F(MinimapPainterTest, DarkStyleDrawsTheRouteInGreen)
{
  style_.map = MapStyle::NavigationDark;
  scene_.route = {*scene_.vehicle, offset_px(*scene_.vehicle, 80.0, -40.0)};
  EXPECT_EQ(paint().pixelColor(image_center + QPoint(60, -30)), green);
}

TEST_F(MinimapPainterTest, GoalInsideTheMinimapShowsThePin)
{
  // The head of the pin is above the goal position
  scene_.goal = offset_px(*scene_.vehicle, 50.0, 60.0);
  EXPECT_TRUE(has_color_near(paint(), image_center + QPoint(50, 60 - 8), indigo));
}

TEST_F(MinimapPainterTest, GoalOutsideTheMinimapShowsItsDirectionAtTheEdge)
{
  // The goal is 2000 pixels east. The direction icon is at the right end of the circle.
  scene_.goal = offset_px(*scene_.vehicle, 2000.0, 0.0);
  const QImage image = paint();
  EXPECT_TRUE(has_color_near(image, QPoint(size_px - 22, size_px / 2 - 6), indigo));
  EXPECT_FALSE(has_color_near(image, QPoint(22, size_px / 2 - 6), indigo));
}

TEST_F(MinimapPainterTest, PerspectiveShowsTheVehicleBelowTheCenterAndTheRouteAhead)
{
  // The vehicle drives north, and the route goes 300 pixels north on the map
  style_.view = MinimapView::Perspective;
  scene_.route = {*scene_.vehicle, offset_px(*scene_.vehicle, 0.0, -300.0)};
  scene_.goal = offset_px(*scene_.vehicle, 0.0, -300.0);
  const QImage image = paint();

  // The vehicle is at 70 % of the height
  const QPoint vehicle(size_px / 2, std::lround(0.7 * size_px));
  EXPECT_TRUE(is_dark(image.pixelColor(vehicle + QPoint(1, -3))));
  // The route goes up from the vehicle, and its far part is shorter on the minimap than on the map
  EXPECT_EQ(image.pixelColor(vehicle + QPoint(0, -60)), indigo);
  EXPECT_NE(image.pixelColor(size_px / 2, 10), indigo);
  int route_top = size_px;
  for (int y = 0; y < size_px; ++y) {
    if (image.pixelColor(size_px / 2 + 3, y) == indigo) {
      route_top = y;
      break;
    }
  }
  EXPECT_GT(route_top, vehicle.y() - 200);
  EXPECT_LT(route_top, vehicle.y() - 100);
}

TEST_F(MinimapPainterTest, PerspectiveDrawsARouteThatStartsBehindTheViewer)
{
  // The route starts 2000 pixels behind the vehicle
  style_.view = MinimapView::Perspective;
  scene_.route = {offset_px(*scene_.vehicle, 0.0, 2000.0), offset_px(*scene_.vehicle, 0.0, -100.0)};
  scene_.goal = offset_px(*scene_.vehicle, 0.0, 2000.0);
  const QImage image = paint();
  const QPoint vehicle(size_px / 2, std::lround(0.7 * size_px));
  EXPECT_EQ(image.pixelColor(vehicle + QPoint(0, -40)), indigo);
  // The goal is behind the viewer, so its direction shows at the bottom
  EXPECT_TRUE(has_color_near(image, QPoint(size_px / 2 - 6, size_px - 22), indigo));
}

TEST_F(MinimapPainterTest, MovedViewShowsTheVehicleAwayFromTheCenter)
{
  // The view image_center is 50 pixels east of the vehicle, so the vehicle is 50 pixels left
  scene_.view_center = offset_px(*scene_.vehicle, 50.0, 0.0);
  const QImage image = paint();
  EXPECT_TRUE(is_dark(image.pixelColor(image_center + QPoint(-50 + 1, -6))));
  EXPECT_FALSE(is_dark(image.pixelColor(image_center + QPoint(1, -6))));
}

TEST_F(MinimapPainterTest, ButtonsAreInTheCornersAndAtTheBottom)
{
  for (const MinimapShape shape :
       {MinimapShape::Squircle, MinimapShape::Circle, MinimapShape::Rectangle}) {
    style_.shape = shape;
    const auto button_center = [shape](const MinimapButton button) {
      return MinimapPainter::buttonCenter(button, minimap_rect, shape);
    };
    const QPointF layers = button_center(MinimapButton::Layers);
    const QPointF compass = button_center(MinimapButton::Compass);
    const QPointF zoom_out = button_center(MinimapButton::ZoomOut);
    const QPointF zoom_in = button_center(MinimapButton::ZoomIn);
    const QPointF recenter = button_center(MinimapButton::Recenter);

    EXPECT_LT(layers.x(), size_px / 2);
    EXPECT_LT(layers.y(), size_px / 2);
    EXPECT_GT(compass.x(), size_px / 2);
    EXPECT_LT(compass.y(), size_px / 2);
    EXPECT_LT(zoom_out.x(), size_px / 2);
    EXPECT_GT(zoom_out.y(), size_px / 2);
    EXPECT_GT(zoom_in.x(), size_px / 2);
    EXPECT_GT(zoom_in.y(), size_px / 2);
    EXPECT_DOUBLE_EQ(recenter.x(), size_px / 2);
    EXPECT_GT(recenter.y(), size_px / 2);

    // Each button is inside the outline, with its radius of 13 pixels
    for (const QPointF & button : {layers, compass, zoom_out, zoom_in, recenter}) {
      EXPECT_TRUE(MinimapPainter::outline(minimap_rect, shape).shrunk(13.0).contains(button));
    }

    const auto button_at = [this](const QPointF & position, const bool has_recenter_button) {
      return MinimapPainter::buttonAt(position, minimap_rect, style_, has_recenter_button);
    };
    EXPECT_EQ(button_at(layers, false), MinimapButton::Layers);
    EXPECT_EQ(button_at(compass, false), MinimapButton::Compass);
    EXPECT_EQ(button_at(zoom_out + QPointF(5.0, 5.0), false), MinimapButton::ZoomOut);
    EXPECT_EQ(button_at(zoom_in, false), MinimapButton::ZoomIn);
    EXPECT_EQ(button_at(minimap_rect.center(), true), std::nullopt);
    // The recenter button is there only when the view moved
    EXPECT_EQ(button_at(recenter, false), std::nullopt);
    EXPECT_EQ(button_at(recenter, true), MinimapButton::Recenter);
  }
}

TEST_F(MinimapPainterTest, RecenterButtonShowsOnlyWhenTheViewMoved)
{
  // A point in the upper part of the button, between its symbol and its ring. The attribution
  // text is below the button, and its height depends on the font of the system.
  const QPoint in_button =
    MinimapPainter::buttonCenter(MinimapButton::Recenter, minimap_rect, style_.shape).toPoint() +
    QPoint(0, -10);
  EXPECT_EQ(paint().pixelColor(in_button), tile_color);

  // The button of the light styles is white, with a dark symbol
  scene_.view_center = offset_px(*scene_.vehicle, 20.0, 0.0);
  EXPECT_GT(paint().pixelColor(in_button).lightness(), 250);
}

TEST_F(MinimapPainterTest, TileViewCoversTheMinimap)
{
  // With north up, the tiles cover the rectangle. A rotated map needs its diagonal.
  style_.view = MinimapView::NorthUp;
  EXPECT_NEAR(
    MinimapPainter::tileViewSize(minimap_rect, style_, zoom).width(), size_px * M_SQRT2, 1e-6);
  style_.view = MinimapView::HeadingUp;
  EXPECT_NEAR(
    MinimapPainter::tileViewSize(minimap_rect, style_, zoom).width(), size_px * M_SQRT2, 1e-6);
  // The 3D view shows more of the map, up to its limit
  style_.view = MinimapView::Perspective;
  EXPECT_NEAR(
    MinimapPainter::tileViewSize(minimap_rect, style_, zoom).width(), 3.2 * size_px, 1e-6);
}

TEST_F(MinimapPainterTest, LargerMinimapShowsTheSameDesignLarger)
{
  // A minimap of 2 times the size: the route and the buttons are at 2 times the distance
  rect_ = QRectF(0.0, 0.0, 2 * size_px, 2 * size_px);
  scene_.route = {*scene_.vehicle, offset_px(*scene_.vehicle, 80.0, -40.0)};
  const QImage image = paint();
  EXPECT_EQ(image.pixelColor(2 * (image_center + QPoint(60, -30))), indigo);
  EXPECT_EQ(image.pixelColor(2 * (image_center + QPoint(60, 30))), tile_color);

  EXPECT_DOUBLE_EQ(MinimapPainter::designScale(rect_), 2.0);
  EXPECT_EQ(
    MinimapPainter::buttonCenter(MinimapButton::Layers, rect_, style_.shape),
    2.0 * MinimapPainter::buttonCenter(MinimapButton::Layers, minimap_rect, style_.shape));
  // It uses the tiles of the next zoom level, which have 2 times the pixels
  EXPECT_EQ(MinimapPainter::tileZoom(rect_, zoom), zoom + 1);
  EXPECT_NEAR(
    MinimapPainter::tileViewSize(rect_, style_, zoom).width(), 2 * size_px * M_SQRT2, 1e-6);
  // The tile server has no tiles above zoom level 19
  EXPECT_EQ(MinimapPainter::tileZoom(rect_, 19), 19);
  EXPECT_EQ(MinimapPainter::tileZoom(minimap_rect, zoom), zoom);
}

TEST_F(MinimapPainterTest, MessageShowsWithoutVehiclePosition)
{
  scene_.vehicle.reset();
  style_.shape = MinimapShape::Rectangle;
  QImage image(size_px, size_px, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter_.paint(painter, minimap_rect, style_, scene_, tiles_.get());

  EXPECT_EQ(image.pixelColor(10, 10), navy);
  EXPECT_EQ(image.pixelColor(size_px - 10, size_px - 10), navy);
  // The text is at the center
  int text_pixels = 0;
  for (int x = 60; x < size_px - 60; ++x) {
    for (int y = size_px / 2 - 20; y < size_px / 2 + 20; ++y) {
      text_pixels += image.pixelColor(x, y) != navy ? 1 : 0;
    }
  }
  EXPECT_GT(text_pixels, 50);
}

TEST_F(MinimapPainterTest, PaintsWithoutTiles)
{
  QImage image(size_px, size_px, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter_.paint(painter, minimap_rect, style_, scene_, nullptr);
  EXPECT_EQ(image.pixelColor(60, 150).alpha(), 255);
}
