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

#include "tile_directory.hpp"
#include "tile_loader.hpp"
#include "web_mercator.hpp"

#include <QImage>
#include <QPainter>

#include <gtest/gtest.h>

using autoware_minimap_overlay_rviz_plugin::apply_map_style;
using autoware_minimap_overlay_rviz_plugin::MapStyle;
using autoware_minimap_overlay_rviz_plugin::TileLoader;

namespace
{
// Center of the tile x, y at a zoom level
QPointF tile_center(const int zoom, const int x, const int y)
{
  return QPointF(x + 0.5, y + 0.5) / (1 << zoom);
}

QImage paint_tiles(TileLoader & loader, const QSize & size)
{
  QImage image(size, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  loader.paint(painter, image.rect());
  return image;
}
}  // namespace

TEST(TileLoader, LoadsAndDrawsTheTilesOfTheView)
{
  // The view is centered on the tile 4/8/5 and reaches into its 8 neighbors
  TileDirectory directory;
  for (int x = 7; x <= 9; ++x) {
    for (int y = 4; y <= 6; ++y) {
      directory.addTile(4, x, y, x == 8 && y == 5 ? Qt::red : Qt::blue);
    }
  }
  TileLoader loader(directory.urlTemplate());
  int loaded = 0;
  QObject::connect(&loader, &TileLoader::tileLoaded, [&loaded]() { ++loaded; });

  loader.setView(tile_center(4, 8, 5), 4, QSizeF(300.0, 300.0));
  ASSERT_TRUE(wait_for([&loaded]() { return loaded == 9; }));

  const QImage image = paint_tiles(loader, QSize(300, 300));
  EXPECT_EQ(image.pixelColor(150, 150), QColor(Qt::red));
  EXPECT_EQ(image.pixelColor(23, 150), QColor(Qt::red));
  EXPECT_EQ(image.pixelColor(21, 150), QColor(Qt::blue));
  EXPECT_EQ(image.pixelColor(150, 280), QColor(Qt::blue));
  EXPECT_EQ(image.pixelColor(299, 299), QColor(Qt::blue));
}

TEST(TileLoader, DrawsNothingBeforeTheTilesArrive)
{
  TileDirectory directory;
  TileLoader loader(directory.urlTemplate());

  EXPECT_EQ(paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32).alpha(), 0);
  loader.setView(tile_center(4, 8, 5), 4, QSizeF(64.0, 64.0));
  EXPECT_EQ(paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32).alpha(), 0);
}

TEST(TileLoader, ReportsATileThatDoesNotLoad)
{
  TileDirectory directory;
  TileLoader loader(directory.urlTemplate());
  QString reason;
  QObject::connect(
    &loader, &TileLoader::tileFailed, [&reason](const QString & text) { reason = text; });

  loader.setView(tile_center(4, 8, 5), 4, QSizeF(64.0, 64.0));
  ASSERT_TRUE(wait_for([&reason]() { return !reason.isEmpty(); }));
}

TEST(TileLoader, LoadsAFailedTileAgainLater)
{
  TileDirectory directory;
  TileLoader loader(directory.urlTemplate());
  bool failed = false;
  bool loaded = false;
  QObject::connect(&loader, &TileLoader::tileFailed, [&failed]() { failed = true; });
  QObject::connect(&loader, &TileLoader::tileLoaded, [&loaded]() { loaded = true; });

  loader.setView(tile_center(4, 8, 5), 4, QSizeF(64.0, 64.0));
  ASSERT_TRUE(wait_for([&failed]() { return failed; }));

  // The first retry comes 2 seconds after the failure, with no new call of setView()
  directory.addTile(4, 8, 5, Qt::green);
  ASSERT_TRUE(wait_for([&loaded]() { return loaded; }, 6000));
  EXPECT_EQ(paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32), QColor(Qt::green));
}

TEST(TileLoader, DropsTheTilesOfAnotherZoomLevel)
{
  TileDirectory directory;
  directory.addTile(4, 8, 5, Qt::red);
  TileLoader loader(directory.urlTemplate());
  bool loaded = false;
  QObject::connect(&loader, &TileLoader::tileLoaded, [&loaded]() { loaded = true; });

  loader.setView(tile_center(4, 8, 5), 4, QSizeF(64.0, 64.0));
  ASSERT_TRUE(wait_for([&loaded]() { return loaded; }));
  EXPECT_EQ(paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32), QColor(Qt::red));

  loader.setView(tile_center(4, 8, 5), 5, QSizeF(64.0, 64.0));
  EXPECT_EQ(paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32).alpha(), 0);
}

TEST(TileLoader, MapStyleChangesTheColorsOfTheTiles)
{
  TileDirectory directory;
  directory.addTile(4, 8, 5, Qt::white);
  TileLoader loader(directory.urlTemplate());
  int loaded = 0;
  QObject::connect(&loader, &TileLoader::tileLoaded, [&loaded]() { ++loaded; });
  loader.setStyle(MapStyle::NavigationDark);

  loader.setView(tile_center(4, 8, 5), 4, QSizeF(64.0, 64.0));
  ASSERT_TRUE(wait_for([&loaded]() { return loaded == 1; }));
  EXPECT_EQ(
    paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32),
    apply_map_style(QImage(directory.tilePath(4, 8, 5)), MapStyle::NavigationDark)
      .pixelColor(0, 0));

  // With the standard style, the tile loads again and shows its own color
  loaded = 0;
  loader.setStyle(MapStyle::Standard);
  ASSERT_TRUE(wait_for([&loaded]() { return loaded == 1; }));
  EXPECT_EQ(paint_tiles(loader, QSize(64, 64)).pixelColor(32, 32), QColor(Qt::white));
}

TEST(TileLoader, WrapsTheColumnsAtTheEdgeOfTheMap)
{
  // At zoom level 2 the map has 4 columns. The view at the west edge shows column 3 on its left.
  TileDirectory directory;
  directory.addTile(2, 3, 1, Qt::red);
  directory.addTile(2, 0, 1, Qt::blue);
  TileLoader loader(directory.urlTemplate());
  int loaded = 0;
  QObject::connect(&loader, &TileLoader::tileLoaded, [&loaded]() { ++loaded; });

  loader.setView(QPointF(0.0, 1.5 / 4.0), 2, QSizeF(100.0, 100.0));
  ASSERT_TRUE(wait_for([&loaded]() { return loaded == 2; }));

  const QImage image = paint_tiles(loader, QSize(100, 100));
  EXPECT_EQ(image.pixelColor(25, 50), QColor(Qt::red));
  EXPECT_EQ(image.pixelColor(75, 50), QColor(Qt::blue));
}
