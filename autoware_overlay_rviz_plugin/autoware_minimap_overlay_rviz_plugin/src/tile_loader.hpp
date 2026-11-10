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

#ifndef TILE_LOADER_HPP_
#define TILE_LOADER_HPP_

#include "map_style.hpp"

#include <QImage>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <QPainter>
#include <QPointF>
#include <QPointer>
#include <QRectF>
#include <QSizeF>
#include <QString>
#include <QTimer>

#include <chrono>
#include <map>
#include <tuple>

namespace autoware_minimap_overlay_rviz_plugin
{

// Downloads the map tiles that a view needs and draws them.
// All calls must come from the thread of the object. In RViz, this is the GUI thread.
class TileLoader : public QObject
{
  Q_OBJECT

public:
  // url_template has the placeholders {z}, {x} and {y}.
  // An empty cache_directory turns off the disk cache.
  explicit TileLoader(
    const QString & url_template, const QString & cache_directory = QString(),
    QObject * parent = nullptr);
  ~TileLoader() override;

  // Sets the map position that is at the center of the view, as returned by to_mercator().
  // Requests the tiles that are not loaded and drops the tiles that are far from the view.
  void setView(const QPointF & center, int zoom, const QSizeF & size_px);

  // Sets the colors of the tiles. A change drops the loaded tiles.
  void setStyle(MapStyle style);

  // Draws the loaded tiles of the view, with the center of the view at the center of `target`.
  // snap_to_pixels keeps the tiles sharp. Turn it off when the painter rotates.
  void paint(QPainter & painter, const QRectF & target, bool snap_to_pixels = true);

Q_SIGNALS:
  void tileLoaded();
  void tileFailed(const QString & reason);

private:
  struct TileId
  {
    int zoom;
    int x;
    int y;
    bool operator<(const TileId & other) const
    {
      return std::tie(zoom, x, y) < std::tie(other.zoom, other.x, other.y);
    }
  };

  struct Tile
  {
    QImage image;
    QPointer<QNetworkReply> reply;
    int failures = 0;
    std::chrono::steady_clock::time_point retry_time{};
  };

  // Tile columns and rows that the view covers. The columns are not wrapped at the antimeridian.
  struct TileRange
  {
    int x_min;
    int x_max;
    int y_min;
    int y_max;
    bool operator==(const TileRange & other) const
    {
      return std::tie(x_min, x_max, y_min, y_max) ==
             std::tie(other.x_min, other.x_max, other.y_min, other.y_max);
    }
  };

  TileRange visibleRange() const;
  void requestMissingTiles();
  void dropFarTiles();
  void dropAllTiles();
  void requestTile(const TileId & id, Tile & tile);
  void onReplyFinished(const TileId & id, QNetworkReply * reply);

  QString url_template_;
  QNetworkAccessManager network_;
  QTimer retry_timer_;
  std::map<TileId, Tile> tiles_;

  bool has_view_ = false;
  QPointF center_px_;
  int zoom_ = 0;
  QSizeF size_px_;
  MapStyle style_ = MapStyle::Standard;

  // Image of the tiles of the view. One image has no seams between the tiles when it is rotated.
  QImage mosaic_;
  TileRange mosaic_range_{};
  bool mosaic_dirty_ = true;
};

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // TILE_LOADER_HPP_
