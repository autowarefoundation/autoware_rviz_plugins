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
#ifndef TILE_FIELD_HPP_
#define TILE_FIELD_HPP_

#include <QImage>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QObject>
#include <QPointF>
#include <QPointer>

#include <chrono>
#include <map>
#include <string>
#include <tuple>

// Downloads the 3 x 3 map tiles around a center tile and keeps them in memory
class TileField : public QObject
{
  Q_OBJECT

public:
  explicit TileField(QObject * parent = nullptr);
  ~TileField() override;

  void setUrlTemplate(const std::string & url_template);
  // Make the tile under the given position the center tile, and fetch the missing tiles
  void setCenter(double lat, double lon, int zoom);
  // Image of the 3 x 3 tiles; tiles that did not arrive yet are transparent
  QImage getTileFieldImage();
  // Position of the top left corner of the tile field image in Web Mercator world pixels
  QPointF getTileFieldOrigin() const;

Q_SIGNALS:
  void tilesUpdated();

private Q_SLOTS:
  void onReplyFinished(QNetworkReply * reply);

private:
  using TileKey = std::tuple<int, int, int>;  // zoom, x, y

  struct Tile
  {
    QImage image;
    QPointer<QNetworkReply> reply;
    int failures = 0;
    std::chrono::steady_clock::time_point next_retry{};
  };

  void fetchMissingTiles();
  void evictFarTiles();
  void requestTile(const TileKey & key, Tile & tile);
  void abortAll();

  QNetworkAccessManager * network_manager_;
  std::map<TileKey, Tile> tiles_;
  std::string url_template_;
  int zoom_ = 0;
  int center_x_tile_ = 0;
  int center_y_tile_ = 0;
  bool has_center_ = false;

  QImage tile_field_image_;
  bool image_dirty_ = true;
};

#endif  // TILE_FIELD_HPP_
