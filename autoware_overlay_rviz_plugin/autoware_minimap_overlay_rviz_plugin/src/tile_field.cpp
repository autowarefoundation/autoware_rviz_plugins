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
#include "include/tile_field.hpp"

#include "include/tile_provider.hpp"
#include "include/web_mercator.hpp"

#include <QDir>
#include <QImageReader>
#include <QNetworkDiskCache>
#include <QNetworkRequest>
#include <QPainter>
#include <QStandardPaths>
#include <QUrl>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

namespace
{
constexpr int tile_size = 256;
constexpr int keep_radius_tiles = 2;
constexpr int max_retry_delay_s = 60;
constexpr qint64 disk_cache_size_bytes = 100 * 1024 * 1024;
constexpr char user_agent[] =
  "autoware_minimap_overlay_rviz_plugin (+https://github.com/autowarefoundation/"
  "autoware_rviz_plugins)";
}  // namespace

TileField::TileField(QObject * parent)
: QObject(parent),
  network_manager_(new QNetworkAccessManager(this)),
  url_template_(TileProvider::getUrlTemplate(TileProvider::OpenStreetMap))
{
  // The OpenStreetMap tile usage policy asks clients to cache tiles
  auto * disk_cache = new QNetworkDiskCache(this);
  disk_cache->setCacheDirectory(
    QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation))
      .filePath("autoware_minimap_overlay_rviz_plugin"));
  disk_cache->setMaximumCacheSize(disk_cache_size_bytes);
  network_manager_->setCache(disk_cache);

  connect(network_manager_, &QNetworkAccessManager::finished, this, &TileField::onReplyFinished);
}

TileField::~TileField()
{
  abortAll();
}

void TileField::abortAll()
{
  for (auto & [key, tile] : tiles_) {
    if (tile.reply) {
      tile.reply->abort();
    }
  }
}

void TileField::setUrlTemplate(const std::string & url_template)
{
  if (url_template_ == url_template) {
    return;
  }
  url_template_ = url_template;
  abortAll();
  tiles_.clear();
  image_dirty_ = true;
  fetchMissingTiles();
}

void TileField::setCenter(double lat, double lon, int zoom)
{
  const QPointF world_pixel = autoware_minimap_overlay_rviz_plugin::lat_lon_to_world_pixel(
    autoware_minimap_overlay_rviz_plugin::clamp_latitude(lat), lon, zoom);
  const int center_x_tile = static_cast<int>(std::floor(world_pixel.x() / tile_size));
  const int center_y_tile = static_cast<int>(std::floor(world_pixel.y() / tile_size));

  const bool changed = !has_center_ || zoom != zoom_ || center_x_tile != center_x_tile_ ||
                       center_y_tile != center_y_tile_;
  zoom_ = zoom;
  center_x_tile_ = center_x_tile;
  center_y_tile_ = center_y_tile;
  has_center_ = true;

  if (changed) {
    image_dirty_ = true;
    evictFarTiles();
  }
  // Also called when the center did not change, to retry the tiles that failed
  fetchMissingTiles();
}

void TileField::evictFarTiles()
{
  for (auto it = tiles_.begin(); it != tiles_.end();) {
    const auto & [zoom, x, y] = it->first;
    const bool far = zoom != zoom_ || std::abs(x - center_x_tile_) > keep_radius_tiles ||
                     std::abs(y - center_y_tile_) > keep_radius_tiles;
    if (far) {
      if (it->second.reply) {
        it->second.reply->abort();
      }
      it = tiles_.erase(it);
    } else {
      ++it;
    }
  }
}

void TileField::fetchMissingTiles()
{
  if (!has_center_) {
    return;
  }
  const int tile_count = 1 << zoom_;
  const auto now = std::chrono::steady_clock::now();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      const int x = center_x_tile_ + dx;
      const int y = center_y_tile_ + dy;
      if (x < 0 || x >= tile_count || y < 0 || y >= tile_count) {
        continue;
      }
      const TileKey key{zoom_, x, y};
      Tile & tile = tiles_[key];
      if (tile.image.isNull() && !tile.reply && now >= tile.next_retry) {
        requestTile(key, tile);
      }
    }
  }
}

void TileField::requestTile(const TileKey & key, Tile & tile)
{
  const auto & [zoom, x, y] = key;
  const QString url = QString::fromStdString(url_template_)
                        .replace("{z}", QString::number(zoom))
                        .replace("{x}", QString::number(x))
                        .replace("{y}", QString::number(y));
  QNetworkRequest request{QUrl(url)};
  request.setHeader(QNetworkRequest::UserAgentHeader, user_agent);
  request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);

  QNetworkReply * reply = network_manager_->get(request);
  reply->setProperty("tile_zoom", zoom);
  reply->setProperty("tile_x", x);
  reply->setProperty("tile_y", y);
  tile.reply = reply;
}

void TileField::onReplyFinished(QNetworkReply * reply)
{
  reply->deleteLater();

  const TileKey key{
    reply->property("tile_zoom").toInt(), reply->property("tile_x").toInt(),
    reply->property("tile_y").toInt()};
  const auto it = tiles_.find(key);
  if (it == tiles_.end() || it->second.reply != reply) {
    // The tile was evicted or requested again
    return;
  }
  Tile & tile = it->second;
  tile.reply = nullptr;

  QImage image;
  if (reply->error() == QNetworkReply::NoError) {
    QImageReader reader(reply);
    image = reader.read();
  }

  if (image.isNull()) {
    // Retry with an exponential backoff, for example when offline or rate limited
    ++tile.failures;
    const int delay_s = std::min(max_retry_delay_s, 1 << std::min(tile.failures, 6));
    tile.next_retry = std::chrono::steady_clock::now() + std::chrono::seconds(delay_s);
    return;
  }

  tile.image = image;
  tile.failures = 0;
  image_dirty_ = true;
  Q_EMIT tilesUpdated();
}

QImage TileField::getTileFieldImage()
{
  if (!image_dirty_) {
    return tile_field_image_;
  }
  if (tile_field_image_.isNull()) {
    tile_field_image_ = QImage(tile_size * 3, tile_size * 3, QImage::Format_ARGB32_Premultiplied);
  }
  tile_field_image_.fill(Qt::transparent);

  QPainter painter(&tile_field_image_);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      const auto it = tiles_.find(TileKey{zoom_, center_x_tile_ + dx, center_y_tile_ + dy});
      if (it != tiles_.end() && !it->second.image.isNull()) {
        painter.drawImage(
          QRectF((dx + 1) * tile_size, (dy + 1) * tile_size, tile_size, tile_size),
          it->second.image);
      }
    }
  }
  painter.end();

  image_dirty_ = false;
  return tile_field_image_;
}

QPointF TileField::getTileFieldOrigin() const
{
  return {
    static_cast<double>((center_x_tile_ - 1) * tile_size),
    static_cast<double>((center_y_tile_ - 1) * tile_size)};
}
