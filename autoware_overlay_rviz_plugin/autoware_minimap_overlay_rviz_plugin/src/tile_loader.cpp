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

#include "tile_loader.hpp"

#include "web_mercator.hpp"

#include <QNetworkDiskCache>
#include <QNetworkRequest>
#include <QUrl>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace autoware_minimap_overlay_rviz_plugin
{

namespace
{
constexpr int keep_margin_tiles = 1;
constexpr int max_retry_delay_s = 60;
constexpr qint64 disk_cache_size_bytes = 100 * 1024 * 1024;
// The OpenStreetMap tile usage policy asks for a User-Agent that identifies the application
constexpr char user_agent[] =
  "autoware_minimap_overlay_rviz_plugin (+https://github.com/autowarefoundation/"
  "autoware_rviz_plugins)";

int wrap(const int value, const int count)
{
  return ((value % count) + count) % count;
}
}  // namespace

TileLoader::TileLoader(
  const QString & url_template, const QString & cache_directory, QObject * parent)
: QObject(parent), url_template_(url_template)
{
  if (!cache_directory.isEmpty()) {
    // The OpenStreetMap tile usage policy asks clients to cache the tiles
    auto * disk_cache = new QNetworkDiskCache(&network_);
    disk_cache->setCacheDirectory(cache_directory);
    disk_cache->setMaximumCacheSize(disk_cache_size_bytes);
    network_.setCache(disk_cache);
  }

  retry_timer_.setSingleShot(true);
  connect(&retry_timer_, &QTimer::timeout, this, &TileLoader::requestMissingTiles);
}

TileLoader::~TileLoader()
{
  dropAllTiles();
}

void TileLoader::dropAllTiles()
{
  for (auto & [id, tile] : tiles_) {
    if (tile.reply) {
      // abort() emits finished(). Disconnect first so that it does not reach this object.
      tile.reply->disconnect(this);
      tile.reply->abort();
      tile.reply->deleteLater();
    }
  }
  tiles_.clear();
  mosaic_dirty_ = true;
}

void TileLoader::setStyle(const MapStyle style)
{
  if (style_ == style) {
    return;
  }
  style_ = style;
  // The disk cache gives the tiles again
  dropAllTiles();
  requestMissingTiles();
}

void TileLoader::setView(const QPointF & center, const int zoom, const QSizeF & size_px)
{
  has_view_ = true;
  center_px_ = center * map_size_px(zoom);
  zoom_ = zoom;
  size_px_ = size_px;

  dropFarTiles();
  requestMissingTiles();
}

TileLoader::TileRange TileLoader::visibleRange() const
{
  const auto tile_index = [](const double position_px) {
    return static_cast<int>(std::floor(position_px / tile_size_px));
  };
  return {
    tile_index(center_px_.x() - size_px_.width() / 2.0),
    tile_index(center_px_.x() + size_px_.width() / 2.0),
    tile_index(center_px_.y() - size_px_.height() / 2.0),
    tile_index(center_px_.y() + size_px_.height() / 2.0)};
}

void TileLoader::dropFarTiles()
{
  const TileRange range = visibleRange();
  const int tile_count = 1 << zoom_;

  for (auto it = tiles_.begin(); it != tiles_.end();) {
    const TileId & id = it->first;
    bool keep = id.zoom == zoom_ && id.y >= range.y_min - keep_margin_tiles &&
                id.y <= range.y_max + keep_margin_tiles;
    if (keep) {
      // The view can cross the east or west edge of the map, so compare the wrapped columns
      keep = false;
      for (int x = range.x_min - keep_margin_tiles; x <= range.x_max + keep_margin_tiles; ++x) {
        keep = keep || wrap(x, tile_count) == id.x;
      }
    }
    if (keep) {
      ++it;
      continue;
    }
    const QPointer<QNetworkReply> reply = it->second.reply;
    it = tiles_.erase(it);
    mosaic_dirty_ = true;
    if (reply) {
      // The tile is gone from the map, so onReplyFinished() ignores this reply
      reply->abort();
    }
  }
}

void TileLoader::requestMissingTiles()
{
  if (!has_view_) {
    return;
  }
  const TileRange range = visibleRange();
  const int tile_count = 1 << zoom_;
  const auto now = std::chrono::steady_clock::now();
  auto next_retry = std::chrono::steady_clock::time_point::max();

  for (int x = range.x_min; x <= range.x_max; ++x) {
    for (int y = std::max(range.y_min, 0); y <= std::min(range.y_max, tile_count - 1); ++y) {
      const TileId id{zoom_, wrap(x, tile_count), y};
      Tile & tile = tiles_[id];
      if (!tile.image.isNull() || tile.reply) {
        continue;
      }
      if (now < tile.retry_time) {
        next_retry = std::min(next_retry, tile.retry_time);
        continue;
      }
      requestTile(id, tile);
    }
  }

  if (next_retry != std::chrono::steady_clock::time_point::max()) {
    const auto delay = std::chrono::ceil<std::chrono::milliseconds>(next_retry - now);
    retry_timer_.start(delay);
  }
}

void TileLoader::requestTile(const TileId & id, Tile & tile)
{
  const QString url = QString(url_template_)
                        .replace("{z}", QString::number(id.zoom))
                        .replace("{x}", QString::number(id.x))
                        .replace("{y}", QString::number(id.y));
  QNetworkRequest request{QUrl(url)};
  request.setHeader(QNetworkRequest::UserAgentHeader, user_agent);
  request.setAttribute(QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::PreferCache);
  request.setAttribute(
    QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::NoLessSafeRedirectPolicy);

  QNetworkReply * reply = network_.get(request);
  tile.reply = reply;
  connect(
    reply, &QNetworkReply::finished, this, [this, id, reply]() { onReplyFinished(id, reply); });
}

void TileLoader::onReplyFinished(const TileId & id, QNetworkReply * reply)
{
  reply->deleteLater();

  const auto it = tiles_.find(id);
  if (it == tiles_.end() || it->second.reply != reply) {
    return;
  }
  Tile & tile = it->second;
  tile.reply = nullptr;

  QString failure;
  if (reply->error() != QNetworkReply::NoError) {
    failure = reply->errorString();
  } else if (!tile.image.loadFromData(reply->readAll())) {
    failure = QString("%1 is not an image").arg(reply->url().toString());
  }

  if (failure.isEmpty()) {
    tile.image = apply_map_style(tile.image, style_);
    tile.failures = 0;
    mosaic_dirty_ = true;
    Q_EMIT tileLoaded();
    return;
  }

  // Wait longer after each failure, for example when the host is offline or rate limited
  tile.failures = std::min(tile.failures + 1, 6);
  const int delay_s = std::min(1 << tile.failures, max_retry_delay_s);
  tile.retry_time = std::chrono::steady_clock::now() + std::chrono::seconds(delay_s);
  if (!retry_timer_.isActive()) {
    retry_timer_.start(std::chrono::seconds(delay_s));
  }
  Q_EMIT tileFailed(failure);
}

void TileLoader::paint(QPainter & painter, const QRectF & target, const bool snap_to_pixels)
{
  if (!has_view_) {
    return;
  }
  const TileRange range = visibleRange();
  const QSize mosaic_size(
    (range.x_max - range.x_min + 1) * tile_size_px, (range.y_max - range.y_min + 1) * tile_size_px);

  if (mosaic_dirty_ || !(range == mosaic_range_) || mosaic_.size() != mosaic_size) {
    if (mosaic_.size() != mosaic_size) {
      mosaic_ = QImage(mosaic_size, QImage::Format_ARGB32_Premultiplied);
    }
    mosaic_.fill(Qt::transparent);
    QPainter mosaic_painter(&mosaic_);
    const int tile_count = 1 << zoom_;
    for (int x = range.x_min; x <= range.x_max; ++x) {
      for (int y = range.y_min; y <= range.y_max; ++y) {
        const auto it = tiles_.find(TileId{zoom_, wrap(x, tile_count), y});
        if (it != tiles_.end() && !it->second.image.isNull()) {
          mosaic_painter.drawImage(
            QRect(
              (x - range.x_min) * tile_size_px, (y - range.y_min) * tile_size_px, tile_size_px,
              tile_size_px),
            it->second.image);
        }
      }
    }
    mosaic_range_ = range;
    mosaic_dirty_ = false;
  }

  QPointF offset = target.center() - center_px_;
  if (snap_to_pixels) {
    offset = offset.toPoint();
  }
  painter.drawImage(
    offset + QPointF(range.x_min * tile_size_px, range.y_min * tile_size_px), mosaic_);
}

}  // namespace autoware_minimap_overlay_rviz_plugin
