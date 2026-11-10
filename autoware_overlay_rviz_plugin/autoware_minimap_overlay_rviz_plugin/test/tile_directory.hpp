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

#ifndef TILE_DIRECTORY_HPP_
#define TILE_DIRECTORY_HPP_

#include <QColor>
#include <QDir>
#include <QEventLoop>
#include <QImage>
#include <QTemporaryDir>
#include <QTimer>
#include <QUrl>

#include <functional>

// Temporary directory with tile image files, for a TileLoader with a file URL
class TileDirectory
{
public:
  void addTile(const int zoom, const int x, const int y, const QColor & color) const
  {
    QImage image(256, 256, QImage::Format_RGB32);
    image.fill(color);
    QDir(directory_.path()).mkpath(QString("%1/%2").arg(zoom).arg(x));
    image.save(tilePath(zoom, x, y));
  }

  QString tilePath(const int zoom, const int x, const int y) const
  {
    return QString("%1/%2/%3/%4.png").arg(directory_.path()).arg(zoom).arg(x).arg(y);
  }

  QString urlTemplate() const
  {
    return QUrl::fromLocalFile(directory_.path()).toString() + "/{z}/{x}/{y}.png";
  }

private:
  QTemporaryDir directory_;
};

// Runs the Qt event loop until the condition is true. Returns false after the timeout.
inline bool wait_for(const std::function<bool()> & condition, const int timeout_ms = 5000)
{
  QEventLoop loop;
  QTimer poll;
  QTimer timeout;
  timeout.setSingleShot(true);
  QObject::connect(&poll, &QTimer::timeout, &loop, [&]() {
    if (condition()) {
      loop.quit();
    }
  });
  QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);
  poll.start(5);
  timeout.start(timeout_ms);
  loop.exec();
  return condition();
}

#endif  // TILE_DIRECTORY_HPP_
