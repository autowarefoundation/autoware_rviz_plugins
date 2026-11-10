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

#include "map_style.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace autoware_minimap_overlay_rviz_plugin
{

namespace
{
using Rgb = std::array<double, 3>;

struct Palette
{
  Rgb land;
  Rgb building;
  Rgb road;
  Rgb major_road;
  Rgb water;
  Rgb text;
};

// The dark style has the charcoal of the Autoware RViz theme and of the other Autoware overlays.
// The light style has the mist and the navy of the Autoware brand colors.
constexpr Palette dark_palette{{23.0, 28.0, 31.0},    {30.0, 36.0, 40.0}, {92.0, 104.0, 112.0},
                               {134.0, 148.0, 158.0}, {16.0, 34.0, 44.0}, {214.0, 220.0, 224.0}};
constexpr Palette light_palette{{228.0, 232.0, 246.0}, {214.0, 219.0, 238.0}, {255.0, 255.0, 255.0},
                                {255.0, 222.0, 140.0}, {160.0, 200.0, 240.0}, {0.0, 0.0, 42.0}};
// Land color of the standard OpenStreetMap style
constexpr Rgb standard_land{242.0, 239.0, 233.0};

// 0 at `from`, 1 at `to`, and a smooth change between them. `from` can be more than `to`.
double smooth_step(const double from, const double to, const double value)
{
  const double t = std::clamp((value - from) / (to - from), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

void blend(Rgb & color, const Rgb & target, const double weight)
{
  for (std::size_t i = 0; i < color.size(); ++i) {
    color[i] += (target[i] - color[i]) * weight;
  }
}

QRgb navigation_color(const QRgb pixel, const Palette & palette)
{
  const double r = qRed(pixel) / 255.0;
  const double g = qGreen(pixel) / 255.0;
  const double b = qBlue(pixel) / 255.0;
  const double max = std::max({r, g, b});
  const double min = std::min({r, g, b});
  const double lightness = (max + min) / 2.0;
  const double chroma = max - min;
  const double saturation =
    chroma > 1e-6 ? std::min(chroma / (1.0 - std::abs(2.0 * lightness - 1.0) + 1e-6), 1.0) : 0.0;
  double hue_deg = 0.0;
  if (chroma > 1e-6) {
    if (max == r) {
      hue_deg = 60.0 * std::fmod((g - b) / chroma + 6.0, 6.0);
    } else if (max == g) {
      hue_deg = 60.0 * ((b - r) / chroma + 2.0);
    } else {
      hue_deg = 60.0 * ((r - g) / chroma + 4.0);
    }
  }

  // The limits below are from the colors of the standard OpenStreetMap style.
  // Each weight changes smoothly, so that the smooth edges of the tile stay smooth.
  Rgb color = palette.land;
  // Buildings are a little darker than the land, with nearly no color
  blend(
    color, palette.building,
    smooth_step(0.90, 0.84, lightness) * smooth_step(0.55, 0.70, lightness) *
      (1.0 - smooth_step(0.25, 0.45, saturation)));
  // Water is light blue
  if (hue_deg > 180.0 && hue_deg < 215.0) {
    blend(
      color, palette.water, smooth_step(0.15, 0.3, saturation) * smooth_step(0.5, 0.65, lightness));
  }
  // Small roads are white
  blend(color, palette.road, smooth_step(0.955, 0.99, lightness) * smooth_step(0.07, 0.03, chroma));
  // Large roads are red, orange or yellow
  if (hue_deg < 68.0 || hue_deg > 330.0) {
    blend(
      color, palette.major_road,
      smooth_step(0.35, 0.6, saturation) * smooth_step(0.62, 0.72, lightness) *
        smooth_step(0.90, 0.88, lightness));
  }
  // The text and the thin lines are dark
  blend(color, palette.text, smooth_step(0.5, 0.2, lightness));

  return qRgb(std::lround(color[0]), std::lround(color[1]), std::lround(color[2]));
}
}  // namespace

QColor map_background(const MapStyle style)
{
  const Rgb & land = style == MapStyle::NavigationDark    ? dark_palette.land
                     : style == MapStyle::NavigationLight ? light_palette.land
                                                          : standard_land;
  return QColor(std::lround(land[0]), std::lround(land[1]), std::lround(land[2]));
}

QImage apply_map_style(const QImage & tile, const MapStyle style)
{
  if (style == MapStyle::Standard) {
    return tile;
  }
  const Palette & palette = style == MapStyle::NavigationDark ? dark_palette : light_palette;

  QImage image = tile.convertToFormat(QImage::Format_RGB32);
  // A tile has few different colors, so remember the color of the last pixel
  QRgb last_pixel = 0;
  QRgb last_color = navigation_color(last_pixel, palette);
  for (int y = 0; y < image.height(); ++y) {
    auto * line = reinterpret_cast<QRgb *>(image.scanLine(y));
    for (int x = 0; x < image.width(); ++x) {
      if (line[x] != last_pixel) {
        last_pixel = line[x];
        last_color = navigation_color(last_pixel, palette);
      }
      line[x] = last_color;
    }
  }
  return image;
}

}  // namespace autoware_minimap_overlay_rviz_plugin
