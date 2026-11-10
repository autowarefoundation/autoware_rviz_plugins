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

#ifndef MAP_STYLE_HPP_
#define MAP_STYLE_HPP_

#include <QColor>
#include <QImage>

namespace autoware_minimap_overlay_rviz_plugin
{

// NavigationDark and NavigationLight show mainly the roads, in the Autoware colors.
// Standard shows the tiles of OpenStreetMap as they are.
enum class MapStyle { NavigationDark, NavigationLight, Standard };

// Color of the land of the map in a style
QColor map_background(MapStyle style);

// Changes the colors of a tile of the standard OpenStreetMap style to the map style.
// The tile server has one style only. The navigation styles find the roads, the water, the
// buildings and the text of a tile by their colors, and give each of them a new color.
QImage apply_map_style(const QImage & tile, MapStyle style);

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // MAP_STYLE_HPP_
