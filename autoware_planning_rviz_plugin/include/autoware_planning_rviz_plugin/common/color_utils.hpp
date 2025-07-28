// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__COMMON__COLOR_UTILS_HPP_
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__COMMON__COLOR_UTILS_HPP_

#include <QColor>

#include <OgreColourValue.h>

#include <memory>

namespace rviz_plugins
{
namespace color_utils
{
std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double velocity);
std::unique_ptr<Ogre::ColourValue> gradation(
  const QColor & color_min, const QColor & color_max, const double ratio);
}  // namespace color_utils
}  // namespace rviz_plugins

#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__COMMON__COLOR_UTILS_HPP_
