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

#include "autoware_planning_rviz_plugin/common/color_utils.hpp"

#include <algorithm>
#include <memory>

namespace rviz_plugins
{
namespace color_utils
{

std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double velocity)
{
  // This is a basic implementation - can be customized as needed
  const double vel_max = 10.0;  // Default max velocity
  const double ratio = std::min(std::max(velocity / vel_max, 0.0), 1.0);

  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);

  if (ratio < 0.5) {
    // Blue to Green transition
    color_ptr->r = 0.0f;
    color_ptr->g = static_cast<float>(ratio * 2.0);
    color_ptr->b = static_cast<float>(1.0 - ratio * 2.0);
  } else {
    // Green to Red transition
    color_ptr->r = static_cast<float>((ratio - 0.5) * 2.0);
    color_ptr->g = static_cast<float>(1.0 - (ratio - 0.5) * 2.0);
    color_ptr->b = 0.0f;
  }

  color_ptr->a = 1.0f;
  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  // Clamp ratio to valid range [0.0, 1.0] to ensure valid color values
  const double clamped_ratio = std::min(std::max(ratio, 0.0), 1.0);

  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g = static_cast<float>(
    color_max.greenF() * clamped_ratio + color_min.greenF() * (1.0 - clamped_ratio));
  color_ptr->r =
    static_cast<float>(color_max.redF() * clamped_ratio + color_min.redF() * (1.0 - clamped_ratio));
  color_ptr->b = static_cast<float>(
    color_max.blueF() * clamped_ratio + color_min.blueF() * (1.0 - clamped_ratio));
  color_ptr->a = 1.0f;
  return color_ptr;
}

}  // namespace color_utils
}  // namespace rviz_plugins
