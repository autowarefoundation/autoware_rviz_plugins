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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__CANDIDATE_TRAJECTORIES_DISPLAY_HPP_  // NOLINT
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__CANDIDATE_TRAJECTORIES_DISPLAY_HPP_  // NOLINT

#include "autoware_planning_rviz_plugin/candidate_trajectories/display_base.hpp"

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>

#include <memory>
#include <vector>

namespace rviz_plugins
{
class AutowareCandidateTrajectoriesDisplay
: public CandidateTrajectoriesDisplayBase<
    autoware_internal_planning_msgs::msg::CandidateTrajectories>
{
  Q_OBJECT

public:
  AutowareCandidateTrajectoriesDisplay();
  ~AutowareCandidateTrajectoriesDisplay() override;

protected Q_SLOTS:
  void updateVisualization();
  void updateColoringModeVisibility();
  void onTopicChanged();

protected:
  // Override base class virtual methods
  void setupColoringModes() override;
  void processMessage(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg_ptr)
    override;
  bool validateFloats(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr & msg_ptr)
    override;

  /// Coloring modes available for candidate trajectories
  enum CandidateColoringMode {
    CANDIDATE_VELOCITY_BASED = 0,  ///< Color trajectories based on velocity
    INDEX_BASED = 1                ///< Color trajectories based on their index (cyclical colors)
  };

  // Index-based coloring properties
  rviz_common::properties::ColorProperty property_index_color_1_{
    "Index Color 1", QColor("#FF8080"), "", &property_coloring_mode_};
  rviz_common::properties::ColorProperty property_index_color_2_{
    "Index Color 2", QColor("#80FF80"), "", &property_coloring_mode_};
  rviz_common::properties::ColorProperty property_index_color_3_{
    "Index Color 3", QColor("#8080FF"), "", &property_coloring_mode_};
  rviz_common::properties::ColorProperty property_index_color_4_{
    "Index Color 4", QColor("#FFFF80"), "", &property_coloring_mode_};

  // Helper method for non-scored trajectory coloring
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnIndex(const size_t index);

private:
  std::vector<QColor> index_colors_;
  void updateIndexColors();
};
}  // namespace rviz_plugins

// NOLINTNEXTLINE
#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__CANDIDATE_TRAJECTORIES_DISPLAY_HPP_
