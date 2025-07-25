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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__SCORED_CANDIDATE_TRAJECTORIES_DISPLAY_HPP_  // NOLINT
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__SCORED_CANDIDATE_TRAJECTORIES_DISPLAY_HPP_  // NOLINT

#include "autoware_planning_rviz_plugin/candidate_trajectories/display_base.hpp"

#include <rviz_common/properties/color_property.hpp>

#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>

#include <memory>
#include <vector>

namespace rviz_plugins
{
class AutowareScoredCandidateTrajectoriesDisplay
: public CandidateTrajectoriesDisplayBase<
    autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>
{
  Q_OBJECT

public:
  AutowareScoredCandidateTrajectoriesDisplay();
  ~AutowareScoredCandidateTrajectoriesDisplay() override;

protected Q_SLOTS:
  void updateVisualization();
  void updateColoringModeVisibility();
  void onTopicChanged();

protected:
  // Override base class virtual methods
  void setupColoringModes() override;
  void processMessage(
    const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr msg_ptr)
    override;
  bool validateFloats(
    const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::ConstSharedPtr &
      msg_ptr) override;

  // Score-specific coloring methods
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnScore(const double score);
  std::unique_ptr<Ogre::ColourValue> setColorDependsOnHighestScore(const bool is_highest_score);

  /// Coloring modes available for scored trajectories
  enum ScoredColoringMode {
    SCORED_VELOCITY_BASED = 0,  ///< Color trajectories based on velocity
    SCORE_BASED = 1,            ///< Color trajectories based on continuous score values
    HIGHEST_SCORE_BASED = 2     ///< Color trajectories binary: highest score vs others
  };

  // Score-specific data members
  std::vector<std::vector<rviz_rendering::MovableText *>> score_texts_;
  std::vector<std::vector<Ogre::SceneNode *>> score_text_nodes_;

  // Score-based coloring properties - parented to coloring_mode
  rviz_common::properties::ColorProperty property_score_color_min_{
    "Min Score Color", QColor("#FF0000"), "", &property_coloring_mode_};
  rviz_common::properties::ColorProperty property_score_color_mid_{
    "Mid Score Color", QColor("#FFFF00"), "", &property_coloring_mode_};
  rviz_common::properties::ColorProperty property_score_color_max_{
    "Max Score Color", QColor("#00FF00"), "", &property_coloring_mode_};

  // Highest-score based coloring properties
  rviz_common::properties::ColorProperty property_highest_score_color_{
    "Highest Score Color", QColor("#00FF00"), "", &property_coloring_mode_};
  rviz_common::properties::ColorProperty property_other_score_color_{
    "Other Score Color", QColor("#808080"), "", &property_coloring_mode_};

  // Score text properties
  rviz_common::properties::BoolProperty property_score_text_view_{
    "View Score Text", false, "", this};
  rviz_common::properties::FloatProperty property_score_text_scale_{
    "Scale", 0.3, "", &property_score_text_view_};
};
}  // namespace rviz_plugins

// NOLINTNEXTLINE
#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__SCORED_CANDIDATE_TRAJECTORIES_DISPLAY_HPP_
