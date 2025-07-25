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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__DISPLAY_BASE_HPP_
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__DISPLAY_BASE_HPP_

#include "autoware_planning_rviz_plugin/common/color_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>

#include <OgreManualObject.h>
#include <OgreSceneNode.h>

#include <memory>
#include <vector>

namespace rviz_plugins
{

template <typename MessageType>
class CandidateTrajectoriesDisplayBase : public rviz_common::Display
{
public:
  CandidateTrajectoriesDisplayBase();
  virtual ~CandidateTrajectoriesDisplayBase();

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;

protected:
  // Pure virtual methods for message type specialization
  /// Process received ROS message and update visualization
  virtual void processMessage(const typename MessageType::ConstSharedPtr msg_ptr) = 0;
  /// Setup coloring mode options specific to the message type
  virtual void setupColoringModes() = 0;

  // Common functionality
  virtual bool validateFloats(const typename MessageType::ConstSharedPtr & msg_ptr);
  virtual std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(const double velocity);
  virtual std::unique_ptr<Ogre::ColourValue> gradation(
    const QColor & color_min, const QColor & color_max, const double ratio);

  // Common initialization methods
  void initializePropertyConstraints();

  // Common visualization methods
  void resizeManualObjects(size_t num_trajectories);
  void clearManualObjects();

  // Common properties - topic first for UI display order
  rviz_common::properties::RosTopicProperty property_topic_;

  // Path visualization properties
  rviz_common::properties::BoolProperty property_path_view_;
  rviz_common::properties::BoolProperty property_path_width_view_;
  rviz_common::properties::FloatProperty property_path_width_;
  rviz_common::properties::FloatProperty property_path_alpha_;
  rviz_common::properties::EnumProperty property_coloring_mode_;
  rviz_common::properties::FloatProperty property_fade_out_distance_;

  // Velocity-based coloring properties (common to all)
  rviz_common::properties::ColorProperty property_velocity_color_min_;
  rviz_common::properties::ColorProperty property_velocity_color_mid_;
  rviz_common::properties::ColorProperty property_velocity_color_max_;
  rviz_common::properties::FloatProperty property_vel_max_;

  // Velocity visualization properties
  rviz_common::properties::BoolProperty property_velocity_view_;
  rviz_common::properties::FloatProperty property_velocity_alpha_;
  rviz_common::properties::FloatProperty property_velocity_scale_;
  rviz_common::properties::BoolProperty property_velocity_color_view_;
  rviz_common::properties::ColorProperty property_velocity_color_;

  // Generator visualization properties
  rviz_common::properties::BoolProperty property_generator_text_view_;
  rviz_common::properties::FloatProperty property_generator_text_scale_;

  // Common data members
  std::vector<Ogre::ManualObject *> path_manual_objects_;
  std::vector<Ogre::ManualObject *> velocity_manual_objects_;
  std::vector<rviz_rendering::MovableText *> generator_texts_;
  std::vector<Ogre::SceneNode *> generator_text_nodes_;

  // Common coloring modes
  enum CommonColoringMode { VELOCITY_BASED = 0 };

  typename MessageType::ConstSharedPtr last_msg_ptr_;

  // Common methods available to derived classes
  void subscribe();
  void unsubscribe();

private:
  typename rclcpp::Subscription<MessageType>::SharedPtr subscription_;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__CANDIDATE_TRAJECTORIES__DISPLAY_BASE_HPP_
