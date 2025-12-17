// Copyright 2024 TIER IV, Inc.
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

#ifndef STRING_STAMPED_OVERLAY_DISPLAY_HPP_
#define STRING_STAMPED_OVERLAY_DISPLAY_HPP_

#include <mutex>
#include <string>

#ifndef Q_MOC_RUN
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#endif

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>

namespace autoware::string_stamped_rviz_plugin
{

class StringStampedOverlayDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  StringStampedOverlayDisplay();
  ~StringStampedOverlayDisplay() override;
  StringStampedOverlayDisplay(const StringStampedOverlayDisplay &) = delete;
  StringStampedOverlayDisplay(StringStampedOverlayDisplay &&) = delete;
  StringStampedOverlayDisplay & operator=(const StringStampedOverlayDisplay &) = delete;
  StringStampedOverlayDisplay & operator=(StringStampedOverlayDisplay &&) = delete;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void update_topic();
  void update_overlay_position();

private:  // NOLINT
  void process_message(const autoware_internal_debug_msgs::msg::StringStamped::ConstSharedPtr msg);
  void draw_text(QImage & image);

  // Overlay object
  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;

  // Subscription
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr subscription_;

  // Properties
  rviz_common::properties::RosTopicProperty * topic_property_;
  rviz_common::properties::IntProperty * padding_property_;
  rviz_common::properties::IntProperty * left_property_;
  rviz_common::properties::IntProperty * top_property_;
  rviz_common::properties::EnumProperty * hor_alignment_property_;
  rviz_common::properties::EnumProperty * ver_alignment_property_;
  rviz_common::properties::IntProperty * font_size_property_;
  rviz_common::properties::ColorProperty * fg_color_property_;
  rviz_common::properties::FloatProperty * fg_alpha_property_;
  rviz_common::properties::ColorProperty * bg_color_property_;
  rviz_common::properties::FloatProperty * bg_alpha_property_;
  rviz_common::properties::FloatProperty * fade_delay_property_;
  rviz_common::properties::FloatProperty * fade_time_property_;

  // State
  std::mutex mutex_;
  std::string text_;
  autoware_internal_debug_msgs::msg::StringStamped::ConstSharedPtr last_msg_;
  autoware_internal_debug_msgs::msg::StringStamped::ConstSharedPtr last_non_empty_msg_;
  int current_alpha_{255};
};

}  // namespace autoware::string_stamped_rviz_plugin

#endif  // STRING_STAMPED_OVERLAY_DISPLAY_HPP_
