// Copyright 2026 Tier IV, Inc.
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
#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__MAP__LANELET2_VECTOR_MAP_DISPLAY_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__MAP__LANELET2_VECTOR_MAP_DISPLAY_HPP_

#include "autoware_perception_rviz_plugin/visibility_control.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace autoware
{
namespace rviz_plugins
{
namespace map
{

class AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC Lanelet2VectorMapDisplay
: public rviz_common::RosTopicDisplay<visualization_msgs::msg::MarkerArray>
{
  Q_OBJECT

public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using RosTopicDisplay = rviz_common::RosTopicDisplay<MarkerArray>;

  Lanelet2VectorMapDisplay();
  ~Lanelet2VectorMapDisplay() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void fixedFrameChanged() override;
  void unsubscribe() override;

private Q_SLOTS:
  void onStyleChanged();

private:
  void processMessage(MarkerArray::ConstSharedPtr msg) override;
  void applyStylesAndPublish();

  MarkerCommon m_marker_common;
  MarkerArray::ConstSharedPtr m_last_msg;

  rviz_common::properties::FloatProperty m_z_offset_property;

  rviz_common::properties::BoolProperty m_road_border_override_property;
  rviz_common::properties::ColorProperty m_road_border_color_property;
  rviz_common::properties::FloatProperty m_road_border_alpha_property;
  rviz_common::properties::FloatProperty m_road_border_width_property;

  rviz_common::properties::BoolProperty m_lane_bound_override_property;
  rviz_common::properties::ColorProperty m_lane_bound_color_property;
  rviz_common::properties::FloatProperty m_lane_bound_alpha_property;
  rviz_common::properties::FloatProperty m_lane_bound_width_property;

  rviz_common::properties::BoolProperty m_center_line_override_property;
  rviz_common::properties::ColorProperty m_center_line_color_property;
  rviz_common::properties::FloatProperty m_center_line_alpha_property;
  rviz_common::properties::FloatProperty m_center_line_width_property;

  rviz_common::properties::BoolProperty m_stopline_override_property;
  rviz_common::properties::ColorProperty m_stopline_color_property;
  rviz_common::properties::FloatProperty m_stopline_alpha_property;
  rviz_common::properties::FloatProperty m_stopline_width_property;

  rviz_common::properties::BoolProperty m_road_fill_override_property;
  rviz_common::properties::ColorProperty m_road_fill_color_property;
  rviz_common::properties::FloatProperty m_road_fill_alpha_property;

  rviz_common::properties::BoolProperty m_traffic_light_override_property;
  rviz_common::properties::ColorProperty m_traffic_light_plate_color_property;
  rviz_common::properties::FloatProperty m_traffic_light_plate_alpha_property;
  rviz_common::properties::ColorProperty m_traffic_light_border_color_property;
  rviz_common::properties::FloatProperty m_traffic_light_border_alpha_property;
  rviz_common::properties::FloatProperty m_traffic_light_border_width_property;
  rviz_common::properties::FloatProperty m_traffic_light_bulb_alpha_property;
};

}  // namespace map
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__MAP__LANELET2_VECTOR_MAP_DISPLAY_HPP_
