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

#ifndef AUTOWARE_PLANNING_RVIZ_PLUGIN__MISSION_ROUTE_GOAL__MISSION_ROUTE_GOAL_DISPLAY_HPP_
#define AUTOWARE_PLANNING_RVIZ_PLUGIN__MISSION_ROUTE_GOAL__MISSION_ROUTE_GOAL_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#endif

namespace rviz_plugins
{

class MissionRouteGoalDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  MissionRouteGoalDisplay();
  ~MissionRouteGoalDisplay() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

private Q_SLOTS:
  void onRouteTopicChanged();
  void onMapTopicChanged();
  void onStyleChanged();

private:  // NOLINT
  struct RouteSamplePt
  {
    double x;
    double y;
    double z;
    double s;
    double tx;
    double ty;
  };

  void setupRouteSubscription();
  void setupMapSubscription();
  void onRoute(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void onMapBin(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void rebuildRouteCenterline();
  void publishVisualMarkers(
    bool has_ego, double ego_x, double ego_y, bool has_cam, double cam_x, double cam_y,
    double cam_z);

  std::mutex m_mutex;
  rviz_default_plugins::displays::MarkerCommon m_marker_common;

  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr m_route_sub;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr m_map_sub;

  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr m_route_msg;
  lanelet::LaneletMapPtr m_lanelet_map;
  std::vector<RouteSamplePt> m_route_pts;

  bool m_centerline_dirty{false};
  bool m_visual_dirty{false};
  bool m_had_ego_last{false};
  double m_last_ego_x{0.0};
  double m_last_ego_y{0.0};
  double m_last_cam_dist{-1.0};

  std::unique_ptr<rviz_common::properties::RosTopicProperty> m_route_topic_property;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> m_map_topic_property;

  rviz_common::properties::BoolProperty m_show_route_property;
  rviz_common::properties::ColorProperty m_route_color_property;
  rviz_common::properties::FloatProperty m_route_alpha_property;
  rviz_common::properties::FloatProperty m_route_width_property;
  rviz_common::properties::BoolProperty m_trim_passed_property;
  rviz_common::properties::FloatProperty m_z_offset_property;

  rviz_common::properties::BoolProperty m_show_goal_property;
  rviz_common::properties::ColorProperty m_goal_color_property;
  rviz_common::properties::FloatProperty m_goal_alpha_property;
  rviz_common::properties::FloatProperty m_goal_radius_property;
  rviz_common::properties::FloatProperty m_beacon_height_property;
  rviz_common::properties::BoolProperty m_show_distance_text_property;
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_PLANNING_RVIZ_PLUGIN__MISSION_ROUTE_GOAL__MISSION_ROUTE_GOAL_DISPLAY_HPP_
