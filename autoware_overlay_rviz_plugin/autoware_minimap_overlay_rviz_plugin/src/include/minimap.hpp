// Copyright 2024 The Autoware Contributors
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
#ifndef MINIMAP_HPP_
#define MINIMAP_HPP_

#include "goal_pose.hpp"
#include "overlay_utils.hpp"
#include "path_overlay.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "tile_field.hpp"
#include "tile_provider.hpp"  // Include the TileProvider header

#include <QImage>
#include <QPixmap>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_rendering/render_window.hpp>

#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace autoware_minimap_overlay_rviz_plugin
{

class VehicleMapDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  VehicleMapDisplay();
  virtual ~VehicleMapDisplay();

  void onInitialize() override;
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateOverlaySize();
  void updateOverlayPosition();
  void onTilesUpdated();
  void updateZoomLevel();
  void updateMapPosition();
  void updateTileProvider();

protected:
  void onEnable() override;
  void onDisable() override;

private:
  void drawWidget(QImage & hud);
  void drawCircle(QPainter & painter, const QRectF & backgroundRect);
  void drawAttribution(QPainter & painter, const QRectF & backgroundRect);
  void requestRedraw();
  void routeCallback(const autoware_adapi_v1_msgs::msg::Route::SharedPtr msg);
  void poseCallback(const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg);
  void routeStateCallback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void routePointsCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg);
  void mapProjectorInfoCallback(const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg);
  void updateGoalPose();
  void updatePath();

  std::optional<std::pair<double, double>> localToLatLon(const geometry_msgs::msg::Point & point);

  autoware_minimap_overlay_rviz_plugin::OverlayObject::SharedPtr overlay_;

  std::mutex mutex_;
  std::mutex property_mutex_;

  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;

  rviz_common::properties::IntProperty * property_horizontal_margin_;
  rviz_common::properties::IntProperty * property_vertical_margin_;

  rviz_common::properties::EnumProperty * property_tile_provider_;
  rviz_common::properties::EnumProperty * property_anchor_vertical_;
  rviz_common::properties::EnumProperty * property_anchor_horizontal_;
  rviz_common::properties::IntProperty * property_border_radius_;

  rviz_common::properties::IntProperty * property_zoom_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * property_latitude_;
  rviz_common::properties::FloatProperty * property_longitude_;
  rviz_common::properties::FloatProperty * property_goal_lat;
  rviz_common::properties::FloatProperty * property_goal_lon;

  int zoom_;

  std::unique_ptr<TileField> tile_field_;
  rclcpp::Node::SharedPtr node_;

  double latitude_ = 0.0;
  double longitude_ = 0.0;
  bool has_vehicle_position_ = false;
  bool needs_redraw_ = true;
  QPixmap pos_pixmap_;

  // subscription ptr and msg ptr
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::Route>::SharedPtr route_sub_;
  autoware_adapi_v1_msgs::msg::Route::SharedPtr route_msg_;

  // subscription ptr and msg ptr
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>::SharedPtr pose_sub_;
  autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr pose_msg_;

  // subscription ptr and msg ptr
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
  autoware_adapi_v1_msgs::msg::RouteState::SharedPtr route_state_msg_;

  // subscription ptr and msg ptr
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr route_points_sub_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr route_points_msg_;

  // subscription ptr and msg ptr
  rclcpp::Subscription<autoware_map_msgs::msg::MapProjectorInfo>::SharedPtr map_projector_info_sub_;
  autoware_map_msgs::msg::MapProjectorInfo::SharedPtr map_projector_info_msg_;

  GoalPose goal_pose_;
  PathOverlay path_overlay_;

  TileProvider::Provider current_tile_provider_;
};

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // MINIMAP_HPP_
