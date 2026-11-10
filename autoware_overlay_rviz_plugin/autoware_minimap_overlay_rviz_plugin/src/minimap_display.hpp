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

#ifndef MINIMAP_DISPLAY_HPP_
#define MINIMAP_DISPLAY_HPP_

#include "minimap_painter.hpp"
#include "route_line.hpp"
#include "tile_loader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_rendering/render_window.hpp>

#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware_minimap_overlay_rviz_plugin
{

// RViz overlay that shows a map around the vehicle, with the goal and the route to it.
// RViz calls the ROS callbacks from the GUI thread, so this class needs no lock.
class MinimapDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  MinimapDisplay();

  void onInitialize() override;
  void reset() override;
  void update(float wall_dt, float ros_dt) override;

  // Handles the mouse on the minimap: the buttons, the drag of the map, and the wheel
  bool eventFilter(QObject * object, QEvent * event) override;

protected:
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateOverlayGeometry();
  void requestRedraw();
  void onTileFailed(const QString & reason);

private:
  using VehicleKinematics = autoware_adapi_v1_msgs::msg::VehicleKinematics;
  using Route = autoware_adapi_v1_msgs::msg::Route;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using MapProjectorInfo = autoware_map_msgs::msg::MapProjectorInfo;

  void subscribe();
  void unsubscribe();
  void clearInputs();

  void onKinematics(const VehicleKinematics::ConstSharedPtr msg);
  void onRoute(const Route::ConstSharedPtr msg);
  void onRouteState(const RouteState::ConstSharedPtr msg);
  void onVectorMap(const LaneletMapBin::ConstSharedPtr msg);
  void onMapProjectorInfo(const MapProjectorInfo::ConstSharedPtr msg);

  // Fills the goal and the route line from the last messages
  void updateRoute();
  // Removes the part of the route line that the vehicle passed
  void updateRemainingRoute();
  MinimapStyle style() const;
  // Rectangle of the minimap in the render window, in the pixels of the overlay
  QRectF overlayRect() const;
  void onButtonClicked(MinimapButton button);
  void changeZoom(int steps);
  // Moves the map under the mouse by a distance in pixels of the minimap
  void panView(const QPointF & distance);

  // Position of a point of the map frame on the Web Mercator map
  std::optional<QPointF> toMercator(const geometry_msgs::msg::Point & point) const;

  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::EnumProperty * horizontal_anchor_property_;
  rviz_common::properties::EnumProperty * vertical_anchor_property_;
  rviz_common::properties::IntProperty * horizontal_margin_property_;
  rviz_common::properties::IntProperty * vertical_margin_property_;
  rviz_common::properties::EnumProperty * shape_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::IntProperty * zoom_property_;
  rviz_common::properties::EnumProperty * view_property_;
  rviz_common::properties::EnumProperty * map_style_property_;

  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
  rviz_rendering::RenderWindow * render_window_ = nullptr;
  std::unique_ptr<MinimapPainter> painter_;
  std::unique_ptr<TileLoader> tiles_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<VehicleKinematics>::SharedPtr kinematics_sub_;
  rclcpp::Subscription<Route>::SharedPtr route_sub_;
  rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr vector_map_sub_;
  rclcpp::Subscription<MapProjectorInfo>::SharedPtr map_projector_info_sub_;

  Route::ConstSharedPtr route_;
  RouteState::ConstSharedPtr route_state_;
  MapProjectorInfo::ConstSharedPtr map_projector_info_;
  lanelet::LaneletMapConstPtr lanelet_map_;

  // Line of the route from its start to the goal, in the map frame and on the Web Mercator map
  std::vector<QPointF> route_line_;
  std::vector<QPointF> route_line_mercator_;
  // Position of the vehicle on the route line
  LineProgress route_progress_;
  // Position of the vehicle in the map frame
  std::optional<QPointF> vehicle_position_;

  MinimapScene scene_;
  // Vehicle pose of the last redraw, to skip the redraw when the vehicle does not move
  std::optional<QPointF> drawn_vehicle_;
  double drawn_heading_deg_ = 0.0;
  bool needs_redraw_ = true;

  // Mouse on the minimap, in pixels of the minimap
  bool mouse_pressed_ = false;
  bool mouse_dragging_ = false;
  QPointF mouse_press_position_;
  QPointF mouse_last_position_;
};

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // MINIMAP_DISPLAY_HPP_
