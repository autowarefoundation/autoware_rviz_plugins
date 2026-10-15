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
#include "include/minimap.hpp"

#include "include/web_mercator.hpp"

#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <rviz_rendering/render_system.hpp>

#include <qimage.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware_minimap_overlay_rviz_plugin
{

VehicleMapDisplay::VehicleMapDisplay()
: rviz_common::Display(), overlay_(nullptr), current_tile_provider_(TileProvider::OpenStreetMap)
{
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 256, "Width of the overlay max:500", this, SLOT(updateOverlaySize()));

  property_width_->setMin(1);
  property_width_->setMax(500);
  property_width_->setShouldBeSaved(true);

  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 256, "Height of the overlay max:500", this, SLOT(updateOverlaySize()));

  property_height_->setMin(1);
  property_height_->setMax(500);
  property_height_->setShouldBeSaved(true);

  property_vertical_margin_ = new rviz_common::properties::IntProperty(
    "Vertical Margin", 10, "Vertical margin of the overlay", this, SLOT(updateOverlayPosition()));
  property_horizontal_margin_ = new rviz_common::properties::IntProperty(
    "Horizontal Margin", 10, "Horizontal margin of the overlay", this,
    SLOT(updateOverlayPosition()));

  property_anchor_vertical_ = new rviz_common::properties::EnumProperty(
    "Vertical Anchor", "Bottom", "Vertical anchor of the overlay", this,
    SLOT(updateOverlayPosition()));

  property_anchor_vertical_->addOption(
    "Top", static_cast<int>(autoware_minimap_overlay_rviz_plugin::VerticalAlignment::TOP));
  property_anchor_vertical_->addOption(
    "Center", static_cast<int>(autoware_minimap_overlay_rviz_plugin::VerticalAlignment::CENTER));
  property_anchor_vertical_->addOption(
    "Bottom", static_cast<int>(autoware_minimap_overlay_rviz_plugin::VerticalAlignment::BOTTOM));

  property_anchor_horizontal_ = new rviz_common::properties::EnumProperty(
    "Horizontal Anchor", "Left", "Horizontal anchor of the overlay", this,
    SLOT(updateOverlayPosition()));

  property_anchor_horizontal_->addOption(
    "Left", static_cast<int>(autoware_minimap_overlay_rviz_plugin::HorizontalAlignment::LEFT));
  property_anchor_horizontal_->addOption(
    "Center", static_cast<int>(autoware_minimap_overlay_rviz_plugin::HorizontalAlignment::CENTER));
  property_anchor_horizontal_->addOption(
    "Right", static_cast<int>(autoware_minimap_overlay_rviz_plugin::HorizontalAlignment::RIGHT));

  property_border_radius_ = new rviz_common::properties::IntProperty(
    "Border Radius", property_height_->getInt() / 2.0, "Border radius of the overlay", this,
    SLOT(updateOverlaySize()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Amount of transparency to apply to the overlay.", this,
    SLOT(updateOverlaySize()));

  property_tile_provider_ = new rviz_common::properties::EnumProperty(
    "Tile Provider", "OpenStreetMap", "Select the tile provider", this, SLOT(updateTileProvider()));

  property_tile_provider_->addOption("OpenStreetMap", TileProvider::OpenStreetMap);

  property_zoom_ = new rviz_common::properties::IntProperty(
    "Zoom", 15, "Zoom level of the map 15-18", this, SLOT(updateZoomLevel()));

  property_zoom_->setMin(15);
  property_zoom_->setMax(18);
  property_zoom_->setShouldBeSaved(true);

  // Read-only properties that show the received positions
  property_latitude_ =
    new rviz_common::properties::FloatProperty("Latitude", 0.0, "Latitude of the ego", this);
  property_longitude_ =
    new rviz_common::properties::FloatProperty("Longitude", 0.0, "Longitude of the ego", this);
  property_goal_lat =
    new rviz_common::properties::FloatProperty("Goal Latitude", 0.0, "Goal pose Latitude", this);
  property_goal_lon =
    new rviz_common::properties::FloatProperty("Goal Longitude", 0.0, "Goal pose Longitude", this);
  for (auto * property :
       {property_latitude_, property_longitude_, property_goal_lat, property_goal_lon}) {
    property->setReadOnly(true);
    property->setShouldBeSaved(false);
  }

  zoom_ = property_zoom_->getInt();

  tile_field_ = std::make_unique<TileField>(this);
  tile_field_->setUrlTemplate(TileProvider::getUrlTemplate(current_tile_provider_));

  connect(tile_field_.get(), &TileField::tilesUpdated, this, &VehicleMapDisplay::onTilesUpdated);
}

VehicleMapDisplay::~VehicleMapDisplay()
{
}

void VehicleMapDisplay::onInitialize()
{
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "AerialMapDisplayObject" << count++;
  overlay_.reset(new autoware_minimap_overlay_rviz_plugin::OverlayObject(ss.str()));
  overlay_->show();
  updateOverlaySize();
  updateOverlayPosition();

  const std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_minimap_overlay_rviz_plugin");
  pos_pixmap_ = QPixmap(QString::fromStdString(package_path + "/icons/pos.png"))
                  .scaled(25, 25, Qt::KeepAspectRatio, Qt::SmoothTransformation);

  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // Route and route state are published with transient local durability
  rclcpp::QoS qos_settings = rclcpp::QoS(1).reliable().transient_local();

  route_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::Route>(
    "/api/routing/route", qos_settings,
    std::bind(&VehicleMapDisplay::routeCallback, this, std::placeholders::_1));

  pose_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
    "/api/vehicle/kinematics", rclcpp::QoS(10).best_effort(),
    std::bind(&VehicleMapDisplay::poseCallback, this, std::placeholders::_1));

  route_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", qos_settings,
    std::bind(&VehicleMapDisplay::routeStateCallback, this, std::placeholders::_1));

  route_points_sub_ = node_->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", 10,
    std::bind(&VehicleMapDisplay::routePointsCallback, this, std::placeholders::_1));

  map_projector_info_sub_ = node_->create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
    "/map/map_projector_info", qos_settings,
    std::bind(&VehicleMapDisplay::mapProjectorInfoCallback, this, std::placeholders::_1));
}

void VehicleMapDisplay::mapProjectorInfoCallback(
  const autoware_map_msgs::msg::MapProjectorInfo::SharedPtr msg)
{
  if (msg->projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "Minimap plugin cannot show the goal pose and the path with a Local map projector");
    map_projector_info_msg_ = nullptr;
  } else {
    map_projector_info_msg_ = msg;
  }

  updateGoalPose();
  updatePath();
}

std::optional<std::pair<double, double>> VehicleMapDisplay::localToLatLon(
  const geometry_msgs::msg::Point & point)
{
  if (!map_projector_info_msg_) {
    return std::nullopt;
  }
  try {
    const auto geo_point =
      autoware::geography_utils::project_reverse(point, *map_projector_info_msg_);
    return std::make_pair(geo_point.latitude, geo_point.longitude);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Minimap plugin failed to project a local point: " << e.what());
    return std::nullopt;
  }
}

void VehicleMapDisplay::reset()
{
}

void VehicleMapDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  updateMapPosition();
}

void VehicleMapDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void VehicleMapDisplay::update(float, float)
{
  if (!overlay_ || !needs_redraw_) {
    return;
  }
  needs_redraw_ = false;

  autoware_minimap_overlay_rviz_plugin::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(Qt::transparent);
  drawWidget(hud);
}

void VehicleMapDisplay::requestRedraw()
{
  needs_redraw_ = true;
  queueRender();
}

void VehicleMapDisplay::updateTileProvider()
{
  current_tile_provider_ =
    static_cast<TileProvider::Provider>(property_tile_provider_->getOptionInt());
  tile_field_->setUrlTemplate(TileProvider::getUrlTemplate(current_tile_provider_));
  updateMapPosition();
}

void VehicleMapDisplay::updateOverlaySize()
{
  if (!overlay_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  requestRedraw();
}

void VehicleMapDisplay::updateOverlayPosition()
{
  if (!overlay_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->setPosition(
    property_horizontal_margin_->getInt(), property_vertical_margin_->getInt(),
    static_cast<HorizontalAlignment>(property_anchor_horizontal_->getOptionInt()),
    static_cast<VerticalAlignment>(property_anchor_vertical_->getOptionInt()));
  requestRedraw();
}

void VehicleMapDisplay::drawWidget(QImage & hud)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (!overlay_->isVisible()) {
    return;
  }

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QRectF backgroundRect(0, 0, hud.width(), hud.height());
  drawCircle(painter, backgroundRect);

  painter.end();
}

void VehicleMapDisplay::drawCircle(QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setOpacity(alpha_property_->getFloat());

  // Clip everything to a rounded rectangle
  QPainterPath path;
  path.setFillRule(Qt::WindingFill);
  const qreal radius = property_border_radius_->getInt();
  path.addRoundedRect(backgroundRect, radius, radius);
  painter.setClipPath(path.simplified());

  // Place the tile field so that the vehicle position is at the center of the overlay
  const QImage tile_field_image = tile_field_->getTileFieldImage();
  const QPointF vehicle_world_pixel =
    autoware_minimap_overlay_rviz_plugin::lat_lon_to_world_pixel(latitude_, longitude_, zoom_);
  painter.drawImage(
    backgroundRect.center() + tile_field_->getTileFieldOrigin() - vehicle_world_pixel,
    tile_field_image);

  if (route_state_msg_ && route_state_msg_->state == autoware_adapi_v1_msgs::msg::RouteState::SET) {
    path_overlay_.draw(painter, backgroundRect, zoom_);
    goal_pose_.draw(painter, backgroundRect, zoom_);
  }

  // Draw the position icon at the center, rotated to the heading of the vehicle
  painter.save();
  painter.translate(backgroundRect.center());
  if (pose_msg_) {
    const auto & orientation = pose_msg_->pose.pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // The icon points up, which is north; the yaw is counterclockwise from east
    painter.rotate((M_PI / 2 - yaw) * 180.0 / M_PI);
  }
  painter.drawPixmap(QPointF(-pos_pixmap_.width() / 2.0, -pos_pixmap_.height() / 2.0), pos_pixmap_);
  painter.restore();

  drawAttribution(painter, backgroundRect);
}

void VehicleMapDisplay::drawAttribution(QPainter & painter, const QRectF & backgroundRect)
{
  // Required by the OpenStreetMap tile usage policy and the ODbL
  const QString text = QString::fromUtf8("© OpenStreetMap contributors");
  QFont font = painter.font();
  font.setPixelSize(9);
  painter.setFont(font);

  const QFontMetricsF metrics(font);
  const QSizeF size(metrics.horizontalAdvance(text) + 6.0, metrics.height() + 2.0);
  // Keep the text inside the rounded corners
  const qreal bottom_offset = std::min(property_border_radius_->getInt() * 0.3, 40.0) + 4.0;
  const QRectF text_rect(
    backgroundRect.center().x() - size.width() / 2.0,
    backgroundRect.bottom() - bottom_offset - size.height(), size.width(), size.height());

  painter.save();
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(255, 255, 255, 180));
  painter.drawRoundedRect(text_rect, 2.0, 2.0);
  painter.setPen(QColor(60, 60, 60));
  painter.drawText(text_rect, Qt::AlignCenter, text);
  painter.restore();
}

void VehicleMapDisplay::onTilesUpdated()
{
  requestRedraw();
}

void VehicleMapDisplay::updateZoomLevel()
{
  zoom_ = property_zoom_->getInt();
  updateMapPosition();
}

void VehicleMapDisplay::updateMapPosition()
{
  if (!has_vehicle_position_ || !isEnabled()) {
    return;
  }
  tile_field_->setCenter(latitude_, longitude_, zoom_);
  requestRedraw();
}

void VehicleMapDisplay::updateGoalPose()
{
  const auto goal = (route_msg_ && !route_msg_->data.empty())
                      ? localToLatLon(route_msg_->data.front().goal.position)
                      : std::nullopt;
  if (!goal) {
    goal_pose_.clear();
    requestRedraw();
    return;
  }
  goal_pose_.setGoalPosition(goal->first, goal->second);
  property_goal_lat->setFloat(goal->first);
  property_goal_lon->setFloat(goal->second);
  requestRedraw();
}

void VehicleMapDisplay::updatePath()
{
  std::vector<std::pair<double, double>> path_points_geo;
  if (route_points_msg_ && map_projector_info_msg_) {
    path_points_geo.reserve(route_points_msg_->points.size());
    for (const auto & point : route_points_msg_->points) {
      const auto geo_point = localToLatLon(point.pose.position);
      if (!geo_point) {
        path_points_geo.clear();
        break;
      }
      path_points_geo.push_back(*geo_point);
    }
  }
  path_overlay_.setPathPoints(std::move(path_points_geo));
  requestRedraw();
}

void VehicleMapDisplay::routePointsCallback(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  route_points_msg_ = msg;
  updatePath();
}

void VehicleMapDisplay::routeStateCallback(
  const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  route_state_msg_ = msg;
  requestRedraw();
}

void VehicleMapDisplay::poseCallback(
  const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg)
{
  const double lat = msg->geographic_pose.position.latitude;
  const double lon = msg->geographic_pose.position.longitude;
  if (!std::isfinite(lat) || !std::isfinite(lon)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Minimap plugin received a vehicle position that is not finite");
    return;
  }

  pose_msg_ = msg;
  latitude_ = clamp_latitude(lat);
  longitude_ = lon;
  has_vehicle_position_ = true;

  property_latitude_->setFloat(latitude_);
  property_longitude_->setFloat(longitude_);

  goal_pose_.setVehiclePosition(latitude_, longitude_);
  path_overlay_.setVehiclePosition(latitude_, longitude_);

  updateMapPosition();
}

void VehicleMapDisplay::routeCallback(const autoware_adapi_v1_msgs::msg::Route::SharedPtr msg)
{
  route_msg_ = msg;
  updateGoalPose();
}

}  // namespace autoware_minimap_overlay_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware_minimap_overlay_rviz_plugin::VehicleMapDisplay, rviz_common::Display)
