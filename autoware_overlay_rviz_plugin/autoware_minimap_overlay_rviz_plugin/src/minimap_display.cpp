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

#include "minimap_display.hpp"

#include "web_mercator.hpp"

#include <QDir>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QStandardPaths>
#include <QWheelEvent>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_system.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>

namespace autoware_minimap_overlay_rviz_plugin
{

namespace
{
using rviz_2d_overlay_plugins::HorizontalAlignment;
using rviz_2d_overlay_plugins::VerticalAlignment;
using rviz_common::properties::StatusProperty;

constexpr char default_tile_url[] = "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
// The tiles of tile.openstreetmap.org end at zoom level 19
constexpr int max_zoom = 19;
constexpr int min_zoom = 3;

// Names of the status lines in the Displays panel
constexpr char vehicle_status[] = "Vehicle";
constexpr char projector_status[] = "Map projector";
constexpr char route_status[] = "Route";
constexpr char tiles_status[] = "Map tiles";
constexpr char icons_status[] = "Icons";

// A smaller movement of the vehicle does not change the image
constexpr double redraw_distance_px = 0.25;
constexpr double redraw_heading_deg = 0.5;

QString format_position(const double latitude, const double longitude)
{
  return QString("%1, %2").arg(latitude, 0, 'f', 5).arg(longitude, 0, 'f', 5);
}
}  // namespace

MinimapDisplay::MinimapDisplay()
{
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 256, "Width of the minimap in pixels", this, SLOT(updateOverlayGeometry()));
  width_property_->setMin(64);
  width_property_->setMax(1024);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 256, "Height of the minimap in pixels", this, SLOT(updateOverlayGeometry()));
  height_property_->setMin(64);
  height_property_->setMax(1024);

  horizontal_anchor_property_ = new rviz_common::properties::EnumProperty(
    "Horizontal Anchor", "Left", "Side of the RViz view that the minimap is attached to", this,
    SLOT(updateOverlayGeometry()));
  horizontal_anchor_property_->addOption("Left", static_cast<int>(HorizontalAlignment::LEFT));
  horizontal_anchor_property_->addOption("Center", static_cast<int>(HorizontalAlignment::CENTER));
  horizontal_anchor_property_->addOption("Right", static_cast<int>(HorizontalAlignment::RIGHT));

  vertical_anchor_property_ = new rviz_common::properties::EnumProperty(
    "Vertical Anchor", "Bottom", "Side of the RViz view that the minimap is attached to", this,
    SLOT(updateOverlayGeometry()));
  vertical_anchor_property_->addOption("Top", static_cast<int>(VerticalAlignment::TOP));
  vertical_anchor_property_->addOption("Center", static_cast<int>(VerticalAlignment::CENTER));
  vertical_anchor_property_->addOption("Bottom", static_cast<int>(VerticalAlignment::BOTTOM));

  horizontal_margin_property_ = new rviz_common::properties::IntProperty(
    "Horizontal Margin", 10, "Horizontal distance from the anchor in pixels", this,
    SLOT(updateOverlayGeometry()));

  vertical_margin_property_ = new rviz_common::properties::IntProperty(
    "Vertical Margin", 10, "Vertical distance from the anchor in pixels", this,
    SLOT(updateOverlayGeometry()));

  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Squircle", "Shape of the minimap. A squircle is a square with smooth corners.", this,
    SLOT(requestRedraw()));
  shape_property_->addOption("Squircle", static_cast<int>(MinimapShape::Squircle));
  shape_property_->addOption("Circle", static_cast<int>(MinimapShape::Circle));
  shape_property_->addOption("Rectangle", static_cast<int>(MinimapShape::Rectangle));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.6, "Opacity of the minimap, from 0 to 1", this, SLOT(requestRedraw()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  zoom_property_ = new rviz_common::properties::IntProperty(
    "Zoom", 17,
    "Zoom level of the map. One level more shows half of the distance. The mouse wheel "
    "and the buttons on the minimap change it.",
    this, SLOT(requestRedraw()));
  zoom_property_->setMin(min_zoom);
  zoom_property_->setMax(max_zoom);

  view_property_ = new rviz_common::properties::EnumProperty(
    "View", "3D",
    "North Up keeps north at the top. Heading Up turns the map so that the vehicle points to the "
    "top. 3D also tilts the map. The compass button on the minimap changes the view.",
    this, SLOT(requestRedraw()));
  view_property_->addOption("3D", static_cast<int>(MinimapView::Perspective));
  view_property_->addOption("Heading Up", static_cast<int>(MinimapView::HeadingUp));
  view_property_->addOption("North Up", static_cast<int>(MinimapView::NorthUp));

  map_style_property_ = new rviz_common::properties::EnumProperty(
    "Map Style", "Navigation Dark",
    "The navigation styles show mainly the roads. OpenStreetMap shows the map tiles as they are. "
    "The layers button on the minimap changes the style.",
    this, SLOT(requestRedraw()));
  map_style_property_->addOption("Navigation Dark", static_cast<int>(MapStyle::NavigationDark));
  map_style_property_->addOption("Navigation Light", static_cast<int>(MapStyle::NavigationLight));
  map_style_property_->addOption("OpenStreetMap", static_cast<int>(MapStyle::Standard));

  tile_url_property_ = new rviz_common::properties::StringProperty(
    "Tile URL", default_tile_url,
    "Address of the map tiles, with the placeholders {z}, {x} and {y}. The tiles must have the "
    "colors of the standard OpenStreetMap style for the navigation styles.",
    this, SLOT(updateTileUrl()));
}

void MinimapDisplay::onInitialize()
{
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int overlay_count = 0;
  overlay_ = std::make_shared<rviz_2d_overlay_plugins::OverlayObject>(
    "MinimapDisplayOverlay" + std::to_string(overlay_count++));
  updateOverlayGeometry();

  const std::string share_directory =
    ament_index_cpp::get_package_share_directory("autoware_minimap_overlay_rviz_plugin");
  painter_ = std::make_unique<MinimapPainter>(QString::fromStdString(share_directory + "/icons"));
  if (!painter_->isValid()) {
    setStatus(StatusProperty::Error, icons_status, "The icon files did not load");
  }

  tiles_ = std::make_unique<TileLoader>(
    tile_url_property_->getString(),
    QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation))
      .filePath("autoware_minimap_overlay_rviz_plugin"));
  connect(tiles_.get(), &TileLoader::tileLoaded, this, [this]() {
    setStatus(StatusProperty::Ok, tiles_status, "OK");
    requestRedraw();
  });
  connect(tiles_.get(), &TileLoader::tileFailed, this, &MinimapDisplay::onTileFailed);

  // The overlay is drawn in the render window, which gets the mouse events
  render_window_ = context_->getViewManager()->getRenderPanel()->getRenderWindow();
  render_window_->installEventFilter(this);

  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  clearInputs();
  if (isEnabled()) {
    overlay_->show();
    subscribe();
  }
}

void MinimapDisplay::onEnable()
{
  if (!overlay_) {
    // RViz enables the display before it calls onInitialize()
    return;
  }
  overlay_->show();
  subscribe();
  requestRedraw();
}

void MinimapDisplay::onDisable()
{
  if (!overlay_) {
    return;
  }
  unsubscribe();
  // No requests for tiles while the minimap does not show
  tiles_->pause();
  mouse_.reset();
  overlay_->hide();
}

void MinimapDisplay::reset()
{
  rviz_common::Display::reset();
  // Display::reset() deletes the status lines
  if (painter_ && !painter_->isValid()) {
    setStatus(StatusProperty::Error, icons_status, "The icon files did not load");
  }
  // A new subscription gets the transient local messages again
  unsubscribe();
  clearInputs();
  if (isEnabled()) {
    subscribe();
  }
}

void MinimapDisplay::subscribe()
{
  if (!node_ || kinematics_sub_) {
    return;
  }
  using std::placeholders::_1;
  const rclcpp::QoS transient_local = rclcpp::QoS(1).transient_local();

  if (!vector_map_sub_) {
    vector_map_sub_ = node_->create_subscription<LaneletMapBin>(
      "/map/vector_map", transient_local, std::bind(&MinimapDisplay::onVectorMap, this, _1));
  }

  kinematics_sub_ = node_->create_subscription<VehicleKinematics>(
    "/api/vehicle/kinematics", rclcpp::QoS(1).best_effort(),
    std::bind(&MinimapDisplay::onKinematics, this, _1));
  route_sub_ = node_->create_subscription<Route>(
    "/api/routing/route", transient_local, std::bind(&MinimapDisplay::onRoute, this, _1));
  route_state_sub_ = node_->create_subscription<RouteState>(
    "/api/routing/state", transient_local, std::bind(&MinimapDisplay::onRouteState, this, _1));
  map_projector_info_sub_ = node_->create_subscription<MapProjectorInfo>(
    "/map/map_projector_info", transient_local,
    std::bind(&MinimapDisplay::onMapProjectorInfo, this, _1));
}

void MinimapDisplay::unsubscribe()
{
  kinematics_sub_.reset();
  route_sub_.reset();
  route_state_sub_.reset();
  map_projector_info_sub_.reset();
  // The subscription to the map stays. A new one gets the map again, and RViz resets the
  // displays at each jump back in time, for example at each loop of a rosbag. The conversion of
  // a large map blocks the GUI thread for a noticeable time.
}

void MinimapDisplay::clearInputs()
{
  route_.reset();
  route_state_.reset();
  map_projector_info_.reset();
  route_line_.clear();
  route_line_mercator_.clear();
  route_progress_ = LineProgress();
  vehicle_position_.reset();
  scene_ = MinimapScene();
  mouse_.reset();

  setStatus(StatusProperty::Warn, vehicle_status, "No message received");
  setStatus(StatusProperty::Warn, projector_status, "No message received");
  setStatus(StatusProperty::Ok, route_status, "No route");
  requestRedraw();
}

void MinimapDisplay::updateOverlayGeometry()
{
  if (!overlay_) {
    return;
  }
  overlay_->updateTextureSize(width_property_->getInt(), height_property_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  overlay_->setPosition(
    horizontal_margin_property_->getInt(), vertical_margin_property_->getInt(),
    static_cast<HorizontalAlignment>(horizontal_anchor_property_->getOptionInt()),
    static_cast<VerticalAlignment>(vertical_anchor_property_->getOptionInt()));
  requestRedraw();
}

void MinimapDisplay::requestRedraw()
{
  needs_redraw_ = true;
  queueRender();
}

void MinimapDisplay::updateTileUrl()
{
  if (tiles_) {
    tiles_->setUrlTemplate(tile_url_property_->getString());
  }
  requestRedraw();
}

void MinimapDisplay::onTileFailed(const QString & reason)
{
  setStatus(StatusProperty::Warn, tiles_status, reason);
}

void MinimapDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  if (!overlay_ || !needs_redraw_) {
    return;
  }
  needs_redraw_ = false;

  scene_.zoom = zoom_property_->getInt();
  updateRemainingRoute();
  drawn_vehicle_ = scene_.vehicle;
  drawn_heading_deg_ = scene_.vehicle_heading_deg;

  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(*overlay_);
  image.fill(Qt::transparent);

  const MinimapStyle style = this->style();

  if (scene_.vehicle) {
    tiles_->setStyle(style.map);
    tiles_->setView(
      scene_.view_center.value_or(*scene_.vehicle),
      MinimapPainter::tileZoom(image.rect(), style, scene_.zoom),
      MinimapPainter::tileViewSize(image.rect(), style, scene_.zoom));
  }

  QPainter painter(&image);
  painter_->paint(painter, image.rect(), style, scene_, tiles_.get());

  // Apply the opacity to the finished image, so that the layers do not show through each other
  painter.setCompositionMode(QPainter::CompositionMode_DestinationIn);
  painter.fillRect(image.rect(), QColor(0, 0, 0, std::lround(alpha_property_->getFloat() * 255)));
}

MinimapStyle MinimapDisplay::style() const
{
  MinimapStyle style;
  style.shape = static_cast<MinimapShape>(shape_property_->getOptionInt());
  style.map = static_cast<MapStyle>(map_style_property_->getOptionInt());
  style.view = static_cast<MinimapView>(view_property_->getOptionInt());
  return style;
}

QRectF MinimapDisplay::overlayRect() const
{
  // The same placement as OverlayObject::setPosition()
  const double scale = render_window_->devicePixelRatio();
  const double window_width = render_window_->width() * scale;
  const double window_height = render_window_->height() * scale;
  const double width = width_property_->getInt();
  const double height = height_property_->getInt();
  const double horizontal_margin = horizontal_margin_property_->getInt();
  const double vertical_margin = vertical_margin_property_->getInt();

  double left = horizontal_margin;
  switch (static_cast<HorizontalAlignment>(horizontal_anchor_property_->getOptionInt())) {
    case HorizontalAlignment::CENTER:
      left = window_width / 2.0 + horizontal_margin - width / 2.0;
      break;
    case HorizontalAlignment::RIGHT:
      left = window_width - horizontal_margin - width;
      break;
    default:
      break;
  }
  double top = vertical_margin;
  switch (static_cast<VerticalAlignment>(vertical_anchor_property_->getOptionInt())) {
    case VerticalAlignment::CENTER:
      top = window_height / 2.0 + vertical_margin - height / 2.0;
      break;
    case VerticalAlignment::BOTTOM:
      top = window_height - vertical_margin - height;
      break;
    default:
      break;
  }
  return {left, top, width, height};
}

bool MinimapDisplay::eventFilter(QObject * object, QEvent * event)
{
  if (object != render_window_ || !overlay_ || !isEnabled() || !scene_.vehicle) {
    return false;
  }
  const QRectF overlay_rect = overlayRect();
  const QRectF rect(QPointF(0.0, 0.0), overlay_rect.size());
  const MinimapStyle style = this->style();
  const double scale = render_window_->devicePixelRatio();
  // Position in the pixels of the minimap
  const auto to_minimap = [&](const QPointF & window_position) {
    return window_position * scale - overlay_rect.topLeft();
  };
  const auto is_on_minimap = [&](const QPointF & position) {
    return MinimapPainter::outline(rect, style.shape).contains(position);
  };

  switch (event->type()) {
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonDblClick: {
      const auto * mouse_event = static_cast<QMouseEvent *>(event);
      const QPointF position = to_minimap(mouse_event->localPos());
      // The tools of RViz must not get a click on the minimap
      return mouse_.press(mouse_event->button(), position, is_on_minimap(position));
    }
    case QEvent::MouseMove: {
      const auto * mouse_event = static_cast<QMouseEvent *>(event);
      const auto motion =
        mouse_.motion(mouse_event->buttons(), to_minimap(mouse_event->localPos()));
      if (!motion.drag.isNull()) {
        panView(motion.drag);
      }
      return motion.consumed;
    }
    case QEvent::MouseButtonRelease: {
      const auto * mouse_event = static_cast<QMouseEvent *>(event);
      const auto release = mouse_.release(mouse_event->button());
      if (release.click) {
        const auto button = MinimapPainter::buttonAt(
          to_minimap(mouse_event->localPos()), rect, style, scene_.view_center.has_value());
        if (button) {
          onButtonClicked(*button);
        }
      }
      return release.consumed;
    }
    case QEvent::Wheel: {
      const auto * wheel_event = static_cast<QWheelEvent *>(event);
      if (!is_on_minimap(to_minimap(wheel_event->position()))) {
        return false;
      }
      const int steps = mouse_.wheel(wheel_event->angleDelta().y());
      if (steps != 0) {
        changeZoom(steps);
      }
      return true;
    }
    default:
      return false;
  }
}

void MinimapDisplay::onButtonClicked(const MinimapButton button)
{
  switch (button) {
    case MinimapButton::ZoomIn:
      changeZoom(1);
      break;
    case MinimapButton::ZoomOut:
      changeZoom(-1);
      break;
    case MinimapButton::Layers:
      switch (static_cast<MapStyle>(map_style_property_->getOptionInt())) {
        case MapStyle::NavigationDark:
          map_style_property_->setString("Navigation Light");
          break;
        case MapStyle::NavigationLight:
          map_style_property_->setString("OpenStreetMap");
          break;
        default:
          map_style_property_->setString("Navigation Dark");
          break;
      }
      break;
    case MinimapButton::Compass:
      // 3D, then Heading Up, then North Up, then 3D again
      switch (static_cast<MinimapView>(view_property_->getOptionInt())) {
        case MinimapView::Perspective:
          view_property_->setString("Heading Up");
          break;
        case MinimapView::HeadingUp:
          view_property_->setString("North Up");
          break;
        default:
          view_property_->setString("3D");
          break;
      }
      break;
    case MinimapButton::Recenter:
      scene_.view_center.reset();
      requestRedraw();
      break;
  }
}

void MinimapDisplay::changeZoom(const int steps)
{
  zoom_property_->setInt(std::clamp(zoom_property_->getInt() + steps, min_zoom, max_zoom));
}

void MinimapDisplay::panView(const QPointF & distance)
{
  if (!scene_.vehicle) {
    return;
  }
  // Turn the distance on the minimap back to a distance on the map with north up.
  // The tilt of the 3D view is not taken out: the map follows the mouse only near the vehicle.
  QTransform rotation;
  rotation.rotate(-MinimapPainter::mapRotation(style(), scene_));
  const QRectF rect(0.0, 0.0, width_property_->getInt(), height_property_->getInt());
  const QPointF on_map = rotation.map(distance) / MinimapPainter::designScale(rect) /
                         map_size_px(zoom_property_->getInt());

  QPointF center = scene_.view_center.value_or(*scene_.vehicle) - on_map;
  center.setY(std::clamp(center.y(), 0.0, 1.0));
  scene_.view_center = center;
  requestRedraw();
}

void MinimapDisplay::onKinematics(const VehicleKinematics::ConstSharedPtr msg)
{
  const double latitude = msg->geographic_pose.position.latitude;
  const double longitude = msg->geographic_pose.position.longitude;

  if (map_projector_info_ && map_projector_info_->projector_type == MapProjectorInfo::LOCAL) {
    // A map with a local projector gives no geographic pose
    return;
  }
  // The comparisons are false for NaN
  if (!(std::abs(latitude) <= 90.0) || !(std::abs(longitude) <= 180.0)) {
    setStatus(
      StatusProperty::Error, vehicle_status, "The geographic position is not a valid position");
    return;
  }

  // Yaw from the quaternion. A zero quaternion gives 0, not NaN.
  const auto & q = msg->pose.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (!std::isfinite(yaw)) {
    setStatus(StatusProperty::Error, vehicle_status, "The orientation is not finite");
    return;
  }

  scene_.vehicle = to_mercator(latitude, longitude);
  // The yaw is counterclockwise from east
  scene_.vehicle_heading_deg = 90.0 - yaw * 180.0 / M_PI;
  setStatus(StatusProperty::Ok, vehicle_status, format_position(latitude, longitude));

  const auto & position = msg->pose.pose.pose.position;
  vehicle_position_ = QPointF(position.x, position.y);

  if (drawn_vehicle_) {
    const QPointF moved_px = (*scene_.vehicle - *drawn_vehicle_) * map_size_px(scene_.zoom);
    const double turned_deg =
      std::abs(std::remainder(scene_.vehicle_heading_deg - drawn_heading_deg_, 360.0));
    if (
      std::hypot(moved_px.x(), moved_px.y()) < redraw_distance_px &&
      turned_deg < redraw_heading_deg) {
      return;
    }
  }
  requestRedraw();
}

void MinimapDisplay::onMapProjectorInfo(const MapProjectorInfo::ConstSharedPtr msg)
{
  map_projector_info_ = msg;
  if (msg->projector_type == MapProjectorInfo::LOCAL) {
    scene_.vehicle.reset();
    scene_.waiting_message = "The map has a local\nprojector, so it has no\ngeographic position";
    mouse_.reset();
    setStatus(
      StatusProperty::Error, projector_status,
      "The map has a local projector, so it has no geographic position");
    setStatus(StatusProperty::Error, vehicle_status, "No geographic position");
  } else {
    scene_.waiting_message = MinimapScene().waiting_message;
    setStatus(StatusProperty::Ok, projector_status, QString::fromStdString(msg->projector_type));
  }
  updateRoute();
}

void MinimapDisplay::onRoute(const Route::ConstSharedPtr msg)
{
  route_ = msg;
  updateRoute();
}

void MinimapDisplay::onRouteState(const RouteState::ConstSharedPtr msg)
{
  route_state_ = msg;
  updateRoute();
}

void MinimapDisplay::onVectorMap(const LaneletMapBin::ConstSharedPtr msg)
{
  try {
    lanelet_map_ = to_lanelet_map(*msg);
  } catch (const std::exception & e) {
    // Without the map, the old route line must not stay
    lanelet_map_.reset();
    updateRoute();
    setStatus(StatusProperty::Error, route_status, QString("The map did not load: ") + e.what());
    return;
  }
  updateRoute();
}

void MinimapDisplay::updateRoute()
{
  scene_.goal.reset();
  route_line_.clear();
  route_line_mercator_.clear();
  route_progress_ = LineProgress();

  // A route that is unset can leave an old route message behind
  const bool has_route =
    route_state_ && route_ && !route_->data.empty() &&
    (route_state_->state == RouteState::SET || route_state_->state == RouteState::ARRIVED ||
     route_state_->state == RouteState::CHANGING);

  if (has_route) {
    const auto & route = route_->data.front();
    scene_.goal = toMercator(route.goal.position);

    if (lanelet_map_ && scene_.goal) {
      for (const auto & point : route_centerline(*lanelet_map_, route)) {
        const auto position = toMercator(point);
        if (!position) {
          route_line_.clear();
          route_line_mercator_.clear();
          break;
        }
        route_line_.emplace_back(point.x, point.y);
        route_line_mercator_.push_back(*position);
      }
    }
  }

  if (!has_route) {
    setStatus(StatusProperty::Ok, route_status, "No route");
  } else if (!scene_.goal) {
    setStatus(StatusProperty::Warn, route_status, "The goal has no geographic position");
  } else if (!lanelet_map_) {
    setStatus(StatusProperty::Warn, route_status, "No map received, so only the goal shows");
  } else if (route_line_.empty()) {
    setStatus(
      StatusProperty::Warn, route_status,
      "The map does not have the lanelets of the route, so only the goal shows");
  } else {
    setStatus(StatusProperty::Ok, route_status, "Set");
  }

  requestRedraw();
}

void MinimapDisplay::updateRemainingRoute()
{
  if (vehicle_position_) {
    route_progress_ = advance_progress(route_line_, route_progress_, *vehicle_position_);
  }
  scene_.route = remaining_line(route_line_mercator_, route_progress_);
}

std::optional<QPointF> MinimapDisplay::toMercator(const geometry_msgs::msg::Point & point) const
{
  if (!map_projector_info_ || map_projector_info_->projector_type == MapProjectorInfo::LOCAL) {
    return std::nullopt;
  }
  try {
    const auto geo_point = autoware::geography_utils::project_reverse(point, *map_projector_info_);
    if (!std::isfinite(geo_point.latitude) || !std::isfinite(geo_point.longitude)) {
      return std::nullopt;
    }
    return to_mercator(geo_point.latitude, geo_point.longitude);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace autoware_minimap_overlay_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware_minimap_overlay_rviz_plugin::MinimapDisplay, rviz_common::Display)
