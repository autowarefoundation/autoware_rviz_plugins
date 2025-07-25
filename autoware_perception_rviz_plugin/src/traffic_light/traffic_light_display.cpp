//
//  Copyright 2025 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware_perception_rviz_plugin/traffic_light/traffic_light_display.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <Ogre.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware_perception_rviz_plugin::traffic_light
{

std::string elementToString(const autoware_perception_msgs::msg::TrafficLightElement & element)
{
  using autoware_perception_msgs::msg::TrafficLightElement;

  static const std::map<uint8_t, std::string> circle_color_map = {
    {TrafficLightElement::RED, "RED"},
    {TrafficLightElement::AMBER, "AMBER"},
    {TrafficLightElement::GREEN, "GREEN"}};

  static const std::map<uint8_t, std::string> arrow_map = {
    {TrafficLightElement::LEFT_ARROW, "LEFT"},
    {TrafficLightElement::RIGHT_ARROW, "RIGHT"},
    {TrafficLightElement::UP_ARROW, "UP"},
    {TrafficLightElement::DOWN_ARROW, "DOWN"},
    {TrafficLightElement::DOWN_LEFT_ARROW, "DOWN_LEFT"},
    {TrafficLightElement::DOWN_RIGHT_ARROW, "DOWN_RIGHT"},
    {TrafficLightElement::UP_LEFT_ARROW, "UP_LEFT"},
    {TrafficLightElement::UP_RIGHT_ARROW, "UP_RIGHT"}};

  if (element.shape == TrafficLightElement::CIRCLE) {
    if (auto it = circle_color_map.find(element.color); it != circle_color_map.end()) {
      return it->second;
    }
  } else if (auto it = arrow_map.find(element.shape); it != arrow_map.end()) {
    return it->second;
  }

  return "UNKNOWN";
}

std::vector<TrafficLightInfo> getTrafficLightInfo(
  const lanelet::LaneletMap & map, const lanelet::Id & traffic_light_id)
{
  std::vector<TrafficLightInfo> traffic_lights;
  auto traffic_light_reg_elems = map.regulatoryElementLayer.find(traffic_light_id);
  if (traffic_light_reg_elems == map.regulatoryElementLayer.end()) {
    return {};
  }
  auto traffic_light =
    dynamic_cast<const lanelet::autoware::AutowareTrafficLight *>(traffic_light_reg_elems->get());
  if (!traffic_light) {
    return {};
  }
  for (const auto & traffic_light_linestring : traffic_light->trafficLights()) {
    auto id = traffic_light_linestring.id();
    auto p1 = traffic_light_linestring.lineString()->front();
    auto p2 = traffic_light_linestring.lineString()->back();
    TrafficLightInfo traffic_light_info;
    traffic_light_info.id = id;
    traffic_light_info.linestring = *traffic_light_linestring.lineString();
    traffic_light_info.height =
      traffic_light_linestring.lineString()->attribute("height").as<double>();

    for (const auto & bulb : traffic_light->lightBulbs()) {
      for (const auto & point : bulb) {
        TrafficLightBulbInfo bulb_info;
        bulb_info.id = point.id();
        bulb_info.color = point.attributeOr("color", "none");
        bulb_info.shape = point.attributeOr("arrow", "none");
        bulb_info.position = Point3d{point.x(), point.y(), point.z()};
        traffic_light_info.bulbs.emplace_back(bulb_info);
      }
    }
    traffic_lights.emplace_back(traffic_light_info);
  }
  return traffic_lights;
}

TrafficLightBulbInfo TrafficLightInfo::getEstimatedBulb(const std::string & color) const
{
  TrafficLightBulbInfo dummy_bulb;

  // Define color-specific offset values for ID generation
  static const std::unordered_map<std::string, int> color_offsets = {
    {"red", 1}, {"yellow", 2}, {"green", 3}};

  // Generate unique ID: multiply TrafficLightInfo's ID by 1000 and add color offset
  // Example: For TrafficLightInfo.id = 42 and color = "red"
  // dummy_bulb.id = 42000 + 1 = 42001
  const int color_offset = color_offsets.count(color) ? color_offsets.at(color) : 0;
  dummy_bulb.id = id * 1000 + color_offset;

  dummy_bulb.color = color;
  dummy_bulb.shape = "circle";  // Default shape is circle

  // Calculate linestring vector
  const double dx = linestring.back().x() - linestring.front().x();
  const double dy = linestring.back().y() - linestring.front().y();
  const double dz = linestring.back().z() - linestring.front().z();
  const double linestring_length = std::sqrt(dx * dx + dy * dy);

  bool horizontal = [&]() -> bool {
    if (height.has_value()) {
      return linestring_length > height.value();
    }
    return linestring_length > 1.0;
  }();

  if (horizontal) {
    // Horizontal traffic light
    // Place bulbs at trisection points based on color
    double position_ratio = [color]() -> double {
      if (color == "red") {
        return 1.0 / 6.0;  // 1/3 point from left
      }
      if (color == "yellow") {
        return 0.5;  // Center point
      }
      if (color == "green") {
        return 5.0 / 6.0;  // 2/3 point from left
      }
      return 0.5;  // Center point for unknown colors
    }();

    double z_offset = height ? height.value() / 2.0 : 0.3;
    // Calculate position at trisection point
    dummy_bulb.position.x = linestring.front().x() + dx * position_ratio;
    dummy_bulb.position.y = linestring.front().y() + dy * position_ratio;
    dummy_bulb.position.z = linestring.front().z() + dz * position_ratio + z_offset;
  } else {
    // Vertical traffic light
    const double offset = height ? height.value() / 6.0 : 0.2;
    const auto center = getLinestringCenter();

    // Place green light at center, yellow and red lights above it
    dummy_bulb.position.x = center.x;
    dummy_bulb.position.y = center.y;

    if (color == "red") {
      dummy_bulb.position.z = center.z + offset * 5.0;  // Top position
    } else if (color == "yellow") {
      dummy_bulb.position.z = center.z + offset * 3.0;  // Middle position
    } else if (color == "green") {
      dummy_bulb.position.z = center.z + offset * 1.0;  // Bottom position (center)
    } else {
      dummy_bulb.position.z = center.z + offset * 3.0;  // Center position for unknown colors
    }
  }

  return dummy_bulb;
}

std::vector<TrafficLightBulbInfo> TrafficLightInfo::getBlightBulbs(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & current_elements) const
{
  using autoware_perception_msgs::msg::TrafficLightElement;

  static const std::map<uint8_t, std::string> color_map = {
    {TrafficLightElement::RED, "red"},
    {TrafficLightElement::AMBER, "yellow"},
    {TrafficLightElement::GREEN, "green"}};

  static const std::map<uint8_t, std::string> arrow_map = {
    {TrafficLightElement::LEFT_ARROW, "left"},
    {TrafficLightElement::RIGHT_ARROW, "right"},
    {TrafficLightElement::UP_ARROW, "up"},
    {TrafficLightElement::DOWN_ARROW, "down"},
    {TrafficLightElement::DOWN_LEFT_ARROW, "down_left"},
    {TrafficLightElement::DOWN_RIGHT_ARROW, "down_right"},
    {TrafficLightElement::UP_LEFT_ARROW, "up_left"},
    {TrafficLightElement::UP_RIGHT_ARROW, "up_right"}};

  std::vector<TrafficLightBulbInfo> current_bulbs;

  auto append_circle_color_bulb = [&current_bulbs, this](const std::string & color) {
    for (const auto & bulb : bulbs) {
      if (bulb.color == color && (bulb.shape == "none" || bulb.shape == "circle")) {
        current_bulbs.emplace_back(bulb);
        return;
      }
    }
    current_bulbs.emplace_back(getEstimatedBulb(color));
  };

  auto append_arrow_bulb = [&current_bulbs, this](const std::string & arrow) {
    for (const auto & bulb : bulbs) {
      if (bulb.shape == arrow) {
        current_bulbs.emplace_back(bulb);
      }
    }
  };

  for (const auto & element : current_elements) {
    if (element.shape == TrafficLightElement::CIRCLE) {
      if (auto it = color_map.find(element.color); it != color_map.end()) {
        append_circle_color_bulb(it->second);
      }
    } else if (auto it = arrow_map.find(element.shape); it != arrow_map.end()) {
      append_arrow_bulb(it->second);
    }
  }

  return current_bulbs;
}

Point3d TrafficLightInfo::getLinestringCenter() const
{
  Point3d center{
    (linestring.front().x() + linestring.back().x()) / 2,
    (linestring.front().y() + linestring.back().y()) / 2,
    (linestring.front().z() + linestring.back().z()) / 2};
  return center;
}

TrafficLightDisplay::TrafficLightDisplay() = default;

TrafficLightDisplay::~TrafficLightDisplay() = default;

void TrafficLightDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();

  auto rviz_ros_node = context_->getRosNodeAbstraction();

  lanelet_map_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Lanelet Map Topic", QString("/map/vector_map"), QString("autoware_map_msgs/msg/LaneletMapBin"),
    QString("Topic for lanelet map data"), this, SLOT(topic_updated_lanelet_map()));
  lanelet_map_topic_property_->initialize(rviz_ros_node);

  traffic_light_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Traffic Light Topic", QString("/perception/traffic_light_recognition/traffic_signals"),
    QString("autoware_perception_msgs/msg/TrafficLightGroupArray"),
    QString("Topic for traffic light data"), this, SLOT(topic_updated_traffic_light()));
  traffic_light_topic_property_->initialize(rviz_ros_node);

  timeout_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Timeout", 1.0, "Timeout in seconds for traffic light data. Set 0 to disable timeout.", this);
  timeout_property_->setMin(0.0);

  text_z_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Text Z Offset", 1.3, "Z offset in meters for traffic light text display", this);

  text_x_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Text X Offset", 0.0, "X offset in meters for traffic light text display", this);

  text_y_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Text Y Offset", 0.0, "Y offset in meters for traffic light text display", this);

  show_text_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Show Text", true, "Show/hide traffic light state text", this);

  show_bulb_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Show Bulb", true, "Show/hide traffic light bulb visualization", this);

  text_prefix_property_ = std::make_unique<rviz_common::properties::StringProperty>(
    "Text Prefix", "", "Prefix string to add before traffic light state text", this);

  font_size_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Font Size", 0.5, "Font size for traffic light state text", this);
  font_size_property_->setMin(0.1);

  text_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Text Color", QColor(255, 255, 255), "Color for traffic light state text", this);

  bulb_radius_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Bulb Radius", 0.4, "Radius in meters for traffic light bulb visualization", this);
  bulb_radius_property_->setMin(0.1);
}

void TrafficLightDisplay::setupRosSubscriptions()
{
  topic_updated_lanelet_map();
  topic_updated_traffic_light();
}

void TrafficLightDisplay::onEnable()
{
  rviz_common::Display::onEnable();
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  setupRosSubscriptions();
  for (const auto & [id, text_node] : traffic_light_text_nodes_) {
    text_node->setVisible(show_text_property_->getBool());
  }
  for (const auto & [id, bulb_display] : traffic_light_bulb_displays_) {
    bulb_display->getEntity()->setVisible(show_bulb_property_->getBool());
  }
}

void TrafficLightDisplay::onDisable()
{
  rviz_common::Display::onDisable();
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  lanelet_map_sub_.reset();
  traffic_light_group_array_sub_.reset();
  for (const auto & [id, text_node] : traffic_light_text_nodes_) {
    text_node->setVisible(false);
  }
  for (const auto & [id, bulb_display] : traffic_light_bulb_displays_) {
    bulb_display->getEntity()->setVisible(false);
  }
}

void TrafficLightDisplay::reset()
{
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  std::lock_guard<std::mutex> lock_lanelet(lanelet_map_mutex_);
  std::lock_guard<std::mutex> lock_traffic(traffic_light_mutex_);

  rviz_common::Display::reset();

  // Clear all display objects
  traffic_light_text_displays_.clear();
  traffic_light_text_nodes_.clear();
  traffic_light_bulb_displays_.clear();

  // Reset data
  lanelet_map_.reset();
  traffic_light_groups_.reset();
  last_traffic_light_received_time_ = rclcpp::Time();

  // Reset subscriptions
  lanelet_map_sub_.reset();
  traffic_light_group_array_sub_.reset();
}

void TrafficLightDisplay::hideAllDisplays()
{
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  for (const auto & [id, text_node] : traffic_light_text_nodes_) {
    text_node->setVisible(false);
  }
  for (const auto & [id, bulb_display] : traffic_light_bulb_displays_) {
    bulb_display->getEntity()->setVisible(false);
  }
}

bool TrafficLightDisplay::checkTimeout() const
{
  const float timeout = timeout_property_->getFloat();
  if (timeout > 0.0) {
    auto current_time = context_->getRosNodeAbstraction().lock()->get_raw_node()->now();
    if ((current_time - last_traffic_light_received_time_).seconds() > timeout) {
      return true;
    }
  }
  return false;
}

void TrafficLightDisplay::updateTrafficLightText(
  const TrafficLightInfo & info, const std::string & state_text)
{
  // Note: This method is called from update() which already holds property_mutex_
  if (traffic_light_text_displays_.find(info.id) == traffic_light_text_displays_.end()) {
    auto text_display = std::make_unique<rviz_rendering::MovableText>(
      state_text, "Liberation Sans", font_size_property_->getFloat());
    text_display->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
    const QColor & color = text_color_property_->getColor();
    text_display->setColor(
      Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), color.alphaF()));
    traffic_light_text_displays_[info.id] = std::move(text_display);
  }

  if (traffic_light_text_nodes_.find(info.id) == traffic_light_text_nodes_.end()) {
    traffic_light_text_nodes_[info.id] = scene_node_->createChildSceneNode();
    traffic_light_text_nodes_[info.id]->attachObject(traffic_light_text_displays_[info.id].get());
  }

  std::string display_text = text_prefix_property_->getStdString() + state_text;
  Ogre::Vector3 position(
    static_cast<float>(info.getLinestringCenter().x) + text_x_offset_property_->getFloat(),
    static_cast<float>(info.getLinestringCenter().y) + text_y_offset_property_->getFloat(),
    static_cast<float>(info.getLinestringCenter().z) + text_z_offset_property_->getFloat());
  traffic_light_text_nodes_[info.id]->setPosition(position);
  traffic_light_text_displays_[info.id]->setCaption(display_text);
  traffic_light_text_displays_[info.id]->setCharacterHeight(font_size_property_->getFloat());
  const QColor & color = text_color_property_->getColor();
  traffic_light_text_displays_[info.id]->setColor(
    Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), color.alphaF()));
  traffic_light_text_nodes_[info.id]->setVisible(show_text_property_->getBool());
}

void TrafficLightDisplay::updateTrafficLightBulbs(
  const TrafficLightInfo & info,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements)
{
  // Note: This method is called from update() which already holds property_mutex_
  auto current_bulbs = info.getBlightBulbs(elements);
  for (const auto & bulb : current_bulbs) {
    if (traffic_light_bulb_displays_.find(bulb.id) == traffic_light_bulb_displays_.end()) {
      auto bulb_display = std::make_unique<rviz_rendering::Shape>(
        rviz_rendering::Shape::Type::Sphere, scene_manager_, scene_node_);

      Ogre::ColourValue color;
      if (bulb.color == "red") {
        color = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);
      } else if (bulb.color == "yellow") {
        color = Ogre::ColourValue(1.0, 0.5, 0.0, 1.0);
      } else if (bulb.color == "green") {
        color = Ogre::ColourValue(0.0, 1.0, 0.0, 1.0);
      }
      bulb_display->setColor(color);

      bulb_display->setPosition(Ogre::Vector3(
        static_cast<float>(bulb.position.x), static_cast<float>(bulb.position.y),
        static_cast<float>(bulb.position.z)));
      const float radius = bulb_radius_property_->getFloat();
      bulb_display->setScale(Ogre::Vector3(radius, radius, radius));
      traffic_light_bulb_displays_[bulb.id] = std::move(bulb_display);
    }
    traffic_light_bulb_displays_[bulb.id]->getEntity()->setVisible(show_bulb_property_->getBool());
  }
}

void TrafficLightDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::lock_guard<std::mutex> lock_lanelet(lanelet_map_mutex_);
  std::lock_guard<std::mutex> lock_traffic(traffic_light_mutex_);

  if (!lanelet_map_ || !traffic_light_groups_) {
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!this->context_->getFrameManager()->getTransform(
        lanelet_map_header_, position, orientation)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("TrafficLightDisplay"), "Error transforming from frame '%s' to frame '%s'",
      lanelet_map_header_.frame_id.c_str(), qPrintable(this->fixed_frame_));
  }
  this->scene_node_->setPosition(position);
  this->scene_node_->setOrientation(orientation);

  if (checkTimeout()) {
    hideAllDisplays();
    return;
  }

  hideAllDisplays();

  for (const auto & traffic_light_group : traffic_light_groups_->traffic_light_groups) {
    auto tl_infos = getTrafficLightInfo(*lanelet_map_, traffic_light_group.traffic_light_group_id);

    std::stringstream ss;
    for (const auto & elem : traffic_light_group.elements) {
      ss << elementToString(elem) << " ";
    }

    std::lock_guard<std::mutex> lock_property(property_mutex_);
    for (const auto & info : tl_infos) {
      updateTrafficLightText(info, ss.str());
      updateTrafficLightBulbs(info, traffic_light_group.elements);
    }
  }
}

void TrafficLightDisplay::topic_updated_lanelet_map()
{
  lanelet_map_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  lanelet_map_sub_ =
    rviz_ros_node->get_raw_node()->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      lanelet_map_topic_property_->getTopicStd(), rclcpp::QoS(1).transient_local(),
      std::bind(&TrafficLightDisplay::onLaneletMapReceived, this, std::placeholders::_1));
}

void TrafficLightDisplay::topic_updated_traffic_light()
{
  traffic_light_group_array_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  traffic_light_group_array_sub_ =
    rviz_ros_node->get_raw_node()
      ->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
        traffic_light_topic_property_->getTopicStd(), rclcpp::QoS(10),
        std::bind(
          &TrafficLightDisplay::onTrafficLightGroupArrayReceived, this, std::placeholders::_1));
}

void TrafficLightDisplay::onLaneletMapReceived(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lanelet_map_mutex_);
  lanelet_map_header_ = msg->header;
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
}

void TrafficLightDisplay::onTrafficLightGroupArrayReceived(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(traffic_light_mutex_);
  traffic_light_groups_ = msg;
  last_traffic_light_received_time_ =
    context_->getRosNodeAbstraction().lock()->get_raw_node()->now();
}

}  // namespace autoware_perception_rviz_plugin::traffic_light

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware_perception_rviz_plugin::traffic_light::TrafficLightDisplay, rviz_common::Display)
