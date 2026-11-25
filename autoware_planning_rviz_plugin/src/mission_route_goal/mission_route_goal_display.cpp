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

#include "autoware_planning_rviz_plugin/mission_route_goal/mission_route_goal_display.hpp"

#include <Eigen/Core>
#include <OgreCamera.h>
#include <OgreVector3.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
lanelet::LaneletMapPtr decodeLaneletMapBin(const autoware_map_msgs::msg::LaneletMapBin & msg)
{
  std::string data_str(msg.data.begin(), msg.data.end());
  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive ia(ss);

  auto map_ptr = std::make_shared<lanelet::LaneletMap>();
  ia >> *map_ptr;
  lanelet::Id id_counter = 0;
  ia >> id_counter;
  lanelet::utils::registerId(id_counter);
  return map_ptr;
}

geometry_msgs::msg::Point makePt(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std_msgs::msg::ColorRGBA makeCol(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = std::clamp(r, 0.0F, 1.0F);
  c.g = std::clamp(g, 0.0F, 1.0F);
  c.b = std::clamp(b, 0.0F, 1.0F);
  c.a = std::clamp(a, 0.0F, 1.0F);
  return c;
}

void pushTri(
  visualization_msgs::msg::Marker & m, const geometry_msgs::msg::Point & p0,
  const std_msgs::msg::ColorRGBA & c0, const geometry_msgs::msg::Point & p1,
  const std_msgs::msg::ColorRGBA & c1, const geometry_msgs::msg::Point & p2,
  const std_msgs::msg::ColorRGBA & c2)
{
  m.points.push_back(p0);
  m.colors.push_back(c0);
  m.points.push_back(p1);
  m.colors.push_back(c1);
  m.points.push_back(p2);
  m.colors.push_back(c2);
}

void pushQuadCCW(
  visualization_msgs::msg::Marker & m, const geometry_msgs::msg::Point & a0,
  const std_msgs::msg::ColorRGBA & ca0, const geometry_msgs::msg::Point & b0,
  const std_msgs::msg::ColorRGBA & cb0, const geometry_msgs::msg::Point & b1,
  const std_msgs::msg::ColorRGBA & cb1, const geometry_msgs::msg::Point & a1,
  const std_msgs::msg::ColorRGBA & ca1)
{
  pushTri(m, a0, ca0, b0, cb0, b1, cb1);
  pushTri(m, a0, ca0, b1, cb1, a1, ca1);
}

void appendSmoothHologramRing(
  visualization_msgs::msg::Marker & m, const double cx, const double cy, const double cz,
  const double radius, const double width, const std_msgs::msg::ColorRGBA & core_col,
  const std_msgs::msg::ColorRGBA & edge_col, const int segments = 64)
{
  const double half_w = width * 0.5;
  const double r_in_fade = std::max(0.0, radius - half_w);
  const double r_in_core = std::max(0.0, radius - half_w * 0.35);
  const double r_out_core = radius + half_w * 0.35;
  const double r_out_fade = radius + half_w;

  for (int s = 0; s < segments; ++s) {
    const double a0 = (2.0 * M_PI * static_cast<double>(s)) / segments;
    const double a1 = (2.0 * M_PI * static_cast<double>(s + 1)) / segments;
    const double c0 = std::cos(a0);
    const double s0 = std::sin(a0);
    const double c1 = std::cos(a1);
    const double s1 = std::sin(a1);

    const auto p0_if = makePt(cx + c0 * r_in_fade, cy + s0 * r_in_fade, cz);
    const auto p1_if = makePt(cx + c1 * r_in_fade, cy + s1 * r_in_fade, cz);
    const auto p0_ic = makePt(cx + c0 * r_in_core, cy + s0 * r_in_core, cz);
    const auto p1_ic = makePt(cx + c1 * r_in_core, cy + s1 * r_in_core, cz);
    const auto p0_oc = makePt(cx + c0 * r_out_core, cy + s0 * r_out_core, cz);
    const auto p1_oc = makePt(cx + c1 * r_out_core, cy + s1 * r_out_core, cz);
    const auto p0_of = makePt(cx + c0 * r_out_fade, cy + s0 * r_out_fade, cz);
    const auto p1_of = makePt(cx + c1 * r_out_fade, cy + s1 * r_out_fade, cz);

    // CCW (+Z upward normal): inner -> outer -> next_outer -> next_inner
    pushQuadCCW(m, p0_if, edge_col, p0_ic, core_col, p1_ic, core_col, p1_if, edge_col);
    pushQuadCCW(m, p0_ic, core_col, p0_oc, core_col, p1_oc, core_col, p1_ic, core_col);
    pushQuadCCW(m, p0_oc, core_col, p0_of, edge_col, p1_of, edge_col, p1_oc, core_col);
  }
}

std::string formatRemainingDistance(const double dist_m)
{
  std::ostringstream oss;
  if (dist_m >= 1000.0) {
    oss << std::fixed << std::setprecision(1) << (dist_m / 1000.0) << " km";
  } else {
    oss << static_cast<int>(std::round(std::max(0.0, dist_m))) << " m";
  }
  return oss.str();
}
}  // namespace

namespace rviz_plugins
{

MissionRouteGoalDisplay::MissionRouteGoalDisplay()
: m_marker_common(this),
  m_show_route_property{
    "Route Line", false, "Render hologram navigation line along the mission route", this,
    SLOT(onStyleChanged())},
  m_route_color_property{
    "Route Color", QColor{0, 225, 165}, "Color of the navigation route guide line",
    &m_show_route_property, SLOT(onStyleChanged()), this},
  m_route_alpha_property{
    "Route Alpha", 0.65F, "Opacity of the navigation route guide line", &m_show_route_property,
    SLOT(onStyleChanged()), this},
  m_route_width_property{
    "Route Width", 0.20F, "Width [m] of the navigation route guide line", &m_show_route_property,
    SLOT(onStyleChanged()), this},
  m_trim_passed_property{
    "Trim Passed Route", true, "Automatically trim route segments behind ego base_link",
    &m_show_route_property, SLOT(onStyleChanged()), this},
  m_z_offset_property{
    "Z Offset", 0.068F, "Vertical offset [m] above road surface", this, SLOT(onStyleChanged())},
  m_show_goal_property{
    "Goal Beacon", true, "Render minimal hologram goal circle on the road surface", this,
    SLOT(onStyleChanged())},
  m_goal_color_property{
    "Goal Color", QColor{0, 235, 165}, "Color of the hologram goal circle",
    &m_show_goal_property, SLOT(onStyleChanged()), this},
  m_goal_alpha_property{
    "Goal Alpha", 0.90F, "Opacity of the hologram goal circle", &m_show_goal_property,
    SLOT(onStyleChanged()), this},
  m_goal_radius_property{
    "Ring Radius", 1.20F, "Outer radius [m] of the ground goal circle", &m_show_goal_property,
    SLOT(onStyleChanged()), this},
  m_beacon_height_property{
    "Beacon Height", 0.0F, "Optional vertical light pillar height [m] (0 = flat circle only)",
    &m_show_goal_property, SLOT(onStyleChanged()), this},
  m_show_distance_text_property{
    "Show Distance Text", false, "Show GOAL label and remaining distance above goal",
    &m_show_goal_property, SLOT(onStyleChanged()), this}
{
  m_show_route_property.setDisableChildrenIfFalse(true);
  m_show_goal_property.setDisableChildrenIfFalse(true);

  m_route_alpha_property.setMin(0.0F);
  m_route_alpha_property.setMax(1.0F);
  m_route_width_property.setMin(0.05F);
  m_route_width_property.setMax(2.0F);
  m_z_offset_property.setMin(-0.5F);
  m_z_offset_property.setMax(1.0F);

  m_goal_alpha_property.setMin(0.0F);
  m_goal_alpha_property.setMax(1.0F);
  m_goal_radius_property.setMin(0.2F);
  m_goal_radius_property.setMax(5.0F);
  m_beacon_height_property.setMin(0.0F);
  m_beacon_height_property.setMax(60.0F);
}

MissionRouteGoalDisplay::~MissionRouteGoalDisplay() = default;

void MissionRouteGoalDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  m_marker_common.initialize(context_, scene_node_);

  auto ros_node_abs = context_->getRosNodeAbstraction();

  m_route_topic_property = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Route Topic", "/planning/mission_planning/route", "autoware_planning_msgs/msg/LaneletRoute",
    "Mission planning LaneletRoute topic", this, SLOT(onRouteTopicChanged()));
  m_route_topic_property->initialize(ros_node_abs);

  m_map_topic_property = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Vector Map Topic", "/map/vector_map", "autoware_map_msgs/msg/LaneletMapBin",
    "Lanelet2 binary map topic (used only when Route Line is enabled)", this,
    SLOT(onMapTopicChanged()));
  m_map_topic_property->initialize(ros_node_abs);
}

void MissionRouteGoalDisplay::load(const rviz_common::Config & config)
{
  rviz_common::Display::load(config);
  m_marker_common.load(config);
}

void MissionRouteGoalDisplay::onEnable()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!m_route_sub) {
    setupRouteSubscription();
  }
  if (m_show_route_property.getBool() && !m_map_sub) {
    setupMapSubscription();
  }
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_marker_common.clearMarkers();
}

void MissionRouteGoalDisplay::fixedFrameChanged()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_marker_common.clearMarkers();
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::reset()
{
  rviz_common::Display::reset();
  std::lock_guard<std::mutex> lock(m_mutex);
  m_marker_common.clearMarkers();
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::onRouteTopicChanged()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_route_sub.reset();
  m_route_msg.reset();
  m_route_pts.clear();
  m_marker_common.clearMarkers();
  if (isEnabled()) {
    setupRouteSubscription();
  }
}

void MissionRouteGoalDisplay::onMapTopicChanged()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_map_sub.reset();
  m_lanelet_map.reset();
  m_route_pts.clear();
  if (isEnabled() && m_show_route_property.getBool()) {
    setupMapSubscription();
  }
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::onStyleChanged()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  if (isEnabled() && m_show_route_property.getBool() && !m_map_sub) {
    setupMapSubscription();
  }
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::setupRouteSubscription()
{
  const std::string topic = m_route_topic_property->getTopicStd();
  if (topic.empty()) {
    return;
  }
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_route_sub = node->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    topic, rclcpp::QoS(5).transient_local().reliable(),
    [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) { onRoute(msg); });
}

void MissionRouteGoalDisplay::setupMapSubscription()
{
  const std::string topic = m_map_topic_property->getTopicStd();
  if (topic.empty()) {
    return;
  }
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  m_map_sub = node->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    topic, rclcpp::QoS(5).transient_local().reliable(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) { onMapBin(msg); });
}

void MissionRouteGoalDisplay::onRoute(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_route_msg = msg;
  m_centerline_dirty = true;
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::onMapBin(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  lanelet::LaneletMapPtr decoded;
  try {
    decoded = decodeLaneletMapBin(*msg);
  } catch (const std::exception &) {
    return;
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_route_msg && !m_route_msg->segments.empty() && m_lanelet_map) {
    const auto first_id = m_route_msg->segments.front().preferred_primitive.id;
    if (
      m_lanelet_map->laneletLayer.exists(first_id) && !decoded->laneletLayer.exists(first_id)) {
      return;
    }
  }

  m_lanelet_map = std::move(decoded);
  m_centerline_dirty = true;
  m_visual_dirty = true;
}

void MissionRouteGoalDisplay::rebuildRouteCenterline()
{
  m_route_pts.clear();
  if (!m_route_msg || !m_lanelet_map || m_route_msg->segments.empty()) {
    return;
  }

  struct RawPt
  {
    double x;
    double y;
    double z;
  };

  std::vector<std::vector<RawPt>> seg_polylines;
  seg_polylines.reserve(m_route_msg->segments.size());

  for (const auto & seg : m_route_msg->segments) {
    const auto id = seg.preferred_primitive.id;
    if (!m_lanelet_map->laneletLayer.exists(id)) {
      continue;
    }
    const auto ll = m_lanelet_map->laneletLayer.get(id);
    const auto cl = ll.centerline();
    if (cl.size() < 2) {
      continue;
    }
    std::vector<RawPt> pts;
    pts.reserve(cl.size());
    for (const auto & p : cl) {
      if (pts.empty() || std::hypot(p.x() - pts.back().x, p.y() - pts.back().y) >= 0.05) {
        pts.push_back({p.x(), p.y(), p.z()});
      }
    }
    if (pts.size() >= 2) {
      seg_polylines.push_back(std::move(pts));
    }
  }

  if (seg_polylines.empty()) {
    return;
  }

  std::vector<RawPt> combined = seg_polylines.front();
  for (size_t idx = 1; idx < seg_polylines.size(); ++idx) {
    const auto & next_poly = seg_polylines[idx];
    for (const auto & pt : next_poly) {
      if (std::hypot(pt.x - combined.back().x, pt.y - combined.back().y) >= 0.08) {
        combined.push_back(pt);
      }
    }
  }

  if (combined.size() < 2) {
    return;
  }

  const auto & gp = m_route_msg->goal_pose.position;
  size_t best_seg = combined.size() - 2;
  double best_t = 1.0;
  double best_d2 = std::numeric_limits<double>::max();
  double tail_acc = 0.0;
  for (size_t i = combined.size() - 1; i > 0; --i) {
    const auto & a = combined[i - 1];
    const auto & b = combined[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double seg_l2 = dx * dx + dy * dy;
    double t = 0.0;
    if (seg_l2 > 1e-8) {
      t = std::clamp(((gp.x - a.x) * dx + (gp.y - a.y) * dy) / seg_l2, 0.0, 1.0);
    }
    const double px = a.x + dx * t;
    const double py = a.y + dy * t;
    const double d2 = (gp.x - px) * (gp.x - px) + (gp.y - py) * (gp.y - py);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_seg = i - 1;
      best_t = t;
    }
    tail_acc += std::sqrt(seg_l2);
    if (tail_acc > 120.0 && best_d2 < 25.0) {
      break;
    }
  }

  if (best_d2 < 36.0) {
    const auto & a = combined[best_seg];
    const auto & b = combined[best_seg + 1];
    RawPt proj{
      a.x + (b.x - a.x) * best_t, a.y + (b.y - a.y) * best_t, a.z + (b.z - a.z) * best_t};
    combined.resize(best_seg + 1);
    if (std::hypot(proj.x - combined.back().x, proj.y - combined.back().y) >= 0.05) {
      combined.push_back(proj);
    }
  }

  if (combined.size() < 2) {
    return;
  }

  m_route_pts.reserve(combined.size());
  double acc_s = 0.0;
  for (size_t i = 0; i < combined.size(); ++i) {
    if (i > 0) {
      acc_s += std::hypot(combined[i].x - combined[i - 1].x, combined[i].y - combined[i - 1].y);
    }
    double dx = 1.0;
    double dy = 0.0;
    if (i == 0) {
      dx = combined[1].x - combined[0].x;
      dy = combined[1].y - combined[0].y;
    } else if (i + 1 == combined.size()) {
      dx = combined[i].x - combined[i - 1].x;
      dy = combined[i].y - combined[i - 1].y;
    } else {
      dx = combined[i + 1].x - combined[i - 1].x;
      dy = combined[i + 1].y - combined[i - 1].y;
    }
    const double n = std::hypot(dx, dy);
    if (n > 1e-6) {
      dx /= n;
      dy /= n;
    }
    m_route_pts.push_back({combined[i].x, combined[i].y, combined[i].z, acc_s, dx, dy});
  }
}

void MissionRouteGoalDisplay::update(float wall_dt, float ros_dt)
{
  m_marker_common.update(wall_dt, ros_dt);
  if (!isEnabled()) {
    return;
  }

  const bool dynamic_needed =
    m_show_route_property.getBool() || m_show_distance_text_property.getBool();

  bool has_ego = false;
  double ego_x = 0.0;
  double ego_y = 0.0;
  if (dynamic_needed && context_ && context_->getFrameManager()) {
    Ogre::Vector3 pos;
    Ogre::Quaternion ori;
    if (context_->getFrameManager()->getTransform(
          "base_link", rclcpp::Time(0, 0, RCL_ROS_TIME), pos, ori)) {
      has_ego = true;
      ego_x = static_cast<double>(pos.x);
      ego_y = static_cast<double>(pos.y);
    }
  }

  bool has_cam = false;
  double cam_x = 0.0;
  double cam_y = 0.0;
  double cam_z = 0.0;
  if (
    m_show_distance_text_property.getBool() && context_ && context_->getViewManager() &&
    context_->getViewManager()->getCurrent()) {
    if (auto * cam = context_->getViewManager()->getCurrent()->getCamera()) {
      const Ogre::Vector3 cp = cam->getDerivedPosition();
      has_cam = true;
      cam_x = static_cast<double>(cp.x);
      cam_y = static_cast<double>(cp.y);
      cam_z = static_cast<double>(cp.z);
    }
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  if (!m_route_msg) {
    return;
  }

  if (m_show_route_property.getBool() && m_centerline_dirty) {
    rebuildRouteCenterline();
    m_centerline_dirty = false;
    m_visual_dirty = true;
  }

  bool need_update = m_visual_dirty;
  if (!need_update && dynamic_needed && has_ego) {
    if (std::hypot(ego_x - m_last_ego_x, ego_y - m_last_ego_y) >= 0.40) {
      need_update = true;
    }
  }

  if (need_update) {
    publishVisualMarkers(has_ego, ego_x, ego_y, has_cam, cam_x, cam_y, cam_z);
    m_visual_dirty = false;
    m_had_ego_last = has_ego;
    m_last_ego_x = ego_x;
    m_last_ego_y = ego_y;
  }
}

void MissionRouteGoalDisplay::publishVisualMarkers(
  bool has_ego, double ego_x, double ego_y, bool has_cam, double cam_x, double cam_y, double cam_z)
{
  m_marker_common.clearMarkers();
  if (!m_route_msg) {
    return;
  }

  auto arr = std::make_shared<visualization_msgs::msg::MarkerArray>();
  std_msgs::msg::Header hdr = m_route_msg->header;
  if (hdr.frame_id.empty()) {
    hdr.frame_id = "map";
  }
  hdr.stamp.sec = 0;
  hdr.stamp.nanosec = 0;

  const double z_off = static_cast<double>(m_z_offset_property.getFloat());
  const auto & gp = m_route_msg->goal_pose.position;

  double remaining_dist = -1.0;
  size_t start_idx = 0;
  double start_seg_t = 0.0;

  if (m_show_route_property.getBool() && !m_route_pts.empty()) {
    const double total_s = m_route_pts.back().s;
    if (has_ego) {
      double best_d2 = std::numeric_limits<double>::max();
      size_t best_i = 0;
      double best_t = 0.0;
      for (size_t i = 0; i + 1 < m_route_pts.size(); ++i) {
        const auto & a = m_route_pts[i];
        const auto & b = m_route_pts[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double l2 = dx * dx + dy * dy;
        double t = 0.0;
        if (l2 > 1e-8) {
          t = std::clamp(((ego_x - a.x) * dx + (ego_y - a.y) * dy) / l2, 0.0, 1.0);
        }
        const double px = a.x + dx * t;
        const double py = a.y + dy * t;
        const double d2 = (ego_x - px) * (ego_x - px) + (ego_y - py) * (ego_y - py);
        if (d2 < best_d2) {
          best_d2 = d2;
          best_i = i;
          best_t = t;
        }
      }
      if (best_d2 < 900.0) {
        const double ego_s =
          m_route_pts[best_i].s + (m_route_pts[best_i + 1].s - m_route_pts[best_i].s) * best_t;
        remaining_dist = std::max(0.0, total_s - ego_s);
        if (m_trim_passed_property.getBool()) {
          start_idx = best_i;
          start_seg_t = best_t;
        }
      }
    }
  } else if (has_ego) {
    remaining_dist = std::hypot(gp.x - ego_x, gp.y - ego_y);
  }

  // Optional Route Line (disabled by default)
  if (m_show_route_property.getBool() && m_route_pts.size() >= 2 && start_idx + 1 < m_route_pts.size()) {
    std::vector<RouteSamplePt> active;
    active.reserve(m_route_pts.size() - start_idx + 1);
    if (start_seg_t > 1e-3) {
      const auto & a = m_route_pts[start_idx];
      const auto & b = m_route_pts[start_idx + 1];
      active.push_back(
        {a.x + (b.x - a.x) * start_seg_t, a.y + (b.y - a.y) * start_seg_t,
         a.z + (b.z - a.z) * start_seg_t, a.s + (b.s - a.s) * start_seg_t, a.tx, a.ty});
      for (size_t i = start_idx + 1; i < m_route_pts.size(); ++i) {
        if (std::hypot(m_route_pts[i].x - active.back().x, m_route_pts[i].y - active.back().y) >= 0.05) {
          active.push_back(m_route_pts[i]);
        }
      }
    } else {
      active.assign(m_route_pts.begin() + static_cast<std::ptrdiff_t>(start_idx), m_route_pts.end());
    }

    if (active.size() >= 2) {
      const QColor qc = m_route_color_property.getColor();
      const float r_alpha = m_route_alpha_property.getFloat();
      const double half_w =
        std::max(0.025, static_cast<double>(m_route_width_property.getFloat()) * 0.5);
      const float cr = static_cast<float>(qc.redF());
      const float cg = static_cast<float>(qc.greenF());
      const float cb = static_cast<float>(qc.blueF());
      const auto col = makeCol(cr, cg, cb, r_alpha * 0.65F);

      visualization_msgs::msg::Marker route_m;
      route_m.header = hdr;
      route_m.ns = "route_line";
      route_m.id = 0;
      route_m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      route_m.action = visualization_msgs::msg::Marker::ADD;
      route_m.pose.orientation.w = 1.0;
      route_m.scale.x = 1.0;
      route_m.scale.y = 1.0;
      route_m.scale.z = 1.0;
      route_m.color = makeCol(1.0F, 1.0F, 1.0F, 1.0F);

      for (size_t i = 0; i + 1 < active.size(); ++i) {
        const auto & p0 = active[i];
        const auto & p1 = active[i + 1];
        const double nx0 = -p0.ty;
        const double ny0 = p0.tx;
        const double nx1 = -p1.ty;
        const double ny1 = p1.tx;
        pushQuadCCW(
          route_m, makePt(p0.x + nx0 * half_w, p0.y + ny0 * half_w, p0.z + z_off), col,
          makePt(p0.x - nx0 * half_w, p0.y - ny0 * half_w, p0.z + z_off), col,
          makePt(p1.x - nx1 * half_w, p1.y - ny1 * half_w, p1.z + z_off), col,
          makePt(p1.x + nx1 * half_w, p1.y + ny1 * half_w, p1.z + z_off), col);
      }
      arr->markers.push_back(std::move(route_m));
    }
  }

  // Minimal Hologram Goal Circle on the road surface
  if (m_show_goal_property.getBool()) {
    const QColor gqc = m_goal_color_property.getColor();
    const float g_alpha = m_goal_alpha_property.getFloat();
    const double outer_r = static_cast<double>(m_goal_radius_property.getFloat());
    const double inner_r = outer_r * 0.44;
    const double pillar_h = static_cast<double>(m_beacon_height_property.getFloat());

    const float gr = static_cast<float>(gqc.redF());
    const float gg = static_cast<float>(gqc.greenF());
    const float gb = static_cast<float>(gqc.blueF());

    const float br = std::clamp(gr * 0.48F + 0.52F, 0.0F, 1.0F);
    const float bg = std::clamp(gg * 0.48F + 0.52F, 0.0F, 1.0F);
    const float bb = std::clamp(gb * 0.48F + 0.52F, 0.0F, 1.0F);

    const double gx = gp.x;
    const double gy = gp.y;
    const double gz = gp.z + z_off;

    visualization_msgs::msg::Marker goal_tris;
    goal_tris.header = hdr;
    goal_tris.ns = "goal_circle";
    goal_tris.id = 1;
    goal_tris.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    goal_tris.action = visualization_msgs::msg::Marker::ADD;
    goal_tris.pose.orientation.w = 1.0;
    goal_tris.scale.x = 1.0;
    goal_tris.scale.y = 1.0;
    goal_tris.scale.z = 1.0;
    goal_tris.color = makeCol(1.0F, 1.0F, 1.0F, 1.0F);

    // Subtle translucent radial fill inside the circle
    constexpr int kSegments = 64;
    const auto center_pt = makePt(gx, gy, gz);
    const auto c_center = makeCol(gr, gg, gb, g_alpha * 0.24F);
    const auto c_mid = makeCol(gr, gg, gb, g_alpha * 0.12F);
    const auto c_rim = makeCol(gr, gg, gb, g_alpha * 0.22F);

    for (int s = 0; s < kSegments; ++s) {
      const double a0 = (2.0 * M_PI * static_cast<double>(s)) / kSegments;
      const double a1 = (2.0 * M_PI * static_cast<double>(s + 1)) / kSegments;
      const double c0 = std::cos(a0);
      const double s0 = std::sin(a0);
      const double c1 = std::cos(a1);
      const double s1 = std::sin(a1);

      const auto in0 = makePt(gx + c0 * inner_r, gy + s0 * inner_r, gz);
      const auto in1 = makePt(gx + c1 * inner_r, gy + s1 * inner_r, gz);
      const auto out0 = makePt(gx + c0 * outer_r, gy + s0 * outer_r, gz);
      const auto out1 = makePt(gx + c1 * outer_r, gy + s1 * outer_r, gz);

      pushTri(goal_tris, center_pt, c_center, in0, c_mid, in1, c_mid);
      pushTri(goal_tris, in0, c_mid, out0, c_rim, out1, c_rim);
      pushTri(goal_tris, in0, c_mid, out1, c_rim, in1, c_mid);
    }

    // Smooth anti-aliased outer & inner neon rings (raised +2mm to stay crisp above disc fill)
    const double ring_w_outer = std::clamp(outer_r * 0.085, 0.055, 0.14);
    const double ring_w_inner = std::clamp(outer_r * 0.050, 0.035, 0.08);
    appendSmoothHologramRing(
      goal_tris, gx, gy, gz + 0.002, outer_r, ring_w_outer,
      makeCol(br, bg, bb, std::min(0.98F, g_alpha * 1.02F)), makeCol(gr, gg, gb, 0.0F), kSegments);
    appendSmoothHologramRing(
      goal_tris, gx, gy, gz + 0.002, inner_r, ring_w_inner,
      makeCol(br, bg, bb, g_alpha * 0.68F), makeCol(gr, gg, gb, 0.0F), kSegments);

    // Optional light pillar (disabled by default: Beacon Height = 0.0)
    if (pillar_h > 0.2) {
      constexpr int kSides = 16;
      constexpr int kLevels = 6;
      const double r_base = inner_r * 0.45;
      for (int lv = 0; lv < kLevels; ++lv) {
        const double v0 = static_cast<double>(lv) / kLevels;
        const double v1 = static_cast<double>(lv + 1) / kLevels;
        const double z0 = gz + pillar_h * v0;
        const double z1 = gz + pillar_h * v1;
        const float pa0 = g_alpha * 0.30F * static_cast<float>(std::pow(1.0 - v0, 1.8));
        const float pa1 = g_alpha * 0.30F * static_cast<float>(std::pow(1.0 - v1, 1.8));
        const auto pc0 = makeCol(gr, gg, gb, pa0);
        const auto pc1 = makeCol(gr, gg, gb, pa1);
        for (int s = 0; s < kSides; ++s) {
          const double a0 = (2.0 * M_PI * static_cast<double>(s)) / kSides;
          const double a1 = (2.0 * M_PI * static_cast<double>(s + 1)) / kSides;
          const auto b0 = makePt(gx + std::cos(a0) * r_base, gy + std::sin(a0) * r_base, z0);
          const auto b1 = makePt(gx + std::cos(a1) * r_base, gy + std::sin(a1) * r_base, z0);
          const auto t0 = makePt(gx + std::cos(a0) * r_base, gy + std::sin(a0) * r_base, z1);
          const auto t1 = makePt(gx + std::cos(a1) * r_base, gy + std::sin(a1) * r_base, z1);
          pushTri(goal_tris, b0, pc0, b1, pc0, t1, pc1);
          pushTri(goal_tris, b0, pc0, t1, pc1, t0, pc1);
        }
      }
    }

    arr->markers.push_back(std::move(goal_tris));

    if (m_show_distance_text_property.getBool()) {
      double view_dist = 35.0;
      if (has_cam) {
        view_dist = std::sqrt(
          (cam_x - gx) * (cam_x - gx) + (cam_y - gy) * (cam_y - gy) + (cam_z - gz) * (cam_z - gz));
      } else if (remaining_dist >= 0.0) {
        view_dist = remaining_dist;
      }
      const double text_h = std::clamp(view_dist * 0.018, 0.70, 3.50);
      visualization_msgs::msg::Marker text_m;
      text_m.header = hdr;
      text_m.ns = "goal_beacon_text";
      text_m.id = 3;
      text_m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_m.action = visualization_msgs::msg::Marker::ADD;
      text_m.pose.position = makePt(gx, gy, gz + std::max(1.5, pillar_h) + text_h);
      text_m.pose.orientation.w = 1.0;
      text_m.scale.z = text_h;
      text_m.color = makeCol(br, bg, bb, 0.92F);
      text_m.text =
        (remaining_dist >= 0.0) ? ("GOAL\n" + formatRemainingDistance(remaining_dist)) : "GOAL";
      arr->markers.push_back(std::move(text_m));
    }
  }

  m_marker_common.addMessage(arr);
}

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::MissionRouteGoalDisplay, rviz_common::Display)
