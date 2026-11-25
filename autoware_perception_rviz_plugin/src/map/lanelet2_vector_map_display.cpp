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

#include "autoware_perception_rviz_plugin/map/lanelet2_vector_map_display.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace map
{

namespace
{
std_msgs::msg::ColorRGBA toColorRGBA(const QColor & qcolor, const float alpha)
{
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(qcolor.redF());
  c.g = static_cast<float>(qcolor.greenF());
  c.b = static_cast<float>(qcolor.blueF());
  c.a = alpha;
  return c;
}

void applyMarkerColor(
  visualization_msgs::msg::Marker & marker, const std_msgs::msg::ColorRGBA & color)
{
  marker.color = color;
  if (marker.type == visualization_msgs::msg::Marker::TRIANGLE_LIST && !marker.points.empty()) {
    // RViz2's TriangleListMarker disables 3D normal lighting ONLY when
    // colors.size() == points.size() (hasVertexColors() == true).
    // Clearing colors enables lighting on flat horizontal ribbons, turning them pitch black!
    marker.colors.assign(marker.points.size(), color);
  } else {
    marker.colors.clear();
  }
}

bool isSamePoint(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b, const double eps = 1e-3)
{
  return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps && std::abs(a.z - b.z) < eps;
}

void applyZOffset(visualization_msgs::msg::Marker & marker, const double z_offset)
{
  if (std::abs(z_offset) < 1e-6) {
    return;
  }
  if (!marker.points.empty()) {
    for (auto & p : marker.points) {
      p.z += z_offset;
    }
  } else {
    marker.pose.position.z += z_offset;
  }
}

void resizeRibbonAndApplyZOffset(
  visualization_msgs::msg::Marker & marker, const double target_width, const double z_offset)
{
  if (
    marker.type == visualization_msgs::msg::Marker::LINE_STRIP ||
    marker.type == visualization_msgs::msg::Marker::LINE_LIST) {
    marker.scale.x = target_width;
    applyZOffset(marker, z_offset);
    return;
  }

  if (
    marker.type != visualization_msgs::msg::Marker::TRIANGLE_LIST || marker.points.size() < 6 ||
    (marker.points.size() % 6) != 0) {
    applyZOffset(marker, z_offset);
    return;
  }

  const double target_half_w = target_width * 0.5;
  auto & pts = marker.points;
  for (size_t i = 0; i + 5 < pts.size(); i += 6) {
    if (!isSamePoint(pts[i + 1], pts[i + 5]) || !isSamePoint(pts[i + 2], pts[i + 4])) {
      for (size_t k = 0; k < 6; ++k) {
        pts[i + k].z += z_offset;
      }
      continue;
    }

    const double sx = 0.5 * (pts[i + 0].x + pts[i + 1].x);
    const double sy = 0.5 * (pts[i + 0].y + pts[i + 1].y);
    const double sz = 0.5 * (pts[i + 0].z + pts[i + 1].z) + z_offset;

    const double ex = 0.5 * (pts[i + 2].x + pts[i + 3].x);
    const double ey = 0.5 * (pts[i + 2].y + pts[i + 3].y);
    const double ez = 0.5 * (pts[i + 2].z + pts[i + 3].z) + z_offset;

    const double ox = 0.5 * (pts[i + 0].x - pts[i + 1].x);
    const double oy = 0.5 * (pts[i + 0].y - pts[i + 1].y);
    const double orig_half_w = std::hypot(ox, oy);

    double nx = ox;
    double ny = oy;
    if (orig_half_w > 1e-6) {
      const double scale = target_half_w / orig_half_w;
      nx = ox * scale;
      ny = oy * scale;
    }

    pts[i + 0].x = sx + nx;
    pts[i + 0].y = sy + ny;
    pts[i + 0].z = sz;

    pts[i + 1].x = sx - nx;
    pts[i + 1].y = sy - ny;
    pts[i + 1].z = sz;

    pts[i + 2].x = ex + nx;
    pts[i + 2].y = ey + ny;
    pts[i + 2].z = ez;

    pts[i + 3].x = ex - nx;
    pts[i + 3].y = ey - ny;
    pts[i + 3].z = ez;

    pts[i + 4] = pts[i + 2];
    pts[i + 5] = pts[i + 1];
  }
}

struct BorderSegment2D
{
  double sx;
  double sy;
  double ex;
  double ey;
};

int64_t packGridCell(const int32_t ix, const int32_t iy)
{
  return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
}

using BorderSpatialGrid = std::unordered_map<int64_t, std::vector<BorderSegment2D>>;

void collectRibbonSegmentsToGrid(
  const visualization_msgs::msg::Marker & marker, BorderSpatialGrid & grid)
{
  if (
    marker.type != visualization_msgs::msg::Marker::TRIANGLE_LIST || marker.points.size() < 6 ||
    (marker.points.size() % 6) != 0) {
    return;
  }
  const auto & pts = marker.points;
  for (size_t i = 0; i + 5 < pts.size(); i += 6) {
    if (!isSamePoint(pts[i + 1], pts[i + 5]) || !isSamePoint(pts[i + 2], pts[i + 4])) {
      continue;
    }
    const double sx = 0.5 * (pts[i + 0].x + pts[i + 1].x);
    const double sy = 0.5 * (pts[i + 0].y + pts[i + 1].y);
    const double ex = 0.5 * (pts[i + 2].x + pts[i + 3].x);
    const double ey = 0.5 * (pts[i + 2].y + pts[i + 3].y);
    const int32_t ix = static_cast<int32_t>(std::floor(0.5 * (sx + ex)));
    const int32_t iy = static_cast<int32_t>(std::floor(0.5 * (sy + ey)));
    grid[packGridCell(ix, iy)].push_back({sx, sy, ex, ey});
  }
}

void stripOverlappingBorderSegments(
  visualization_msgs::msg::Marker & marker, const BorderSpatialGrid & border_grid)
{
  if (
    border_grid.empty() || marker.type != visualization_msgs::msg::Marker::TRIANGLE_LIST ||
    marker.points.size() < 6 || (marker.points.size() % 6) != 0) {
    return;
  }

  const auto & pts = marker.points;
  const bool has_colors = (marker.colors.size() == pts.size());

  std::vector<geometry_msgs::msg::Point> filtered_pts;
  std::vector<std_msgs::msg::ColorRGBA> filtered_cols;
  filtered_pts.reserve(pts.size());
  if (has_colors) {
    filtered_cols.reserve(pts.size());
  }

  constexpr double kMatchTol = 0.12;
  constexpr double kMatchTolSq = kMatchTol * kMatchTol;

  for (size_t i = 0; i + 5 < pts.size(); i += 6) {
    bool is_dup = false;
    if (isSamePoint(pts[i + 1], pts[i + 5]) && isSamePoint(pts[i + 2], pts[i + 4])) {
      const double sx = 0.5 * (pts[i + 0].x + pts[i + 1].x);
      const double sy = 0.5 * (pts[i + 0].y + pts[i + 1].y);
      const double ex = 0.5 * (pts[i + 2].x + pts[i + 3].x);
      const double ey = 0.5 * (pts[i + 2].y + pts[i + 3].y);
      const int32_t cx = static_cast<int32_t>(std::floor(0.5 * (sx + ex)));
      const int32_t cy = static_cast<int32_t>(std::floor(0.5 * (sy + ey)));

      for (int32_t dx = -1; dx <= 1 && !is_dup; ++dx) {
        for (int32_t dy = -1; dy <= 1 && !is_dup; ++dy) {
          const auto it = border_grid.find(packGridCell(cx + dx, cy + dy));
          if (it == border_grid.end()) {
            continue;
          }
          for (const auto & b : it->second) {
            const double d_fwd_s = (sx - b.sx) * (sx - b.sx) + (sy - b.sy) * (sy - b.sy);
            const double d_fwd_e = (ex - b.ex) * (ex - b.ex) + (ey - b.ey) * (ey - b.ey);
            if (d_fwd_s < kMatchTolSq && d_fwd_e < kMatchTolSq) {
              is_dup = true;
              break;
            }
            const double d_rev_s = (sx - b.ex) * (sx - b.ex) + (sy - b.ey) * (sy - b.ey);
            const double d_rev_e = (ex - b.sx) * (ex - b.sx) + (ey - b.sy) * (ey - b.sy);
            if (d_rev_s < kMatchTolSq && d_rev_e < kMatchTolSq) {
              is_dup = true;
              break;
            }
          }
        }
      }
    }

    if (!is_dup) {
      for (size_t k = 0; k < 6; ++k) {
        filtered_pts.push_back(pts[i + k]);
        if (has_colors) {
          filtered_cols.push_back(marker.colors[i + k]);
        }
      }
    }
  }

  marker.points = std::move(filtered_pts);
  if (has_colors) {
    marker.colors = std::move(filtered_cols);
  }
}

bool isClutterArrowOrIdNamespace(const std::string & ns)
{
  return ns.find("arrows") != std::string::npos ||
         ns.find("lanelet direction") != std::string::npos ||
         ns.find("lane_start_bound") != std::string::npos ||
         ns.find("lanelet_id") != std::string::npos || ns == "traffic_light_id" ||
         ns == "traffic_light_reg_elem_id";
}

bool isSurfaceFillNamespace(const std::string & ns)
{
  return ns == "road_lanelets" || ns == "shoulder_road_lanelets" || ns == "walkway_lanelets" ||
         ns == "bicycle_lane_lanelets";
}
}  // namespace

Lanelet2VectorMapDisplay::Lanelet2VectorMapDisplay()
: m_marker_common(this),
  m_z_offset_property{
    "Z Offset", 0.05F, "Vertical offset [m] applied to lane/border lines above ground pointclouds",
    this, SLOT(onStyleChanged())},
  m_hide_arrows_and_ids_property{
    "Hide Arrows & IDs", true,
    "Suppress direction arrows, lane start bounds, and ID text across road/shoulder/bicycle lanes",
    this, SLOT(onStyleChanged())},
  m_road_border_override_property{
    "Road Borders", true, "Override style of road_borders / curbstone", this,
    SLOT(onStyleChanged())},
  m_road_border_color_property{
    "Color",
    QColor{235, 65, 95},
    "Color for road_borders",
    &m_road_border_override_property,
    SLOT(onStyleChanged()),
    this},
  m_road_border_alpha_property{
    "Alpha",
    0.90F,
    "Opacity for road_borders",
    &m_road_border_override_property,
    SLOT(onStyleChanged()),
    this},
  m_road_border_width_property{
    "Width",
    0.085F,
    "Ribbon width [m] for road_borders (default map node is 0.20m)",
    &m_road_border_override_property,
    SLOT(onStyleChanged()),
    this},
  m_lane_bound_override_property{
    "Lane Bounds", true, "Override style of left/right lane boundaries", this,
    SLOT(onStyleChanged())},
  m_lane_bound_color_property{
    "Color",
    QColor{225, 235, 250},
    "Color for lane boundary lines",
    &m_lane_bound_override_property,
    SLOT(onStyleChanged()),
    this},
  m_lane_bound_alpha_property{
    "Alpha",
    0.94F,
    "Opacity for lane boundary lines",
    &m_lane_bound_override_property,
    SLOT(onStyleChanged()),
    this},
  m_lane_bound_width_property{
    "Width",
    0.08F,
    "Ribbon width [m] for lane boundaries (default map node is 0.10m)",
    &m_lane_bound_override_property,
    SLOT(onStyleChanged()),
    this},
  m_center_line_override_property{
    "Center Line", true, "Override style of center_lane_line", this, SLOT(onStyleChanged())},
  m_center_line_color_property{
    "Color",
    QColor{110, 145, 180},
    "Color for center_lane_line",
    &m_center_line_override_property,
    SLOT(onStyleChanged()),
    this},
  m_center_line_alpha_property{
    "Alpha",
    0.42F,
    "Opacity for center_lane_line",
    &m_center_line_override_property,
    SLOT(onStyleChanged()),
    this},
  m_center_line_width_property{
    "Width",
    0.025F,
    "Ribbon width [m] for center_lane_line",
    &m_center_line_override_property,
    SLOT(onStyleChanged()),
    this},
  m_stopline_override_property{
    "Stop Lines & Markings", true, "Override style of stop_lines and road markings", this,
    SLOT(onStyleChanged())},
  m_stopline_color_property{
    "Color",
    QColor{230, 238, 250},
    "Color for stop_lines and pedestrian markings",
    &m_stopline_override_property,
    SLOT(onStyleChanged()),
    this},
  m_stopline_alpha_property{
    "Alpha",
    0.88F,
    "Opacity for stop_lines and pedestrian markings",
    &m_stopline_override_property,
    SLOT(onStyleChanged()),
    this},
  m_stopline_width_property{
    "Stop Line Width",
    0.30F,
    "Ribbon width [m] for stop_lines (default map node is 0.50m)",
    &m_stopline_override_property,
    SLOT(onStyleChanged()),
    this},
  m_road_fill_override_property{
    "Road Surface Fill", true,
    "Override color/alpha of road_lanelets & shoulder_road_lanelets when enabled", this,
    SLOT(onStyleChanged())},
  m_road_fill_color_property{
    "Color",
    QColor{26, 30, 40},
    "Color for road_lanelets polygon fill",
    &m_road_fill_override_property,
    SLOT(onStyleChanged()),
    this},
  m_road_fill_alpha_property{
    "Alpha",
    0.22F,
    "Opacity for road_lanelets polygon fill",
    &m_road_fill_override_property,
    SLOT(onStyleChanged()),
    this},
  m_bicycle_lane_override_property{
    "Bicycle Lane Fill", true,
    "Override color/alpha of bicycle_lane_lanelets so it stays distinct from cyan predicted paths",
    this, SLOT(onStyleChanged())},
  m_bicycle_lane_color_property{
    "Color",
    QColor{25, 70, 165},
    "Deep royal-blue pavement tint for bicycle_lane_lanelets",
    &m_bicycle_lane_override_property,
    SLOT(onStyleChanged()),
    this},
  m_bicycle_lane_alpha_property{
    "Alpha",
    0.16F,
    "Linear opacity for bicycle_lane_lanelets polygon fill",
    &m_bicycle_lane_override_property,
    SLOT(onStyleChanged()),
    this},
  m_traffic_light_override_property{
    "Traffic Lights", true,
    "Render hologram housing plate, neon bezel, and smoked lenses for traffic lights", this,
    SLOT(onStyleChanged())},
  m_traffic_light_plate_color_property{
    "Plate Color",
    QColor{16, 26, 40},
    "Color for traffic_light_triangle housing plate",
    &m_traffic_light_override_property,
    SLOT(onStyleChanged()),
    this},
  m_traffic_light_plate_alpha_property{
    "Plate Alpha",
    0.62F,
    "Opacity for traffic_light_triangle housing plate",
    &m_traffic_light_override_property,
    SLOT(onStyleChanged()),
    this},
  m_traffic_light_border_color_property{
    "Border Color",
    QColor{90, 205, 250},
    "Color for traffic light hologram bezel outline and lens rings",
    &m_traffic_light_override_property,
    SLOT(onStyleChanged()),
    this},
  m_traffic_light_border_alpha_property{
    "Border Alpha",
    0.92F,
    "Opacity for traffic light hologram bezel outline",
    &m_traffic_light_override_property,
    SLOT(onStyleChanged()),
    this},
  m_traffic_light_border_width_property{
    "Border Width",
    0.028F,
    "Line width [m] for traffic light hologram bezel outline",
    &m_traffic_light_override_property,
    SLOT(onStyleChanged()),
    this},
  m_traffic_light_bulb_alpha_property{
    "Unlit Bulb Alpha",
    0.45F,
    "Opacity for unlit map traffic_light lens spheres",
    &m_traffic_light_override_property,
    SLOT(onStyleChanged()),
    this}
{
  m_road_border_override_property.setDisableChildrenIfFalse(true);
  m_lane_bound_override_property.setDisableChildrenIfFalse(true);
  m_center_line_override_property.setDisableChildrenIfFalse(true);
  m_stopline_override_property.setDisableChildrenIfFalse(true);
  m_road_fill_override_property.setDisableChildrenIfFalse(true);
  m_bicycle_lane_override_property.setDisableChildrenIfFalse(true);
  m_traffic_light_override_property.setDisableChildrenIfFalse(true);

  m_road_border_alpha_property.setMin(0.0F);
  m_road_border_alpha_property.setMax(1.0F);
  m_road_border_width_property.setMin(0.005F);
  m_road_border_width_property.setMax(2.0F);

  m_lane_bound_alpha_property.setMin(0.0F);
  m_lane_bound_alpha_property.setMax(1.0F);
  m_lane_bound_width_property.setMin(0.005F);
  m_lane_bound_width_property.setMax(2.0F);

  m_center_line_alpha_property.setMin(0.0F);
  m_center_line_alpha_property.setMax(1.0F);
  m_center_line_width_property.setMin(0.002F);
  m_center_line_width_property.setMax(1.0F);

  m_stopline_alpha_property.setMin(0.0F);
  m_stopline_alpha_property.setMax(1.0F);
  m_stopline_width_property.setMin(0.01F);
  m_stopline_width_property.setMax(2.0F);

  m_road_fill_alpha_property.setMin(0.0F);
  m_road_fill_alpha_property.setMax(1.0F);

  m_bicycle_lane_alpha_property.setMin(0.0F);
  m_bicycle_lane_alpha_property.setMax(1.0F);

  m_traffic_light_plate_alpha_property.setMin(0.0F);
  m_traffic_light_plate_alpha_property.setMax(1.0F);
  m_traffic_light_border_alpha_property.setMin(0.0F);
  m_traffic_light_border_alpha_property.setMax(1.0F);
  m_traffic_light_border_width_property.setMin(0.005F);
  m_traffic_light_border_width_property.setMax(0.5F);
  m_traffic_light_bulb_alpha_property.setMin(0.0F);
  m_traffic_light_bulb_alpha_property.setMax(1.0F);
}

void Lanelet2VectorMapDisplay::onInitialize()
{
  this->qos_profile = rclcpp::QoS(5).transient_local().reliable();
  RosTopicDisplay::RTDClass::onInitialize();
  m_marker_common.initialize(this->context_, this->scene_node_);
  this->topic_property_->setMessageType(
    QString::fromStdString(rosidl_generator_traits::name<MarkerArray>()));
  this->topic_property_->setValue("/map/vector_map_marker");
  this->topic_property_->setDescription("Lanelet2 vector map MarkerArray topic.");
}

void Lanelet2VectorMapDisplay::load(const rviz_common::Config & config)
{
  RosTopicDisplay::Display::load(config);
  m_marker_common.load(config);
}

void Lanelet2VectorMapDisplay::update(float wall_dt, float ros_dt)
{
  m_marker_common.update(wall_dt, ros_dt);
}

void Lanelet2VectorMapDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common.clearMarkers();
  if (m_last_msg && this->isEnabled()) {
    applyStylesAndPublish();
  }
}

void Lanelet2VectorMapDisplay::fixedFrameChanged()
{
  m_marker_common.clearMarkers();
  if (m_last_msg && this->isEnabled()) {
    applyStylesAndPublish();
  }
}

void Lanelet2VectorMapDisplay::unsubscribe()
{
  m_last_msg.reset();
  RosTopicDisplay::unsubscribe();
}

void Lanelet2VectorMapDisplay::onStyleChanged()
{
  if (m_last_msg) {
    applyStylesAndPublish();
  }
}

void Lanelet2VectorMapDisplay::processMessage(MarkerArray::ConstSharedPtr msg)
{
  m_last_msg = msg;
  applyStylesAndPublish();
}

void Lanelet2VectorMapDisplay::applyStylesAndPublish()
{
  if (!m_last_msg) {
    return;
  }

  m_marker_common.clearMarkers();

  const double z_off = static_cast<double>(m_z_offset_property.getFloat());
  const bool hide_clutter = m_hide_arrows_and_ids_property.getBool();

  const bool override_rb = m_road_border_override_property.getBool();
  const auto rb_color =
    toColorRGBA(m_road_border_color_property.getColor(), m_road_border_alpha_property.getFloat());
  const double rb_width = static_cast<double>(m_road_border_width_property.getFloat());

  const bool override_lb = m_lane_bound_override_property.getBool();
  const auto lb_color =
    toColorRGBA(m_lane_bound_color_property.getColor(), m_lane_bound_alpha_property.getFloat());
  const double lb_width = static_cast<double>(m_lane_bound_width_property.getFloat());

  const bool override_cl = m_center_line_override_property.getBool();
  const auto cl_color =
    toColorRGBA(m_center_line_color_property.getColor(), m_center_line_alpha_property.getFloat());
  const double cl_width = static_cast<double>(m_center_line_width_property.getFloat());

  const bool override_sl = m_stopline_override_property.getBool();
  const auto sl_color =
    toColorRGBA(m_stopline_color_property.getColor(), m_stopline_alpha_property.getFloat());
  const double sl_width = static_cast<double>(m_stopline_width_property.getFloat());

  const bool override_rf = m_road_fill_override_property.getBool();
  const auto rf_color =
    toColorRGBA(m_road_fill_color_property.getColor(), m_road_fill_alpha_property.getFloat());

  const bool override_bl = m_bicycle_lane_override_property.getBool();
  const auto bl_color = toColorRGBA(
    m_bicycle_lane_color_property.getColor(), m_bicycle_lane_alpha_property.getFloat());

  const bool override_tl = m_traffic_light_override_property.getBool();
  const auto tl_plate_color = toColorRGBA(
    m_traffic_light_plate_color_property.getColor(),
    m_traffic_light_plate_alpha_property.getFloat());
  const auto tl_border_color = toColorRGBA(
    m_traffic_light_border_color_property.getColor(),
    m_traffic_light_border_alpha_property.getFloat());
  const double tl_border_width =
    static_cast<double>(m_traffic_light_border_width_property.getFloat());
  const float tl_bulb_alpha = m_traffic_light_bulb_alpha_property.getFloat();

  struct TrafficLightQuad
  {
    geometry_msgs::msg::Point v0;
    geometry_msgs::msg::Point v1;
    geometry_msgs::msg::Point v2;
    geometry_msgs::msg::Point v3;
    double cx{0.0};
    double cy{0.0};
    double cz{0.0};
    double ux{1.0};
    double uy{0.0};
    double uz{0.0};
    double wx{0.0};
    double wy{0.0};
    double wz{1.0};
  };

  std::vector<TrafficLightQuad> tl_quads;
  std_msgs::msg::Header tl_header;
  bool has_tl_header = false;

  if (override_tl) {
    for (const auto & orig : m_last_msg->markers) {
      if (orig.ns != "traffic_light_triangle" || orig.type != Marker::TRIANGLE_LIST) {
        continue;
      }
      if (!has_tl_header) {
        tl_header = orig.header;
        has_tl_header = true;
      }
      const auto & pts = orig.points;
      for (size_t i = 0; i + 5 < pts.size(); i += 6) {
        if (!isSamePoint(pts[i + 0], pts[i + 3]) || !isSamePoint(pts[i + 2], pts[i + 4])) {
          continue;
        }
        const auto & v0 = pts[i + 0];
        const auto & v1 = pts[i + 1];
        const auto & v2 = pts[i + 2];
        const auto & v3 = pts[i + 5];
        const double cx = 0.25 * (v0.x + v1.x + v2.x + v3.x);
        const double cy = 0.25 * (v0.y + v1.y + v2.y + v3.y);
        const double cz = 0.25 * (v0.z + v1.z + v2.z + v3.z);

        bool duplicate = false;
        for (const auto & existing : tl_quads) {
          const double dx = existing.cx - cx;
          const double dy = existing.cy - cy;
          const double dz = existing.cz - cz;
          if (dx * dx + dy * dy + dz * dz < 1e-4) {
            duplicate = true;
            break;
          }
        }
        if (duplicate) {
          continue;
        }

        TrafficLightQuad q;
        q.v0 = v0;
        q.v1 = v1;
        q.v2 = v2;
        q.v3 = v3;
        q.cx = cx;
        q.cy = cy;
        q.cz = cz;

        const double ex = v1.x - v0.x;
        const double ey = v1.y - v0.y;
        const double ez = v1.z - v0.z;
        const double elen = std::sqrt(ex * ex + ey * ey + ez * ez);
        if (elen > 1e-6) {
          q.ux = ex / elen;
          q.uy = ey / elen;
          q.uz = ez / elen;
        }

        const double hx = v3.x - v0.x;
        const double hy = v3.y - v0.y;
        const double hz = v3.z - v0.z;
        const double hlen = std::sqrt(hx * hx + hy * hy + hz * hz);
        if (hlen > 1e-6) {
          q.wx = hx / hlen;
          q.wy = hy / hlen;
          q.wz = hz / hlen;
        }

        tl_quads.push_back(q);
      }
    }
  }

  Marker lens_ring_marker;
  lens_ring_marker.ns = "traffic_light";
  lens_ring_marker.id = 900000;
  lens_ring_marker.type = Marker::LINE_LIST;
  lens_ring_marker.action = Marker::ADD;
  lens_ring_marker.pose.orientation.w = 1.0;
  lens_ring_marker.scale.x = std::max(0.010, tl_border_width * 0.50);
  lens_ring_marker.color = tl_border_color;
  lens_ring_marker.color.a = tl_border_color.a * 0.58F;
  bool has_lens_header = false;

  BorderSpatialGrid border_grid;
  for (const auto & orig : m_last_msg->markers) {
    if (orig.ns == "road_borders" || orig.ns == "curbstone") {
      collectRibbonSegmentsToGrid(orig, border_grid);
    }
  }

  auto styled = std::make_shared<MarkerArray>();
  styled->markers.reserve(m_last_msg->markers.size());

  std::vector<Marker> foreground_markers;
  foreground_markers.reserve(m_last_msg->markers.size());
  std::vector<Marker> border_markers;
  border_markers.reserve(8);

  for (const auto & orig : m_last_msg->markers) {
    const std::string & ns = orig.ns;

    if (
      hide_clutter &&
      (isClutterArrowOrIdNamespace(ns) || ns == "bicycle_lane_center_lane_line" ||
       ns == "shoulder_center_lane_line")) {
      continue;
    }

    Marker m = orig;

    if (isSurfaceFillNamespace(ns)) {
      if (ns == "bicycle_lane_lanelets") {
        if (override_bl) {
          applyZOffset(m, 0.004);
          m.color = bl_color;
          // Keep marker.color.a = 1.0 so TriangleListMarker uses linear vertex alpha directly
          if (m.type == Marker::TRIANGLE_LIST && !m.points.empty()) {
            m.color.a = 1.0F;
            m.colors.assign(m.points.size(), bl_color);
          }
        }
      } else if (override_rf && (ns == "road_lanelets" || ns == "shoulder_road_lanelets")) {
        applyMarkerColor(m, rf_color);
      }
      // Surface fills stay at base Z (below lane lines) and are queued first
      styled->markers.push_back(std::move(m));
      continue;
    }

    if (override_tl && ns == "traffic_light_triangle") {
      // Replaced below by deduplicated hologram plate + neon bezel outline
      continue;
    }

    if (override_tl && ns == "traffic_light" && m.type == Marker::SPHERE) {
      if (!has_lens_header) {
        lens_ring_marker.header = m.header;
        has_lens_header = true;
      }

      const float r = orig.color.r;
      const float g = orig.color.g;
      const float b = orig.color.b;
      if (r > 0.15F && g < 0.15F) {
        m.color.r = 0.80F;
        m.color.g = 0.16F;
        m.color.b = 0.22F;
      } else if (r > 0.15F && g > 0.15F && b < 0.15F) {
        m.color.r = 0.88F;
        m.color.g = 0.64F;
        m.color.b = 0.12F;
      } else if (g > 0.15F && r < 0.15F) {
        m.color.r = 0.08F;
        m.color.g = 0.82F;
        m.color.b = 0.56F;
      } else {
        m.color.r = 0.55F;
        m.color.g = 0.68F;
        m.color.b = 0.80F;
      }
      m.color.a = tl_bulb_alpha;
      m.scale.x *= 0.76;
      m.scale.y *= 0.76;
      m.scale.z *= 0.76;

      // Add a crisp circular lens bezel ring on the nearest traffic light housing plane
      const TrafficLightQuad * best_q = nullptr;
      double best_d2 = 6.25;  // max 2.5m search radius
      for (const auto & q : tl_quads) {
        const double dx = q.cx - m.pose.position.x;
        const double dy = q.cy - m.pose.position.y;
        const double dz = q.cz - m.pose.position.z;
        const double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_q = &q;
        }
      }
      if (best_q) {
        constexpr int kSegments = 16;
        constexpr double kRingRadius = 0.138;
        for (int seg = 0; seg < kSegments; ++seg) {
          const double a0 = (2.0 * M_PI * static_cast<double>(seg)) / kSegments;
          const double a1 = (2.0 * M_PI * static_cast<double>(seg + 1)) / kSegments;
          const double c0 = std::cos(a0) * kRingRadius;
          const double s0 = std::sin(a0) * kRingRadius;
          const double c1 = std::cos(a1) * kRingRadius;
          const double s1 = std::sin(a1) * kRingRadius;

          geometry_msgs::msg::Point p0;
          p0.x = m.pose.position.x + c0 * best_q->ux + s0 * best_q->wx;
          p0.y = m.pose.position.y + c0 * best_q->uy + s0 * best_q->wy;
          p0.z = m.pose.position.z + c0 * best_q->uz + s0 * best_q->wz;

          geometry_msgs::msg::Point p1;
          p1.x = m.pose.position.x + c1 * best_q->ux + s1 * best_q->wx;
          p1.y = m.pose.position.y + c1 * best_q->uy + s1 * best_q->wy;
          p1.z = m.pose.position.z + c1 * best_q->uz + s1 * best_q->wz;

          lens_ring_marker.points.push_back(p0);
          lens_ring_marker.points.push_back(p1);
        }
      }

      foreground_markers.push_back(std::move(m));
      continue;
    }

    if (ns == "road_borders" || ns == "curbstone") {
      if (override_rb) {
        resizeRibbonAndApplyZOffset(m, rb_width, z_off + 0.006);
        applyMarkerColor(m, rb_color);
      } else {
        applyZOffset(m, z_off + 0.006);
      }
      border_markers.push_back(std::move(m));
      continue;
    } else if (
      ns == "left_lane_bound" || ns == "right_lane_bound" || ns == "shoulder_left_lane_bound" ||
      ns == "shoulder_right_lane_bound" || ns == "bicycle_lane_left_lane_bound" ||
      ns == "bicycle_lane_right_lane_bound" || ns == "partitions") {
      stripOverlappingBorderSegments(m, border_grid);
      if (m.points.empty()) {
        continue;
      }
      if (override_lb) {
        resizeRibbonAndApplyZOffset(m, lb_width, z_off);
        applyMarkerColor(m, lb_color);
      } else {
        applyZOffset(m, z_off);
      }
    } else if (
      ns == "center_lane_line" || ns == "shoulder_center_lane_line" ||
      ns == "bicycle_lane_center_lane_line") {
      if (override_cl) {
        resizeRibbonAndApplyZOffset(m, cl_width, z_off * 0.85);
        applyMarkerColor(m, cl_color);
      } else {
        applyZOffset(m, z_off * 0.85);
      }
    } else if (ns == "stop_lines") {
      if (override_sl) {
        resizeRibbonAndApplyZOffset(m, sl_width, z_off);
        applyMarkerColor(m, sl_color);
      } else {
        applyZOffset(m, z_off);
      }
    } else if (ns == "pedestrian_polygon_markings" || ns == "hatched_road_markings_bound") {
      applyZOffset(m, z_off);
      if (override_sl) {
        applyMarkerColor(m, sl_color);
      }
    } else if (ns == "crosswalk_lanelets") {
      applyZOffset(m, z_off * 0.5);
    }

    foreground_markers.push_back(std::move(m));
  }

  if (override_tl && !tl_quads.empty() && has_tl_header) {
    Marker plate_marker;
    plate_marker.header = tl_header;
    plate_marker.ns = "traffic_light_triangle";
    plate_marker.id = 0;
    plate_marker.type = Marker::TRIANGLE_LIST;
    plate_marker.action = Marker::ADD;
    plate_marker.pose.orientation.w = 1.0;
    plate_marker.scale.x = 1.0;
    plate_marker.scale.y = 1.0;
    plate_marker.scale.z = 1.0;
    plate_marker.color = tl_plate_color;
    // Keep marker.color.a = 1.0 so TriangleListMarker does not square vertex alpha
    plate_marker.color.a = 1.0F;
    plate_marker.points.reserve(tl_quads.size() * 6);
    plate_marker.colors.reserve(tl_quads.size() * 6);

    std_msgs::msg::ColorRGBA bot_c = tl_plate_color;
    std_msgs::msg::ColorRGBA top_c = tl_plate_color;
    top_c.r = std::min(1.0F, tl_plate_color.r * 1.35F + 0.03F);
    top_c.g = std::min(1.0F, tl_plate_color.g * 1.35F + 0.05F);
    top_c.b = std::min(1.0F, tl_plate_color.b * 1.35F + 0.08F);

    Marker bezel_marker;
    bezel_marker.header = tl_header;
    bezel_marker.ns = "traffic_light_triangle";
    bezel_marker.id = 1;
    bezel_marker.type = Marker::LINE_LIST;
    bezel_marker.action = Marker::ADD;
    bezel_marker.pose.orientation.w = 1.0;
    bezel_marker.scale.x = tl_border_width;
    bezel_marker.color = tl_border_color;
    bezel_marker.points.reserve(tl_quads.size() * 8);

    for (const auto & q : tl_quads) {
      plate_marker.points.push_back(q.v0);
      plate_marker.colors.push_back(bot_c);
      plate_marker.points.push_back(q.v1);
      plate_marker.colors.push_back(bot_c);
      plate_marker.points.push_back(q.v2);
      plate_marker.colors.push_back(top_c);

      plate_marker.points.push_back(q.v0);
      plate_marker.colors.push_back(bot_c);
      plate_marker.points.push_back(q.v2);
      plate_marker.colors.push_back(top_c);
      plate_marker.points.push_back(q.v3);
      plate_marker.colors.push_back(top_c);

      bezel_marker.points.push_back(q.v0);
      bezel_marker.points.push_back(q.v1);
      bezel_marker.points.push_back(q.v1);
      bezel_marker.points.push_back(q.v2);
      bezel_marker.points.push_back(q.v2);
      bezel_marker.points.push_back(q.v3);
      bezel_marker.points.push_back(q.v3);
      bezel_marker.points.push_back(q.v0);
    }

    foreground_markers.push_back(std::move(plate_marker));
    foreground_markers.push_back(std::move(bezel_marker));
  }

  if (override_tl && !lens_ring_marker.points.empty() && has_lens_header) {
    foreground_markers.push_back(std::move(lens_ring_marker));
  }

  for (auto & m : foreground_markers) {
    styled->markers.push_back(std::move(m));
  }
  for (auto & m : border_markers) {
    styled->markers.push_back(std::move(m));
  }

  m_marker_common.addMessage(styled);
}

}  // namespace map
}  // namespace rviz_plugins
}  // namespace autoware

PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::map::Lanelet2VectorMapDisplay, rviz_common::Display)
