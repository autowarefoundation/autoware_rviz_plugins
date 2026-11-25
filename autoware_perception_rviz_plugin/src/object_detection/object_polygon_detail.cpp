// Copyright 2021 Apex.AI, Inc.
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
// limitations under the License..

#include "autoware_perception_rviz_plugin/object_detection/object_polygon_detail.hpp"

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <limits>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace
{
// get string of double value rounded after first decimal place
// e.g. roundAfterFirstDecimalPlace(12.345) -> "1.2"
std::string getRoundedDoubleString(const double val)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(1) << val;
  return ss.str();
}
}  // namespace

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
namespace detail
{
using Marker = visualization_msgs::msg::Marker;

visualization_msgs::msg::Marker::SharedPtr get_path_confidence_marker_ptr(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & path_confidence_color)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("path confidence");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.y = 0.5;
  marker_ptr->scale.z = 0.5;
  marker_ptr->pose = initPose();
  marker_ptr->color = path_confidence_color;
  marker_ptr->pose.position = predicted_path.path.back().position;
  marker_ptr->text = std::to_string(predicted_path.confidence);
  marker_ptr->color.a = 0.5;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_predicted_path_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & predicted_path_color, const bool is_simple)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("path");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->pose = initPose();
  marker_ptr->color = predicted_path_color;
  marker_ptr->color.a = 0.6;
  marker_ptr->scale.x = 0.015;
  calc_path_line_list(predicted_path, marker_ptr->points, is_simple);
  for (size_t k = 0; k < marker_ptr->points.size(); ++k) {
    marker_ptr->points.at(k).z -= shape.dimensions.z * 0.5;
  }
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_hologram_predicted_path_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & class_color, const HologramPathOptions & options)
{
  if (predicted_path.path.size() < 2) {
    return nullptr;
  }

  struct CtrlPt
  {
    double x;
    double y;
    double z;
    double s;
  };

  const double dt = static_cast<double>(predicted_path.time_step.sec) +
                    static_cast<double>(predicted_path.time_step.nanosec) * 1e-9;
  const double max_time = options.horizon_sec > 0.0 ? options.horizon_sec : 1e9;
  const double max_len = options.max_length > 0.0 ? options.max_length : 1e9;
  const double ground_z_offset = -static_cast<double>(shape.dimensions.z) * 0.5 + 0.072;

  std::vector<CtrlPt> ctrl;
  ctrl.reserve(predicted_path.path.size());

  const auto & p0 = predicted_path.path.front().position;
  ctrl.push_back({p0.x, p0.y, p0.z + ground_z_offset, 0.0});

  double prev_raw_x = p0.x;
  double prev_raw_y = p0.y;
  double prev_raw_z = p0.z + ground_z_offset;
  double prev_time = 0.0;
  double cum_raw_s = 0.0;

  for (size_t idx = 1; idx < predicted_path.path.size(); ++idx) {
    const auto & pos = predicted_path.path.at(idx).position;
    const double cur_x = pos.x;
    const double cur_y = pos.y;
    const double cur_z = pos.z + ground_z_offset;
    const double cur_time = (dt > 1e-4) ? (static_cast<double>(idx) * dt) : 0.0;

    const double raw_dx = cur_x - prev_raw_x;
    const double raw_dy = cur_y - prev_raw_y;
    const double raw_seg = std::hypot(raw_dx, raw_dy);

    double clip_ratio = 1.0;
    bool clipped = false;
    if (dt > 1e-4 && cur_time > max_time && cur_time > prev_time) {
      clip_ratio = std::min(clip_ratio, (max_time - prev_time) / (cur_time - prev_time));
      clipped = true;
    }
    if (raw_seg > 1e-6 && cum_raw_s + raw_seg > max_len) {
      clip_ratio = std::min(clip_ratio, (max_len - cum_raw_s) / raw_seg);
      clipped = true;
    }
    clip_ratio = std::clamp(clip_ratio, 0.0, 1.0);

    const double target_x = prev_raw_x + raw_dx * clip_ratio;
    const double target_y = prev_raw_y + raw_dy * clip_ratio;
    const double target_z = prev_raw_z + (cur_z - prev_raw_z) * clip_ratio;

    const double ctrl_dx = target_x - ctrl.back().x;
    const double ctrl_dy = target_y - ctrl.back().y;
    const double ctrl_seg = std::hypot(ctrl_dx, ctrl_dy);
    if (ctrl_seg >= 0.04 || (clipped && ctrl_seg >= 0.01)) {
      ctrl.push_back({target_x, target_y, target_z, ctrl.back().s + ctrl_seg});
    }

    cum_raw_s += raw_seg * clip_ratio;
    prev_raw_x = cur_x;
    prev_raw_y = cur_y;
    prev_raw_z = cur_z;
    prev_time = cur_time;

    if (clipped) {
      break;
    }
  }

  if (ctrl.size() < 2) {
    return nullptr;
  }

  const double total_len = ctrl.back().s;
  const double min_visible_len =
    std::max(0.45, static_cast<double>(shape.dimensions.x) * 0.28 + 0.35);
  if (total_len < min_visible_len) {
    return nullptr;
  }

  const double start_s =
    std::clamp(static_cast<double>(shape.dimensions.x) * 0.26, 0.0, total_len * 0.32);
  const double visible_len = total_len - start_s;
  if (visible_len < 0.30) {
    return nullptr;
  }

  // Compute smooth unit tangents at control points for cubic Hermite resampling
  const size_t n_ctrl = ctrl.size();
  std::vector<Eigen::Vector2d> tangents(n_ctrl, Eigen::Vector2d(1.0, 0.0));
  for (size_t i = 0; i < n_ctrl; ++i) {
    Eigen::Vector2d t_vec(0.0, 0.0);
    if (i > 0) {
      Eigen::Vector2d d_prev(ctrl[i].x - ctrl[i - 1].x, ctrl[i].y - ctrl[i - 1].y);
      const double n_prev = d_prev.norm();
      if (n_prev > 1e-6) {
        t_vec += d_prev / n_prev;
      }
    }
    if (i + 1 < n_ctrl) {
      Eigen::Vector2d d_next(ctrl[i + 1].x - ctrl[i].x, ctrl[i + 1].y - ctrl[i].y);
      const double n_next = d_next.norm();
      if (n_next > 1e-6) {
        t_vec += d_next / n_next;
      }
    }
    const double t_norm = t_vec.norm();
    if (t_norm > 1e-6) {
      tangents[i] = t_vec / t_norm;
    }
  }

  struct SamplePt
  {
    double x;
    double y;
    double z;
    double s;
    double tx;
    double ty;
  };

  auto eval_hermite = [&](size_t seg_idx, double t_param) -> CtrlPt {
    const auto & c0 = ctrl[seg_idx];
    const auto & c1 = ctrl[seg_idx + 1];
    const double seg_len = c1.s - c0.s;
    const double t2 = t_param * t_param;
    const double t3 = t2 * t_param;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t_param;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    const double mx0 = tangents[seg_idx].x() * seg_len;
    const double my0 = tangents[seg_idx].y() * seg_len;
    const double mx1 = tangents[seg_idx + 1].x() * seg_len;
    const double my1 = tangents[seg_idx + 1].y() * seg_len;
    return {
      h00 * c0.x + h10 * mx0 + h01 * c1.x + h11 * mx1,
      h00 * c0.y + h10 * my0 + h01 * c1.y + h11 * my1,
      c0.z + (c1.z - c0.z) * t_param,
      c0.s + seg_len * t_param};
  };

  std::vector<SamplePt> samples;
  samples.reserve(n_ctrl * 3 + 4);

  for (size_t seg_idx = 0; seg_idx + 1 < n_ctrl; ++seg_idx) {
    const double s0 = ctrl[seg_idx].s;
    const double s1 = ctrl[seg_idx + 1].s;
    const double seg_len = s1 - s0;
    if (seg_len < 1e-5 || s1 <= start_s) {
      continue;
    }

    const double t_begin = (s0 < start_s) ? std::clamp((start_s - s0) / seg_len, 0.0, 1.0) : 0.0;
    const double rem_len = seg_len * (1.0 - t_begin);
    const int n_sub = std::clamp(static_cast<int>(std::ceil(rem_len / 0.35)), 1, 16);

    if (samples.empty()) {
      const auto first_pt = eval_hermite(seg_idx, t_begin);
      samples.push_back({first_pt.x, first_pt.y, first_pt.z, first_pt.s, 1.0, 0.0});
    }

    for (int step = 1; step <= n_sub; ++step) {
      const double t_param =
        t_begin + (1.0 - t_begin) * (static_cast<double>(step) / static_cast<double>(n_sub));
      const auto pt = eval_hermite(seg_idx, t_param);
      const double d_prev = std::hypot(pt.x - samples.back().x, pt.y - samples.back().y);
      if (d_prev >= 0.015) {
        samples.push_back({pt.x, pt.y, pt.z, samples.back().s + d_prev, 1.0, 0.0});
      }
    }
  }

  if (samples.size() < 2) {
    return nullptr;
  }

  const double actual_start_s = samples.front().s;
  const double actual_vis_len = samples.back().s - actual_start_s;
  if (actual_vis_len < 0.25) {
    return nullptr;
  }

  // Compute smooth unit tangents on the dense sample curve
  const size_t n_samp = samples.size();
  for (size_t i = 0; i < n_samp; ++i) {
    double dx = 0.0;
    double dy = 0.0;
    if (i == 0) {
      dx = samples[1].x - samples[0].x;
      dy = samples[1].y - samples[0].y;
    } else if (i + 1 == n_samp) {
      dx = samples[i].x - samples[i - 1].x;
      dy = samples[i].y - samples[i - 1].y;
    } else {
      dx = samples[i + 1].x - samples[i - 1].x;
      dy = samples[i + 1].y - samples[i - 1].y;
    }
    const double norm = std::hypot(dx, dy);
    if (norm > 1e-6) {
      samples[i].tx = dx / norm;
      samples[i].ty = dy / norm;
    } else if (i > 0) {
      samples[i].tx = samples[i - 1].tx;
      samples[i].ty = samples[i - 1].ty;
    }
  }

  const double obj_w_factor =
    std::clamp(static_cast<double>(shape.dimensions.y) / 1.8, 0.52, 1.20);
  const double branch_w_factor = options.is_primary ? 1.0 : 0.74;
  const double base_half_w = std::max(0.05, options.width * 0.5 * obj_w_factor * branch_w_factor);

  const float conf_alpha_scale =
    options.is_primary ? 1.0F : std::clamp(options.relative_confidence * 0.62F, 0.22F, 0.58F);
  const float peak_alpha = std::clamp(options.alpha * conf_alpha_scale, 0.05F, 1.0F);
  const double fade_in_len = std::clamp(actual_vis_len * 0.14, 0.25, 0.80);

  const float cr = std::clamp(class_color.r * 1.08F + 0.04F, 0.0F, 1.0F);
  const float cg = std::clamp(class_color.g * 1.08F + 0.04F, 0.0F, 1.0F);
  const float cb = std::clamp(class_color.b * 1.08F + 0.04F, 0.0F, 1.0F);

  const float er = std::clamp(cr * 0.60F + 0.40F, 0.0F, 1.0F);
  const float eg = std::clamp(cg * 0.60F + 0.40F, 0.0F, 1.0F);
  const float eb = std::clamp(cb * 0.60F + 0.40F, 0.0F, 1.0F);

  const float vr = std::clamp(cr * 0.38F + 0.62F, 0.0F, 1.0F);
  const float vg = std::clamp(cg * 0.38F + 0.62F, 0.0F, 1.0F);
  const float vb = std::clamp(cb * 0.38F + 0.62F, 0.0F, 1.0F);

  auto make_pt = [](double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };
  auto make_col = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = std::clamp(a, 0.0F, 1.0F);
    return c;
  };

  struct CrossSection
  {
    geometry_msgs::msg::Point l_out;
    geometry_msgs::msg::Point l_in;
    geometry_msgs::msg::Point center;
    geometry_msgs::msg::Point r_in;
    geometry_msgs::msg::Point r_out;
    std_msgs::msg::ColorRGBA edge_col;
    std_msgs::msg::ColorRGBA inner_col;
    std_msgs::msg::ColorRGBA center_col;
    double half_w;
    float env;
  };

  std::vector<CrossSection> sections(n_samp);
  for (size_t i = 0; i < n_samp; ++i) {
    const auto & sp = samples[i];
    const double rel_s = sp.s - actual_start_s;
    const double u = std::clamp(rel_s / actual_vis_len, 0.0, 1.0);

    const double half_w = base_half_w * (1.0 - 0.50 * u);
    const double edge_w = std::clamp(half_w * 0.22, 0.012, 0.032);
    const double inner_w = std::max(0.0, half_w - edge_w);

    const double fin = std::clamp(rel_s / fade_in_len, 0.25, 1.0);
    const double fout = std::pow(std::max(0.0, 1.0 - std::pow(u, 1.85)), 1.15);
    const float env = static_cast<float>(fin * fout);
    const float a = env * peak_alpha;

    const double nx = -sp.ty;
    const double ny = sp.tx;

    sections[i].l_out = make_pt(sp.x + nx * half_w, sp.y + ny * half_w, sp.z);
    sections[i].l_in = make_pt(sp.x + nx * inner_w, sp.y + ny * inner_w, sp.z);
    sections[i].center = make_pt(sp.x, sp.y, sp.z);
    sections[i].r_in = make_pt(sp.x - nx * inner_w, sp.y - ny * inner_w, sp.z);
    sections[i].r_out = make_pt(sp.x - nx * half_w, sp.y - ny * half_w, sp.z);

    sections[i].edge_col = make_col(er, eg, eb, std::min(0.96F, a * 1.18F));
    sections[i].inner_col = make_col(cr, cg, cb, a * 0.24F);
    sections[i].center_col = make_col(cr, cg, cb, a * 0.44F);
    sections[i].half_w = half_w;
    sections[i].env = env;
  }

  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker_ptr->ns = std::string("path_hologram");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->pose = initPose();
  marker_ptr->scale.x = 1.0;
  marker_ptr->scale.y = 1.0;
  marker_ptr->scale.z = 1.0;
  // Keep marker.color opaque white so RViz2 TriangleListMarker uses per-vertex alpha directly
  marker_ptr->color = make_col(1.0F, 1.0F, 1.0F, 1.0F);

  marker_ptr->points.reserve((n_samp - 1) * 24 + 96);
  marker_ptr->colors.reserve((n_samp - 1) * 24 + 96);

  auto push_quad_ccw = [&](
                         const geometry_msgs::msg::Point & a0,
                         const std_msgs::msg::ColorRGBA & ca0,
                         const geometry_msgs::msg::Point & b0,
                         const std_msgs::msg::ColorRGBA & cb0,
                         const geometry_msgs::msg::Point & b1,
                         const std_msgs::msg::ColorRGBA & cb1,
                         const geometry_msgs::msg::Point & a1,
                         const std_msgs::msg::ColorRGBA & ca1) {
    // a0, a1 on left (+N side); b0, b1 on right (-N side) -> CCW (+Z upward normal)
    marker_ptr->points.push_back(a0);
    marker_ptr->colors.push_back(ca0);
    marker_ptr->points.push_back(b0);
    marker_ptr->colors.push_back(cb0);
    marker_ptr->points.push_back(b1);
    marker_ptr->colors.push_back(cb1);

    marker_ptr->points.push_back(a0);
    marker_ptr->colors.push_back(ca0);
    marker_ptr->points.push_back(b1);
    marker_ptr->colors.push_back(cb1);
    marker_ptr->points.push_back(a1);
    marker_ptr->colors.push_back(ca1);
  };

  for (size_t i = 0; i + 1 < n_samp; ++i) {
    const auto & s0 = sections[i];
    const auto & s1 = sections[i + 1];

    push_quad_ccw(
      s0.l_out, s0.edge_col, s0.l_in, s0.inner_col, s1.l_in, s1.inner_col, s1.l_out, s1.edge_col);
    push_quad_ccw(
      s0.l_in, s0.inner_col, s0.center, s0.center_col, s1.center, s1.center_col, s1.l_in,
      s1.inner_col);
    push_quad_ccw(
      s0.center, s0.center_col, s0.r_in, s0.inner_col, s1.r_in, s1.inner_col, s1.center,
      s1.center_col);
    push_quad_ccw(
      s0.r_in, s0.inner_col, s0.r_out, s0.edge_col, s1.r_out, s1.edge_col, s1.r_in, s1.inner_col);
  }

  // Forward V-chevrons ('>') along primary / high-confidence branches
  if ((options.is_primary || options.relative_confidence >= 0.65F) && actual_vis_len >= 1.1) {
    const double chevron_step = std::clamp(base_half_w * 12.5, 1.50, 2.75);
    const double s_chevron_start = actual_start_s + std::min(chevron_step * 0.45, 0.90);
    const double s_chevron_end = samples.back().s - std::min(chevron_step * 0.25, 0.45);

    size_t search_idx = 0;
    for (double sc = s_chevron_start; sc <= s_chevron_end; sc += chevron_step) {
      while (search_idx + 2 < n_samp && samples[search_idx + 1].s < sc) {
        ++search_idx;
      }
      const auto & sp0 = samples[search_idx];
      const auto & sp1 = samples[search_idx + 1];
      const double seg_ds = std::max(1e-6, sp1.s - sp0.s);
      const double frac = std::clamp((sc - sp0.s) / seg_ds, 0.0, 1.0);

      const double px = sp0.x + (sp1.x - sp0.x) * frac;
      const double py = sp0.y + (sp1.y - sp0.y) * frac;
      const double pz = sp0.z + (sp1.z - sp0.z) * frac + 0.003;

      double tx = sp0.tx + (sp1.tx - sp0.tx) * frac;
      double ty = sp0.ty + (sp1.ty - sp0.ty) * frac;
      const double t_norm = std::hypot(tx, ty);
      if (t_norm > 1e-6) {
        tx /= t_norm;
        ty /= t_norm;
      }
      const double nx = -ty;
      const double ny = tx;

      const double hw =
        sections[search_idx].half_w +
        (sections[search_idx + 1].half_w - sections[search_idx].half_w) * frac;
      const float env =
        sections[search_idx].env +
        (sections[search_idx + 1].env - sections[search_idx].env) * static_cast<float>(frac);

      const float chev_a = std::clamp(env * peak_alpha * 1.30F, 0.0F, 0.98F);
      if (chev_a < 0.04F) {
        continue;
      }

      const double wing_y = hw * 0.66;
      const double wing_back = wing_y * 0.90;
      const double hs = std::clamp(hw * 0.15, 0.015, 0.032);

      const auto tip_col = make_col(vr, vg, vb, chev_a);
      const auto wing_col = make_col(er, eg, eb, chev_a * 0.72F);

      const auto tip_f = make_pt(px + tx * hs, py + ty * hs, pz);
      const auto tip_b = make_pt(px - tx * hs, py - ty * hs, pz);
      const auto wl_f =
        make_pt(px - tx * (wing_back - hs) + nx * wing_y, py - ty * (wing_back - hs) + ny * wing_y, pz);
      const auto wl_b =
        make_pt(px - tx * (wing_back + hs) + nx * wing_y, py - ty * (wing_back + hs) + ny * wing_y, pz);
      const auto wr_f =
        make_pt(px - tx * (wing_back - hs) - nx * wing_y, py - ty * (wing_back - hs) - ny * wing_y, pz);
      const auto wr_b =
        make_pt(px - tx * (wing_back + hs) - nx * wing_y, py - ty * (wing_back + hs) - ny * wing_y, pz);

      // Left chevron arm (CCW +Z normal)
      push_quad_ccw(wl_b, wing_col, tip_b, tip_col, tip_f, tip_col, wl_f, wing_col);
      // Right chevron arm (CCW +Z normal)
      push_quad_ccw(tip_b, tip_col, wr_b, wing_col, wr_f, wing_col, tip_f, tip_col);
    }
  }

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_predicted_path_footprint_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const std_msgs::msg::ColorRGBA & predicted_path_color, const bool is_simple)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("path footprint");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->pose = initPose();
  marker_ptr->color = predicted_path_color;
  marker_ptr->color.a = 0.5;
  marker_ptr->scale.x = 0.015;
  calc_path_box_line_list(shape, predicted_path, marker_ptr->points, is_simple);
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_twist_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("twist");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // velocity line
  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  point.z = 0.0;
  marker_ptr->points.push_back(point);
  point.x = twist_with_covariance.twist.linear.x;
  point.y = twist_with_covariance.twist.linear.y;
  point.z = twist_with_covariance.twist.linear.z;
  marker_ptr->points.push_back(point);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.999;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.0;
  marker_ptr->color.b = 0.0;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_twist_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
  const double & confidence_interval_coefficient)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
  marker_ptr->ns = std::string("twist covariance");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // position is the tip of the velocity vector
  const double velocity = std::sqrt(
    twist_with_covariance.twist.linear.x * twist_with_covariance.twist.linear.x +
    twist_with_covariance.twist.linear.y * twist_with_covariance.twist.linear.y);
  const double velocity_angle =
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x);
  const double pos_yaw_angle = 2.0 * std::atan2(
                                       pose_with_covariance.pose.orientation.z,
                                       pose_with_covariance.pose.orientation.w);  // [rad]
  marker_ptr->pose.position.x += velocity * std::cos(pos_yaw_angle + velocity_angle);
  marker_ptr->pose.position.y += velocity * std::sin(pos_yaw_angle + velocity_angle);

  // velocity covariance
  // extract eigen values and eigen vectors
  Eigen::Matrix2d eigen_twist_covariance;
  eigen_twist_covariance << twist_with_covariance.covariance[0],
    twist_with_covariance.covariance[1], twist_with_covariance.covariance[6],
    twist_with_covariance.covariance[7];
  double phi, sigma1, sigma2;
  calc_covariance_eigen_vectors(eigen_twist_covariance, sigma1, sigma2, phi);
  phi = pos_yaw_angle + phi;
  double area = sigma1 * sigma2;
  double alpha = std::min(0.5, 1.0 / area);
  alpha = std::max(0.1, alpha);

  // ellipse orientation
  marker_ptr->pose.orientation.x = 0.0;
  marker_ptr->pose.orientation.y = 0.0;
  marker_ptr->pose.orientation.z = std::sin(phi * 0.5);
  marker_ptr->pose.orientation.w = std::cos(phi * 0.5);

  // ellipse size
  marker_ptr->scale.x = sigma1 * confidence_interval_coefficient;
  marker_ptr->scale.y = sigma2 * confidence_interval_coefficient;
  marker_ptr->scale.z = 0.05;

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = alpha;
  marker_ptr->color.r = 0.2;
  marker_ptr->color.g = 0.4;
  marker_ptr->color.b = 0.9;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_yaw_rate_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_ptr->ns = std::string("yaw rate");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // yaw rate
  const double yaw_rate = twist_with_covariance.twist.angular.z;
  const double velocity = std::sqrt(
    twist_with_covariance.twist.linear.x * twist_with_covariance.twist.linear.x +
    twist_with_covariance.twist.linear.y * twist_with_covariance.twist.linear.y +
    twist_with_covariance.twist.linear.z * twist_with_covariance.twist.linear.z);
  const double velocity_angle =
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x);
  const double yaw_mark_length = velocity * 0.8;

  geometry_msgs::msg::Point point;
  // first point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);
  // yaw rate arc
  calc_arc_line_strip(
    velocity_angle, velocity_angle + yaw_rate, yaw_mark_length, marker_ptr->points);
  // last point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.9;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.0;
  marker_ptr->color.b = 0.0;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_yaw_rate_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
  const double & confidence_interval_coefficient, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_ptr->ns = std::string("yaw rate covariance");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;

  // yaw rate covariance
  const double yaw_rate_covariance = twist_with_covariance.covariance[35];
  const double yaw_rate_sigma = std::sqrt(yaw_rate_covariance) * confidence_interval_coefficient;
  const double yaw_rate = twist_with_covariance.twist.angular.z;
  const double velocity = std::sqrt(
    twist_with_covariance.twist.linear.x * twist_with_covariance.twist.linear.x +
    twist_with_covariance.twist.linear.y * twist_with_covariance.twist.linear.y +
    twist_with_covariance.twist.linear.z * twist_with_covariance.twist.linear.z);
  const double velocity_angle =
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x);
  const double yaw_mark_length = velocity * 0.8;
  const double bar_width = std::max(velocity * 0.05, 0.1);
  const double velocity_yaw_angle = velocity_angle + yaw_rate;
  const double velocity_yaw_p_sigma_angle = velocity_yaw_angle + yaw_rate_sigma;
  const double velocity_yaw_n_sigma_angle = velocity_yaw_angle - yaw_rate_sigma;

  const double point_list[7][3] = {
    {yaw_mark_length * std::cos(velocity_yaw_angle), yaw_mark_length * std::sin(velocity_yaw_angle),
     0},
    {yaw_mark_length * std::cos(velocity_yaw_p_sigma_angle),
     yaw_mark_length * std::sin(velocity_yaw_p_sigma_angle), 0},
    {yaw_mark_length * std::cos(velocity_yaw_n_sigma_angle),
     yaw_mark_length * std::sin(velocity_yaw_n_sigma_angle), 0},
    {(yaw_mark_length + bar_width) * std::cos(velocity_yaw_p_sigma_angle),
     (yaw_mark_length + bar_width) * std::sin(velocity_yaw_p_sigma_angle), 0},
    {(yaw_mark_length - bar_width) * std::cos(velocity_yaw_p_sigma_angle),
     (yaw_mark_length - bar_width) * std::sin(velocity_yaw_p_sigma_angle), 0},
    {(yaw_mark_length + bar_width) * std::cos(velocity_yaw_n_sigma_angle),
     (yaw_mark_length + bar_width) * std::sin(velocity_yaw_n_sigma_angle), 0},
    {(yaw_mark_length - bar_width) * std::cos(velocity_yaw_n_sigma_angle),
     (yaw_mark_length - bar_width) * std::sin(velocity_yaw_n_sigma_angle), 0},
  };
  const int point_pairs[4][2] = {
    {0, 1},
    {0, 2},
    {3, 4},
    {5, 6},
  };
  calc_line_list_from_points(point_list, point_pairs, 4, marker_ptr->points);

  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.9;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 0.2;
  marker_ptr->color.b = 0.4;

  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_velocity_text_marker_ptr(
  const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("velocity");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;

  double vel = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);
  marker_ptr->text = std::to_string(static_cast<int>(vel * 3.6)) + std::string("[km/h]");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = vis_pos;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_acceleration_text_marker_ptr(
  const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Point & vis_pos,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("acceleration");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;

  double acc = std::sqrt(
    accel.linear.x * accel.linear.x + accel.linear.y * accel.linear.y +
    accel.linear.z * accel.linear.z);
  marker_ptr->text = getRoundedDoubleString(acc) + std::string("[m/s^2]");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = vis_pos;
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

void calc_arc_line_strip(
  const double start_angle, const double end_angle, const double radius,
  std::vector<geometry_msgs::msg::Point> & points)
{
  geometry_msgs::msg::Point point;
  // arc points
  const double maximum_delta_angle = 10.0 * M_PI / 180.0;
  const int num_points =
    std::max(3, static_cast<int>(std::abs(end_angle - start_angle) / maximum_delta_angle));
  for (int i = 0; i < num_points; ++i) {
    const double angle = start_angle + (end_angle - start_angle) * static_cast<double>(i) /
                                         static_cast<double>(num_points - 1);
    point.x = radius * std::cos(angle);
    point.y = radius * std::sin(angle);
    point.z = 0;
    points.push_back(point);
  }
}

void calc_covariance_eigen_vectors(
  const Eigen::Matrix2d & matrix, double & sigma1, double & sigma2, double & yaw)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(matrix);
  Eigen::Vector2d eigen_values = solver.eigenvalues();
  // eigen values
  sigma1 = std::sqrt(eigen_values.x());
  sigma2 = std::sqrt(eigen_values.y());
  // orientation of covariance ellipse
  Eigen::Vector2d e1 = solver.eigenvectors().col(0);
  yaw = std::atan2(e1.y(), e1.x());
}

visualization_msgs::msg::Marker::SharedPtr get_pose_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
  const double & confidence_interval_coefficient)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
  marker_ptr->ns = std::string("position covariance");
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose.position = pose_with_covariance.pose.position;
  // orientation is for the given frame_id

  // position covariance
  // extract eigen values and eigen vectors
  Eigen::Matrix2d eigen_pose_covariance;
  eigen_pose_covariance << pose_with_covariance.covariance[0], pose_with_covariance.covariance[1],
    pose_with_covariance.covariance[6], pose_with_covariance.covariance[7];
  double yaw, sigma1, sigma2;
  calc_covariance_eigen_vectors(eigen_pose_covariance, sigma1, sigma2, yaw);

  // ellipse orientation
  marker_ptr->pose.orientation.x = 0.0;
  marker_ptr->pose.orientation.y = 0.0;
  marker_ptr->pose.orientation.z = std::sin(yaw * 0.5);
  marker_ptr->pose.orientation.w = std::cos(yaw * 0.5);

  // ellipse size
  marker_ptr->scale.x = sigma1 * confidence_interval_coefficient;
  marker_ptr->scale.y = sigma2 * confidence_interval_coefficient;
  marker_ptr->scale.z = 0.05;

  // ellipse color density
  double area = sigma1 * sigma2;
  double alpha = std::min(0.5, 3.0 / area);
  alpha = std::max(0.1, alpha);

  // marker configuration
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = alpha;
  marker_ptr->color.r = 0.8;
  marker_ptr->color.g = 0.8;
  marker_ptr->color.b = 0.8;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_yaw_covariance_marker_ptr(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance, const double & length,
  const double & confidence_interval_coefficient, const double & line_width)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_ptr->ns = std::string("yaw covariance");
  marker_ptr->scale.x = line_width;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = pose_with_covariance.pose;
  geometry_msgs::msg::Point point;

  // orientation covariance
  double yaw_vector_length = std::max(length, 1.0);
  double yaw_sigma =
    std::sqrt(pose_with_covariance.covariance[35]) * confidence_interval_coefficient;
  // get arc points
  if (yaw_sigma > M_PI) {
    yaw_vector_length = 1.0;
  }
  // first point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);
  // arc points
  calc_arc_line_strip(-yaw_sigma, yaw_sigma, yaw_vector_length, marker_ptr->points);
  // last point
  point.x = 0;
  point.y = 0;
  point.z = 0;
  marker_ptr->points.push_back(point);

  // marker configuration
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color.a = 0.9;
  marker_ptr->color.r = 1.0;
  marker_ptr->color.g = 1.0;
  marker_ptr->color.b = 1.0;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_uuid_marker_ptr(
  const std::string & uuid, const geometry_msgs::msg::Point & centroid,
  const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("uuid");
  marker_ptr->text = uuid.substr(0, 4);
  marker_ptr->action = Marker::MODIFY;
  marker_ptr->scale.z = 0.5;
  marker_ptr->color = color_rgba;
  marker_ptr->pose.position = centroid;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_label_marker_ptr(
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std::string & label, const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("label");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;
  marker_ptr->text = label;
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_existence_probability_marker_ptr(
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const float existence_probability, const std_msgs::msg::ColorRGBA & color_rgba)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_ptr->ns = std::string("existence probability");
  marker_ptr->scale.x = 0.5;
  marker_ptr->scale.z = 0.5;
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << existence_probability;
  marker_ptr->text = oss.str();
  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->color = color_rgba;
  return marker_ptr;
}

visualization_msgs::msg::Marker::SharedPtr get_shape_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width,
  const bool & is_orientation_available, const ObjectFillType fill_type,
  const bool & display_footprint, const bool & show_direction_indicator)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->ns = std::string("shape");
  marker_ptr->color = color_rgba;
  marker_ptr->scale.x = line_width;

  // Filled shapes are opaque solids, so cap the user-selected alpha to keep them see-through.
  constexpr float max_fill_alpha = 0.75f;

  using autoware_perception_msgs::msg::Shape;
  if (shape_msg.type == Shape::BOUNDING_BOX) {
    if (fill_type == ObjectFillType::Skeleton) {
      marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
      calc_bounding_box_line_list(shape_msg, marker_ptr->points);
      if (display_footprint && !shape_msg.footprint.points.empty()) {
        calc_polygon_line_list(shape_msg, marker_ptr->points);
      }
      if (show_direction_indicator) {
        if (is_orientation_available) {
          calc_bounding_box_direction_line_list(shape_msg, marker_ptr->points);
        } else {
          calc_bounding_box_orientation_line_list(shape_msg, marker_ptr->points);
        }
      }
    } else if (fill_type == ObjectFillType::Fill) {
      marker_ptr->type = visualization_msgs::msg::Marker::CUBE;
      marker_ptr->scale = shape_msg.dimensions;
      marker_ptr->color.a = std::min(color_rgba.a, max_fill_alpha);
    } else if (fill_type == ObjectFillType::HologramFill) {
      marker_ptr->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      marker_ptr->scale.x = 1.0;
      marker_ptr->scale.y = 1.0;
      marker_ptr->scale.z = 1.0;
      std_msgs::msg::ColorRGBA capped_color = color_rgba;
      capped_color.a = std::min(color_rgba.a, max_fill_alpha);
      // TriangleListMarker multiplies vertex_color.a * marker.color.a, so keep marker.color.a = 1.0
      marker_ptr->color = capped_color;
      marker_ptr->color.a = 1.0F;
      calc_bounding_box_triangle_list(
        shape_msg, capped_color, marker_ptr->points, marker_ptr->colors);
    }
  } else if (shape_msg.type == Shape::CYLINDER) {
    if (fill_type == ObjectFillType::Skeleton) {
      marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
      calc_cylinder_line_list(shape_msg, marker_ptr->points);
    } else if (fill_type == ObjectFillType::Fill) {
      marker_ptr->type = visualization_msgs::msg::Marker::CYLINDER;
      marker_ptr->scale = shape_msg.dimensions;
      marker_ptr->color.a = std::min(color_rgba.a, max_fill_alpha);
    } else if (fill_type == ObjectFillType::HologramFill) {
      marker_ptr->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      marker_ptr->scale.x = 1.0;
      marker_ptr->scale.y = 1.0;
      marker_ptr->scale.z = 1.0;
      std_msgs::msg::ColorRGBA capped_color = color_rgba;
      capped_color.a = std::min(color_rgba.a, max_fill_alpha);
      marker_ptr->color = capped_color;
      marker_ptr->color.a = 1.0F;
      calc_cylinder_triangle_list(shape_msg, capped_color, marker_ptr->points, marker_ptr->colors);
    }
  } else {  // including shape_msg.type == Shape::POLYGON
    if (
      (fill_type == ObjectFillType::Fill || fill_type == ObjectFillType::HologramFill) &&
      shape_msg.footprint.points.size() >= 3) {
      marker_ptr->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      marker_ptr->scale.x = 1.0;
      marker_ptr->scale.y = 1.0;
      marker_ptr->scale.z = 1.0;
      std_msgs::msg::ColorRGBA capped_color = color_rgba;
      capped_color.a = std::min(color_rgba.a, max_fill_alpha);
      marker_ptr->color = capped_color;
      calc_polygon_triangle_list(shape_msg, marker_ptr->points);
      if (fill_type == ObjectFillType::HologramFill) {
        marker_ptr->color.a = 1.0F;
        marker_ptr->colors.reserve(marker_ptr->points.size());
        auto shaded = [&capped_color](const float factor) {
          std_msgs::msg::ColorRGBA c = capped_color;
          c.r = std::min(1.0F, c.r * factor);
          c.g = std::min(1.0F, c.g * factor);
          c.b = std::min(1.0F, c.b * factor);
          return c;
        };
        // calc_polygon_triangle_list emits 12 vertices per edge: 3 top, 3 bottom, 6 side
        for (size_t i = 0; i + 11 < marker_ptr->points.size(); i += 12) {
          const auto top_c = shaded(1.00F);
          const auto bot_c = shaded(0.76F);
          const auto side_c = shaded(0.88F);
          marker_ptr->colors.insert(marker_ptr->colors.end(), 3, top_c);
          marker_ptr->colors.insert(marker_ptr->colors.end(), 3, bot_c);
          marker_ptr->colors.insert(marker_ptr->colors.end(), 6, side_c);
        }
      }
    } else {
      marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
      calc_polygon_line_list(shape_msg, marker_ptr->points);
    }
  }

  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);

  return marker_ptr;
}

// cppcheck-suppress unusedFunction
visualization_msgs::msg::Marker::SharedPtr get_2d_shape_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width,
  const bool & is_orientation_available, const bool & display_footprint)
{
  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->ns = std::string("shape");

  using autoware_perception_msgs::msg::Shape;
  if (shape_msg.type == Shape::BOUNDING_BOX) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_bounding_box_bottom_line_list(shape_msg, marker_ptr->points);
    if (display_footprint && !shape_msg.footprint.points.empty()) {
      calc_2d_polygon_bottom_line_list(shape_msg, marker_ptr->points);
    }
    if (is_orientation_available) {
      calc_2d_bounding_box_bottom_direction_line_list(shape_msg, marker_ptr->points);
    } else {
      calc_2d_bounding_box_bottom_orientation_line_list(shape_msg, marker_ptr->points);
    }
  } else if (shape_msg.type == Shape::CYLINDER) {
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_cylinder_bottom_line_list(shape_msg, marker_ptr->points);
  } else {  // including shape_msg.type == Shape::POLYGON
    marker_ptr->type = visualization_msgs::msg::Marker::LINE_LIST;
    calc_2d_polygon_bottom_line_list(shape_msg, marker_ptr->points);
  }

  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
  marker_ptr->scale.x = line_width;
  marker_ptr->color = color_rgba;

  return marker_ptr;
}

void calc_line_list_from_points(
  const double point_list[][3], const int point_pairs[][2], const int & num_pairs,
  std::vector<geometry_msgs::msg::Point> & points)
{
  geometry_msgs::msg::Point point;
  for (int i = 0; i < num_pairs; ++i) {
    point.x = point_list[point_pairs[i][0]][0];
    point.y = point_list[point_pairs[i][0]][1];
    point.z = point_list[point_pairs[i][0]][2];
    points.push_back(point);
    point.x = point_list[point_pairs[i][1]][0];
    point.y = point_list[point_pairs[i][1]][1];
    point.z = point_list[point_pairs[i][1]][2];
    points.push_back(point);
  }
}

visualization_msgs::msg::Marker::SharedPtr get_mesh_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  const auto label = detail::get_best_label(classification, kLoggerName);
  std::string mesh_name;
  switch (label) {
    case autoware_perception_msgs::msg::ObjectClassification::ANIMAL:
      return nullptr;
      break;
    case autoware_perception_msgs::msg::ObjectClassification::HAZARD:
      return nullptr;
      break;
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
      mesh_name = "car";
      if (shape_msg.dimensions.z * 2.8 > shape_msg.dimensions.x) {
        mesh_name = "suv";
      }
      break;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      mesh_name = "pedestrian";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      mesh_name = "bicycle";
      if (shape_msg.dimensions.z * 1.5 > shape_msg.dimensions.x) {
        mesh_name = "cyclist";
      }
      break;
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
      mesh_name = "truck";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
      mesh_name = "bus";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      mesh_name = "trailer";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      mesh_name = "motorcycle";
      break;
    default:
      return nullptr;
      break;
  }

  auto marker_ptr = std::make_shared<Marker>();
  marker_ptr->ns = std::string("mesh/" + mesh_name);
  marker_ptr->mesh_resource =
    "package://autoware_perception_rviz_plugin/meshes/" + mesh_name + ".dae";
  marker_ptr->type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  marker_ptr->scale.x = shape_msg.dimensions.x;
  marker_ptr->scale.y = shape_msg.dimensions.y;
  marker_ptr->scale.z = shape_msg.dimensions.z;

  marker_ptr->action = visualization_msgs::msg::Marker::MODIFY;
  marker_ptr->pose = to_pose(centroid, orientation);
  marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker_ptr->mesh_use_embedded_materials = true;

  for (const auto & indicator : classification) {
    if (indicator.label == kRgbColorLabel) {  // RGB color override
      int rgb = static_cast<int>(indicator.probability);
      marker_ptr->color.r = ((rgb >> 16) & 0xFF) / 255.0f;
      marker_ptr->color.g = ((rgb >> 8) & 0xFF) / 255.0f;
      marker_ptr->color.b = (rgb & 0xFF) / 255.0f;
      marker_ptr->color.a = 1.0;
      marker_ptr->mesh_use_embedded_materials = false;
      break;
    }
  }

  return marker_ptr;
}

visualization_msgs::msg::MarkerArray::SharedPtr get_indicator_marker_ptr(
  const autoware_perception_msgs::msg::Shape & shape_msg,
  const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
{
  const auto label = detail::get_best_label(classification, kLoggerName);
  std::string mesh_name;
  switch (label) {
    case autoware_perception_msgs::msg::ObjectClassification::ANIMAL:
      return nullptr;
      break;
    case autoware_perception_msgs::msg::ObjectClassification::HAZARD:
      return nullptr;
      break;
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
      mesh_name = "car";
      if (shape_msg.dimensions.z * 2.8 > shape_msg.dimensions.x) {
        mesh_name = "suv";
      }
      break;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return nullptr;  // There are currently no indicators for pedestrians
      break;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return nullptr;  // There are currently no indicators for bicycles
      break;
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
      mesh_name = "truck";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
      mesh_name = "bus";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      mesh_name = "trailer";
      break;
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
      return nullptr;  // There are currently no indicators for the motorcycle
      break;
    default:
      return nullptr;
      break;
  }

  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();
  for (const auto & indicator : classification) {
    if (
      indicator.label == kBrakeLightLabel || indicator.label == kLeftIndicatorLabel ||
      indicator.label == kRightIndicatorLabel) {
      Marker marker;
      marker.ns = std::string("indicator");

      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      if (indicator.label == kBrakeLightLabel) {
        marker.mesh_resource =
          "package://autoware_perception_rviz_plugin/meshes/" + mesh_name + "-brake.dae";
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else {
        if (indicator.label == kLeftIndicatorLabel) {
          marker.mesh_resource =
            "package://autoware_perception_rviz_plugin/meshes/" + mesh_name + "-indicator-left.dae";
        } else if (indicator.label == kRightIndicatorLabel) {
          marker.mesh_resource = "package://autoware_perception_rviz_plugin/meshes/" + mesh_name +
                                 "-indicator-right.dae";
        }
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }

      marker.scale.x = shape_msg.dimensions.x;
      marker.scale.y = shape_msg.dimensions.y;
      marker.scale.z = shape_msg.dimensions.z;

      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = to_pose(centroid, orientation);
      marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker.mesh_use_embedded_materials = false;
      marker.color.a = indicator.probability;

      markers->markers.push_back(marker);
    }
  }

  return markers;
}

void calc_bounding_box_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  geometry_msgs::msg::Point point;

  // bounding box corner points
  // top and bottom surface, clockwise
  const double point_list[8][3] = {
    {length_half, width_half, height_half},    {length_half, -width_half, height_half},
    {-length_half, -width_half, height_half},  {-length_half, width_half, height_half},
    {length_half, width_half, -height_half},   {length_half, -width_half, -height_half},
    {-length_half, -width_half, -height_half}, {-length_half, width_half, -height_half},
  };
  const int point_pairs[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };
  calc_line_list_from_points(point_list, point_pairs, 12, points);
}

void calc_bounding_box_triangle_list(
  const autoware_perception_msgs::msg::Shape & shape, const std_msgs::msg::ColorRGBA & base_color,
  std::vector<geometry_msgs::msg::Point> & points, std::vector<std_msgs::msg::ColorRGBA> & colors)
{
  constexpr double shrink = 0.994;
  const double hx = shape.dimensions.x * 0.5 * shrink;
  const double hy = shape.dimensions.y * 0.5 * shrink;
  const double hz = shape.dimensions.z * 0.5 * shrink;

  const double v[8][3] = {
    {hx, hy, hz},     // 0: front-left-top
    {hx, -hy, hz},    // 1: front-right-top
    {-hx, -hy, hz},   // 2: rear-right-top
    {-hx, hy, hz},    // 3: rear-left-top
    {hx, hy, -hz},    // 4: front-left-bottom
    {hx, -hy, -hz},   // 5: front-right-bottom
    {-hx, -hy, -hz},  // 6: rear-right-bottom
    {-hx, hy, -hz},   // 7: rear-left-bottom
  };

  struct FaceQuad
  {
    int idx[4];
    float shade;
  };

  // All 6 faces with gentle unlit shading so every face stays clearly colored regardless of
  // camera/light direction while preserving 3D depth cues.
  const FaceQuad faces[6] = {
    {{0, 3, 2, 1}, 1.00F},  // Top (+z)
    {{4, 5, 6, 7}, 0.76F},  // Bottom (-z)
    {{0, 1, 5, 4}, 0.92F},  // Front (+x)
    {{2, 3, 7, 6}, 0.92F},  // Rear (-x)
    {{3, 0, 4, 7}, 0.86F},  // Left (+y)
    {{1, 2, 6, 5}, 0.86F},  // Right (-y)
  };

  points.reserve(points.size() + 36);
  colors.reserve(colors.size() + 36);

  auto push_vtx = [&](const int corner, const std_msgs::msg::ColorRGBA & c) {
    geometry_msgs::msg::Point p;
    p.x = v[corner][0];
    p.y = v[corner][1];
    p.z = v[corner][2];
    points.push_back(p);
    colors.push_back(c);
  };

  for (const auto & f : faces) {
    std_msgs::msg::ColorRGBA c = base_color;
    c.r = std::min(1.0F, c.r * f.shade);
    c.g = std::min(1.0F, c.g * f.shade);
    c.b = std::min(1.0F, c.b * f.shade);

    push_vtx(f.idx[0], c);
    push_vtx(f.idx[1], c);
    push_vtx(f.idx[2], c);

    push_vtx(f.idx[0], c);
    push_vtx(f.idx[2], c);
    push_vtx(f.idx[3], c);
  }
}

void calc_cylinder_triangle_list(
  const autoware_perception_msgs::msg::Shape & shape, const std_msgs::msg::ColorRGBA & base_color,
  std::vector<geometry_msgs::msg::Point> & points, std::vector<std_msgs::msg::ColorRGBA> & colors)
{
  constexpr double shrink = 0.994;
  constexpr int n = 20;
  const double radius = shape.dimensions.x * 0.5 * shrink;
  const double hz = shape.dimensions.z * 0.5 * shrink;

  auto shaded = [&base_color](const float factor) {
    std_msgs::msg::ColorRGBA c = base_color;
    c.r = std::min(1.0F, c.r * factor);
    c.g = std::min(1.0F, c.g * factor);
    c.b = std::min(1.0F, c.b * factor);
    return c;
  };

  const auto top_c = shaded(1.00F);
  const auto bot_c = shaded(0.76F);
  const auto side_c = shaded(0.88F);

  points.reserve(points.size() + n * 12);
  colors.reserve(colors.size() + n * 12);

  auto push_pt =
    [&](const double x, const double y, const double z, const std_msgs::msg::ColorRGBA & c) {
      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      points.push_back(p);
      colors.push_back(c);
    };

  for (int i = 0; i < n; ++i) {
    const double a0 = (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                      M_PI / static_cast<double>(n);
    const double a1 = (static_cast<double>(i + 1) / static_cast<double>(n)) * 2.0 * M_PI +
                      M_PI / static_cast<double>(n);
    const double x0 = std::cos(a0) * radius;
    const double y0 = std::sin(a0) * radius;
    const double x1 = std::cos(a1) * radius;
    const double y1 = std::sin(a1) * radius;

    // Top cap
    push_pt(0.0, 0.0, hz, top_c);
    push_pt(x0, y0, hz, top_c);
    push_pt(x1, y1, hz, top_c);

    // Bottom cap
    push_pt(0.0, 0.0, -hz, bot_c);
    push_pt(x1, y1, -hz, bot_c);
    push_pt(x0, y0, -hz, bot_c);

    // Side quad
    push_pt(x0, y0, -hz, side_c);
    push_pt(x1, y1, -hz, side_c);
    push_pt(x1, y1, hz, side_c);

    push_pt(x0, y0, -hz, side_c);
    push_pt(x1, y1, hz, side_c);
    push_pt(x0, y0, hz, side_c);
  }
}

void calc_bounding_box_direction_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  // direction triangle
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double triangle_size_half = std::min(width_half * 1.4, shape.dimensions.x);
  geometry_msgs::msg::Point point;

  // triangle-shaped direction indicator
  const double point_list[6][3] = {
    {length_half, 0, height_half},
    {length_half - triangle_size_half, width_half, height_half},
    {length_half - triangle_size_half, -width_half, height_half},
    {length_half, 0, -height_half},
    {length_half, width_half, height_half},
    {length_half, -width_half, height_half},
  };
  const int point_pairs[5][2] = {
    {0, 1}, {1, 2}, {0, 2}, {3, 4}, {3, 5},
  };
  calc_line_list_from_points(point_list, point_pairs, 5, points);
}

void calc_bounding_box_orientation_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double tick_width = width_half * 0.5;
  const double tick_length = std::min(tick_width, length_half);
  geometry_msgs::msg::Point point;

  // front corner cuts for orientation
  const double point_list[4][3] = {
    {length_half, width_half - tick_width, height_half},
    {length_half - tick_length, width_half, height_half},
    {length_half, -width_half + tick_width, height_half},
    {length_half - tick_length, -width_half, height_half},
  };
  const int point_pairs[2][2] = {
    {0, 1},
    {2, 3},
  };
  calc_line_list_from_points(point_list, point_pairs, 2, points);
}

void calc_2d_bounding_box_bottom_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  geometry_msgs::msg::Point point;

  // bounding box corner points
  // top surface, clockwise
  const double point_list[4][3] = {
    {length_half, width_half, -height_half},
    {length_half, -width_half, -height_half},
    {-length_half, -width_half, -height_half},
    {-length_half, width_half, -height_half},
  };
  const int point_pairs[4][2] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},
  };
  calc_line_list_from_points(point_list, point_pairs, 4, points);
}

void calc_2d_bounding_box_bottom_direction_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double triangle_size_half = std::min(width_half * 1.4, shape.dimensions.x);
  geometry_msgs::msg::Point point;

  // triangle-shaped direction indicator
  const double point_list[3][3] = {
    {length_half, 0, -height_half},
    {length_half - triangle_size_half, width_half, -height_half},
    {length_half - triangle_size_half, -width_half, -height_half},
  };
  const int point_pairs[3][2] = {
    {0, 1},
    {1, 2},
    {0, 2},
  };
  calc_line_list_from_points(point_list, point_pairs, 3, points);
}

void calc_2d_bounding_box_bottom_orientation_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double length_half = shape.dimensions.x * 0.5;
  const double width_half = shape.dimensions.y * 0.5;
  const double height_half = shape.dimensions.z * 0.5;
  const double tick_width = width_half * 0.5;
  const double tick_length = std::min(tick_width, length_half);
  geometry_msgs::msg::Point point;

  // front corner cuts for orientation
  const double point_list[4][3] = {
    {length_half, width_half - tick_width, -height_half},
    {length_half - tick_length, width_half, -height_half},
    {length_half, -width_half + tick_width, -height_half},
    {length_half - tick_length, -width_half, -height_half},
  };
  const int point_pairs[2][2] = {
    {0, 1},
    {2, 3},
  };
  calc_line_list_from_points(point_list, point_pairs, 2, points);
}

void calc_cylinder_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double radius = shape.dimensions.x * 0.5;
  {
    constexpr int n = 20;
    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = shape.dimensions.z * 0.5;
    calc_circle_line_list(center, radius, points, n);
    center.z = -shape.dimensions.z * 0.5;
    calc_circle_line_list(center, radius, points, n);
  }
  {
    constexpr int n = 4;
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::Point point;
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.z = shape.dimensions.z * 0.5;
      points.push_back(point);
      point.x = std::cos(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.y = std::sin(
                  (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                  M_PI / static_cast<double>(n)) *
                radius;
      point.z = -shape.dimensions.z * 0.5;
      points.push_back(point);
    }
  }
}

void calc_2d_cylinder_bottom_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const double radius = shape.dimensions.x * 0.5;
  {
    constexpr int n = 20;
    geometry_msgs::msg::Point center;
    center.x = 0.0;
    center.y = 0.0;
    center.z = -shape.dimensions.z * 0.5;
    calc_circle_line_list(center, radius, points, n);
  }
}

void calc_circle_line_list(
  const geometry_msgs::msg::Point center, const double radius,
  std::vector<geometry_msgs::msg::Point> & points, const int n)
{
  for (int i = 0; i < n; ++i) {
    geometry_msgs::msg::Point point;
    point.x = std::cos(
                (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.x;
    point.y = std::sin(
                (static_cast<double>(i) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.y;
    point.z = center.z;
    points.push_back(point);
    point.x = std::cos(
                (static_cast<double>(i + 1.0) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.x;
    point.y = std::sin(
                (static_cast<double>(i + 1.0) / static_cast<double>(n)) * 2.0 * M_PI +
                M_PI / static_cast<double>(n)) *
                radius +
              center.y;
    point.z = center.z;
    points.push_back(point);
  }
}

void calc_polygon_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const auto & footprint = shape.footprint.points;
  if (footprint.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName), "there are no enough footprint to visualize polygon");
    return;
  }
  const double top_z = shape.dimensions.z * 0.5;
  const double bottom_z = -shape.dimensions.z * 0.5;

  auto push_point = [&points](const double x, const double y, const double z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    points.push_back(point);
  };

  // top and bottom outlines
  for (const double z : {top_z, bottom_z}) {
    for (size_t i = 0; i < footprint.size(); ++i) {
      const auto & current = footprint.at(i);
      const auto & next = footprint.at((i + 1) % footprint.size());
      push_point(current.x, current.y, z);
      push_point(next.x, next.y, z);
    }
  }
  // vertical edges connecting the top and bottom outlines
  for (const auto & corner : footprint) {
    push_point(corner.x, corner.y, top_z);
    push_point(corner.x, corner.y, bottom_z);
  }
}

void calc_polygon_triangle_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const auto & footprint = shape.footprint.points;
  if (footprint.size() < 3) {
    return;
  }
  const double top_z = shape.dimensions.z * 0.5;
  const double bottom_z = -shape.dimensions.z * 0.5;

  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto & pt : footprint) {
    center_x += pt.x;
    center_y += pt.y;
  }
  center_x /= static_cast<double>(footprint.size());
  center_y /= static_cast<double>(footprint.size());

  auto push_point = [&points](const double x, const double y, const double z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    points.push_back(point);
  };

  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & curr = footprint.at(i);
    const auto & next = footprint.at((i + 1) % footprint.size());

    // Top face triangle
    push_point(center_x, center_y, top_z);
    push_point(curr.x, curr.y, top_z);
    push_point(next.x, next.y, top_z);

    // Bottom face triangle
    push_point(center_x, center_y, bottom_z);
    push_point(next.x, next.y, bottom_z);
    push_point(curr.x, curr.y, bottom_z);

    // Side quad (two triangles)
    push_point(curr.x, curr.y, bottom_z);
    push_point(next.x, next.y, bottom_z);
    push_point(next.x, next.y, top_z);

    push_point(curr.x, curr.y, bottom_z);
    push_point(next.x, next.y, top_z);
    push_point(curr.x, curr.y, top_z);
  }
}

void calc_2d_polygon_bottom_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::Point> & points)
{
  const auto & footprint = shape.footprint.points;
  if (footprint.size() < 2) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName), "there are no enough footprint to visualize polygon");
    return;
  }
  const double bottom_z = -shape.dimensions.z * 0.5;
  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & current = footprint.at(i);
    const auto & next = footprint.at((i + 1) % footprint.size());
    geometry_msgs::msg::Point point;
    point.z = bottom_z;
    point.x = current.x;
    point.y = current.y;
    points.push_back(point);
    point.x = next.x;
    point.y = next.y;
    points.push_back(point);
  }
}

void calc_path_line_list(
  const autoware_perception_msgs::msg::PredictedPath & paths,
  std::vector<geometry_msgs::msg::Point> & points, const bool is_simple)
{
  for (int i = 0; i < static_cast<int>(paths.path.size()) - 1; ++i) {
    // draw line
    geometry_msgs::msg::Point point;
    point.x = paths.path.at(i).position.x;
    point.y = paths.path.at(i).position.y;
    point.z = paths.path.at(i).position.z;
    points.push_back(point);
    point.x = paths.path.at(i + 1).position.x;
    point.y = paths.path.at(i + 1).position.y;
    point.z = paths.path.at(i + 1).position.z;
    points.push_back(point);

    if (!is_simple || i % 2 == 0) {
      // get yaw from the orientation if valid, otherwise from the line direction
      const auto & orientation = paths.path.at(i + 1).orientation;
      tf2::Quaternion tf_quat;
      tf2::fromMsg(orientation, tf_quat);
      const bool is_default_orientation =
        tf_quat.x() == 0.0 && tf_quat.y() == 0.0 && tf_quat.z() == 0.0 && tf_quat.w() == 1.0;
      const bool is_valid_orientation =
        (tf_quat.length() > std::numeric_limits<double>::epsilon()) && !is_default_orientation;
      const double yaw = is_valid_orientation
                           ? tf2::getYaw(tf_quat.normalize())
                           : std::atan2(
                               paths.path.at(i + 1).position.y - paths.path.at(i).position.y,
                               paths.path.at(i + 1).position.x - paths.path.at(i).position.x);

      // draw triangle
      constexpr double length = 0.5;
      const double arrow_angle = M_PI * 5.5 / 6.0;
      const double point_list[3][3] = {
        {point.x, point.y, point.z},
        {point.x + length * std::cos(yaw + arrow_angle),
         point.y + length * std::sin(yaw + arrow_angle), point.z},
        {point.x + length * std::cos(yaw - arrow_angle),
         point.y + length * std::sin(yaw - arrow_angle), point.z},
      };
      const int point_pairs[3][2] = {
        {0, 1},
        {1, 2},
        {2, 0},
      };
      calc_line_list_from_points(point_list, point_pairs, 3, points);
    }
  }
}

void calc_path_box_line_list(
  const autoware_perception_msgs::msg::Shape & shape,
  const autoware_perception_msgs::msg::PredictedPath & path,
  std::vector<geometry_msgs::msg::Point> & points, const bool is_simple)
{
  // object shape outline in the object-local frame, reused for every path step
  std::vector<geometry_msgs::msg::Point> shape_points;
  using autoware_perception_msgs::msg::Shape;
  if (shape.type == Shape::BOUNDING_BOX) {
    calc_bounding_box_line_list(shape, shape_points);
  } else if (shape.type == Shape::CYLINDER) {
    calc_cylinder_line_list(shape, shape_points);
  } else {  // including Shape::POLYGON
    calc_polygon_line_list(shape, shape_points);
  }

  const int step = is_simple ? 2 : 1;
  for (size_t i = 0; i < path.path.size(); i += step) {
    const auto & pose = path.path.at(i);

    // get yaw from the orientation if valid, otherwise from the path direction
    tf2::Quaternion tf_quat;
    tf2::fromMsg(pose.orientation, tf_quat);
    const bool is_default_orientation =
      tf_quat.x() == 0.0 && tf_quat.y() == 0.0 && tf_quat.z() == 0.0 && tf_quat.w() == 1.0;
    const bool is_valid_orientation =
      (tf_quat.length() > std::numeric_limits<double>::epsilon()) && !is_default_orientation;
    double yaw = 0.0;
    if (is_valid_orientation) {
      yaw = tf2::getYaw(tf_quat.normalize());
    } else if (i + 1 < path.path.size()) {
      yaw = std::atan2(
        path.path.at(i + 1).position.y - pose.position.y,
        path.path.at(i + 1).position.x - pose.position.x);
    } else if (i > 0) {
      yaw = std::atan2(
        pose.position.y - path.path.at(i - 1).position.y,
        pose.position.x - path.path.at(i - 1).position.x);
    }

    // transform the local shape outline to the path pose (yaw rotation + translation)
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (const auto & shape_point : shape_points) {
      geometry_msgs::msg::Point point;
      point.x = pose.position.x + shape_point.x * cos_yaw - shape_point.y * sin_yaw;
      point.y = pose.position.y + shape_point.x * sin_yaw + shape_point.y * cos_yaw;
      point.z = pose.position.z + shape_point.z;
      points.push_back(point);
    }
  }
}

}  // namespace detail
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware
