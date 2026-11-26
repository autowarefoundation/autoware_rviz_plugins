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

  struct PathPt
  {
    double x;
    double y;
    double z;
    double s;
    double tx;
    double ty;
  };

  const double dt = static_cast<double>(predicted_path.time_step.sec) +
                    static_cast<double>(predicted_path.time_step.nanosec) * 1e-9;
  const double max_time = options.horizon_sec > 0.0 ? options.horizon_sec : 1e9;
  const double max_len = options.max_length > 0.0 ? options.max_length : 1e9;
  const double ground_z_offset = -static_cast<double>(shape.dimensions.z) * 0.5 + 0.05;

  std::vector<PathPt> pts;
  pts.reserve(predicted_path.path.size());

  const auto & p0 = predicted_path.path.front().position;
  pts.push_back({p0.x, p0.y, p0.z + ground_z_offset, 0.0, 1.0, 0.0});

  double prev_x = p0.x;
  double prev_y = p0.y;
  double prev_z = p0.z + ground_z_offset;
  double prev_time = 0.0;

  for (size_t idx = 1; idx < predicted_path.path.size(); ++idx) {
    const auto & pos = predicted_path.path.at(idx).position;
    const double cur_x = pos.x;
    const double cur_y = pos.y;
    const double cur_z = pos.z + ground_z_offset;
    const double cur_time = (dt > 1e-4) ? (static_cast<double>(idx) * dt) : 0.0;

    const double dx = cur_x - prev_x;
    const double dy = cur_y - prev_y;
    const double seg = std::hypot(dx, dy);

    double clip_ratio = 1.0;
    bool clipped = false;
    if (dt > 1e-4 && cur_time > max_time && cur_time > prev_time) {
      clip_ratio = std::min(clip_ratio, (max_time - prev_time) / (cur_time - prev_time));
      clipped = true;
    }
    if (seg > 1e-6 && pts.back().s + seg > max_len) {
      clip_ratio = std::min(clip_ratio, (max_len - pts.back().s) / seg);
      clipped = true;
    }
    clip_ratio = std::clamp(clip_ratio, 0.0, 1.0);

    const double tx = prev_x + dx * clip_ratio;
    const double ty = prev_y + dy * clip_ratio;
    const double tz = prev_z + (cur_z - prev_z) * clip_ratio;
    const double ds = std::hypot(tx - pts.back().x, ty - pts.back().y);
    if (ds >= 1e-3) {
      pts.push_back({tx, ty, tz, pts.back().s + ds, 1.0, 0.0});
    }

    prev_x = cur_x;
    prev_y = cur_y;
    prev_z = cur_z;
    prev_time = cur_time;
    if (clipped) {
      break;
    }
  }

  if (pts.size() < 2 || pts.back().s < 0.05) {
    return nullptr;
  }

  const size_t n_pts = pts.size();
  for (size_t i = 0; i < n_pts; ++i) {
    const size_t i0 = (i == 0) ? 0 : i - 1;
    const size_t i1 = (i + 1 == n_pts) ? n_pts - 1 : i + 1;
    const double dx = pts[i1].x - pts[i0].x;
    const double dy = pts[i1].y - pts[i0].y;
    const double norm = std::hypot(dx, dy);
    if (norm > 1e-6) {
      pts[i].tx = dx / norm;
      pts[i].ty = dy / norm;
    } else if (i > 0) {
      pts[i].tx = pts[i - 1].tx;
      pts[i].ty = pts[i - 1].ty;
    }
  }

  const double total_len = pts.back().s;
  const double base_half_w = std::max(0.04, options.width * 0.5);
  const float conf_scale = std::clamp(options.relative_confidence, 0.25F, 1.0F);
  const float peak_alpha = std::clamp(options.alpha * conf_scale, 0.05F, 1.0F);

  const float er = std::clamp(class_color.r * 0.65F + 0.35F, 0.0F, 1.0F);
  const float eg = std::clamp(class_color.g * 0.65F + 0.35F, 0.0F, 1.0F);
  const float eb = std::clamp(class_color.b * 0.65F + 0.35F, 0.0F, 1.0F);

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

  struct Section
  {
    geometry_msgs::msg::Point left;
    geometry_msgs::msg::Point center;
    geometry_msgs::msg::Point right;
    std_msgs::msg::ColorRGBA edge_col;
    std_msgs::msg::ColorRGBA center_col;
  };

  std::vector<Section> sections(n_pts);
  for (size_t i = 0; i < n_pts; ++i) {
    const double u = std::clamp(pts[i].s / total_len, 0.0, 1.0);
    const double half_w = base_half_w * (1.0 - 0.45 * u);
    const float a = static_cast<float>(1.0 - 0.85 * u) * peak_alpha;
    const double nx = -pts[i].ty;
    const double ny = pts[i].tx;

    sections[i].left = make_pt(pts[i].x + nx * half_w, pts[i].y + ny * half_w, pts[i].z);
    sections[i].center = make_pt(pts[i].x, pts[i].y, pts[i].z);
    sections[i].right = make_pt(pts[i].x - nx * half_w, pts[i].y - ny * half_w, pts[i].z);
    sections[i].edge_col = make_col(er, eg, eb, a);
    sections[i].center_col = make_col(class_color.r, class_color.g, class_color.b, a * 0.40F);
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
  marker_ptr->color = make_col(1.0F, 1.0F, 1.0F, 1.0F);
  marker_ptr->points.reserve((n_pts - 1) * 12);
  marker_ptr->colors.reserve((n_pts - 1) * 12);

  auto push_quad = [&](
                     const geometry_msgs::msg::Point & a0, const std_msgs::msg::ColorRGBA & ca0,
                     const geometry_msgs::msg::Point & b0, const std_msgs::msg::ColorRGBA & cb0,
                     const geometry_msgs::msg::Point & b1, const std_msgs::msg::ColorRGBA & cb1,
                     const geometry_msgs::msg::Point & a1, const std_msgs::msg::ColorRGBA & ca1) {
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

  for (size_t i = 0; i + 1 < n_pts; ++i) {
    const auto & s0 = sections[i];
    const auto & s1 = sections[i + 1];
    push_quad(
      s0.left, s0.edge_col, s0.center, s0.center_col, s1.center, s1.center_col, s1.left,
      s1.edge_col);
    push_quad(
      s0.center, s0.center_col, s0.right, s0.edge_col, s1.right, s1.edge_col, s1.center,
      s1.center_col);
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
  const auto & raw_footprint = shape.footprint.points;
  std::vector<Eigen::Vector2d> poly;
  poly.reserve(raw_footprint.size());
  for (const auto & pt : raw_footprint) {
    if (poly.empty() || std::hypot(pt.x - poly.back().x(), pt.y - poly.back().y()) > 1e-6) {
      poly.emplace_back(pt.x, pt.y);
    }
  }
  if (
    poly.size() >= 2 &&
    std::hypot(poly.front().x() - poly.back().x(), poly.front().y() - poly.back().y()) <= 1e-6) {
    poly.pop_back();
  }
  if (poly.size() < 3) {
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

  // Normalize winding to CCW for 2D ear-clipping triangulation (supports concave polygons)
  double signed_area2 = 0.0;
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto & curr = poly[i];
    const auto & next = poly[(i + 1) % poly.size()];
    signed_area2 += curr.x() * next.y() - next.x() * curr.y();
  }

  std::vector<size_t> remaining(poly.size());
  for (size_t i = 0; i < poly.size(); ++i) {
    remaining[i] = (signed_area2 >= 0.0) ? i : (poly.size() - 1 - i);
  }

  auto cross2d =
    [](const Eigen::Vector2d & a, const Eigen::Vector2d & b, const Eigen::Vector2d & c) {
      return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    };

  auto point_in_triangle = [&cross2d](
                             const Eigen::Vector2d & p, const Eigen::Vector2d & a,
                             const Eigen::Vector2d & b, const Eigen::Vector2d & c) {
    return cross2d(a, b, p) >= -1e-9 && cross2d(b, c, p) >= -1e-9 && cross2d(c, a, p) >= -1e-9;
  };

  std::vector<std::array<size_t, 3>> triangles;
  triangles.reserve(poly.size() - 2);

  while (remaining.size() > 3) {
    const size_t m = remaining.size();
    bool ear_found = false;
    for (size_t k = 0; k < m; ++k) {
      const size_t i_prev = remaining[(k + m - 1) % m];
      const size_t i_curr = remaining[k];
      const size_t i_next = remaining[(k + 1) % m];
      const auto & a = poly[i_prev];
      const auto & b = poly[i_curr];
      const auto & c = poly[i_next];

      if (cross2d(a, b, c) <= 1e-9) {
        continue;
      }

      bool contains_other = false;
      for (size_t j = 0; j < m; ++j) {
        const size_t i_other = remaining[j];
        if (i_other == i_prev || i_other == i_curr || i_other == i_next) {
          continue;
        }
        const auto & p = poly[i_other];
        if (
          (p - a).squaredNorm() <= 1e-12 || (p - b).squaredNorm() <= 1e-12 ||
          (p - c).squaredNorm() <= 1e-12) {
          continue;
        }
        if (point_in_triangle(p, a, b, c)) {
          contains_other = true;
          break;
        }
      }

      if (!contains_other) {
        triangles.push_back({i_prev, i_curr, i_next});
        remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(k));
        ear_found = true;
        break;
      }
    }

    if (!ear_found) {
      // Degenerate / self-intersecting fallback: clip the most convex remaining corner
      size_t best_k = 0;
      double best_cross = -1e30;
      for (size_t k = 0; k < m; ++k) {
        const double cr = cross2d(
          poly[remaining[(k + m - 1) % m]], poly[remaining[k]], poly[remaining[(k + 1) % m]]);
        if (cr > best_cross) {
          best_cross = cr;
          best_k = k;
        }
      }
      if (std::abs(best_cross) > 1e-12) {
        triangles.push_back(
          {remaining[(best_k + m - 1) % m], remaining[best_k], remaining[(best_k + 1) % m]});
      }
      remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best_k));
    }
  }
  if (remaining.size() == 3) {
    triangles.push_back({remaining[0], remaining[1], remaining[2]});
  }

  for (const auto & tri : triangles) {
    const auto & p0 = poly[tri[0]];
    const auto & p1 = poly[tri[1]];
    const auto & p2 = poly[tri[2]];

    // Top face triangle (+Z normal, CCW)
    push_point(p0.x(), p0.y(), top_z);
    push_point(p1.x(), p1.y(), top_z);
    push_point(p2.x(), p2.y(), top_z);

    // Bottom face triangle (-Z normal, CW from top)
    push_point(p0.x(), p0.y(), bottom_z);
    push_point(p2.x(), p2.y(), bottom_z);
    push_point(p1.x(), p1.y(), bottom_z);
  }

  for (size_t i = 0; i < poly.size(); ++i) {
    const auto & curr = poly[i];
    const auto & next = poly[(i + 1) % poly.size()];

    // Side quad (two triangles)
    push_point(curr.x(), curr.y(), bottom_z);
    push_point(next.x(), next.y(), bottom_z);
    push_point(next.x(), next.y(), top_z);

    push_point(curr.x(), curr.y(), bottom_z);
    push_point(next.x(), next.y(), top_z);
    push_point(curr.x(), curr.y(), top_z);
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
