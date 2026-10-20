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
#include "include/goal_pose.hpp"

#include "include/web_mercator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>

GoalPose::GoalPose() : goal_lat_(0.0), goal_lon_(0.0), vehicle_lat_(0.0), vehicle_lon_(0.0)
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_minimap_overlay_rviz_plugin");
  std::string goal_image_path = package_path + "/icons/goal.png";
  goal_image_ = QImage(goal_image_path.c_str());
  goal_image_ = goal_image_.scaled(20, 20, Qt::KeepAspectRatio);
}

void GoalPose::setGoalPosition(double lat, double lon)
{
  goal_lat_ = lat;
  goal_lon_ = lon;
  has_goal_ = true;
}

void GoalPose::clear()
{
  has_goal_ = false;
}

void GoalPose::draw(QPainter & painter, const QRectF & backgroundRect, int zoom) const
{
  if (!has_goal_) {
    return;
  }
  const QPointF goal_pixel = autoware_minimap_overlay_rviz_plugin::lat_lon_to_overlay_pixel(
    goal_lat_, goal_lon_, vehicle_lat_, vehicle_lon_, zoom, backgroundRect);

  // The tip of the pin is at the bottom center of the image
  painter.drawImage(
    goal_pixel - QPointF(goal_image_.width() / 2.0, goal_image_.height()), goal_image_);
}

// Getter methods for the goal latitude and longitude
double GoalPose::getGoalLatitude() const
{
  return goal_lat_;
}

double GoalPose::getGoalLongitude() const
{
  return goal_lon_;
}

void GoalPose::setVehiclePosition(double lat, double lon)
{
  vehicle_lat_ = lat;
  vehicle_lon_ = lon;
}
