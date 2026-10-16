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
#ifndef GOAL_POSE_HPP_
#define GOAL_POSE_HPP_

#include <QImage>
#include <QPainter>
#include <QRectF>

class GoalPose
{
public:
  GoalPose();
  void setGoalPosition(double lat, double lon);
  void clear();
  void setVehiclePosition(double lat, double lon);
  void draw(QPainter & painter, const QRectF & backgroundRect, int zoom) const;
  // Getter methods for the goal latitude and longitude
  double getGoalLatitude() const;
  double getGoalLongitude() const;

private:
  double goal_lat_;
  double goal_lon_;
  double vehicle_lat_;
  double vehicle_lon_;
  bool has_goal_ = false;
  QImage goal_image_;
};

#endif  // GOAL_POSE_HPP_
