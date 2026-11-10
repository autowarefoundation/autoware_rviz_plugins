// Copyright 2026 The Autoware Contributors
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

#include "minimap_mouse.hpp"

#include <cmath>

namespace autoware_minimap_overlay_rviz_plugin
{

namespace
{
// A mouse press that moves less than this distance is a click
constexpr double click_distance_px = 4.0;
// Angle of one notch of a mouse wheel, in the units of QWheelEvent::angleDelta()
constexpr int wheel_step_angle = 120;
}  // namespace

bool MinimapMouse::press(
  const Qt::MouseButton button, const QPointF & position, const bool on_minimap)
{
  if (!on_minimap) {
    return false;
  }
  taken_buttons_ |= button;
  // The left and the middle button move the map. Only the left button clicks a button.
  if (drag_button_ == Qt::NoButton && (button == Qt::LeftButton || button == Qt::MiddleButton)) {
    drag_button_ = button;
    dragging_ = false;
    press_position_ = position;
    last_position_ = position;
  }
  return true;
}

MinimapMouse::Motion MinimapMouse::motion(const Qt::MouseButtons buttons, const QPointF & position)
{
  // A release can get lost, for example when a dialog opens during a drag
  taken_buttons_ &= buttons;
  if (drag_button_ != Qt::NoButton && !(taken_buttons_ & drag_button_)) {
    drag_button_ = Qt::NoButton;
  }
  if (drag_button_ == Qt::NoButton) {
    return {};
  }

  const QPointF from_press = position - press_position_;
  if (std::hypot(from_press.x(), from_press.y()) > click_distance_px) {
    dragging_ = true;
  }
  Motion motion;
  motion.consumed = true;
  if (dragging_) {
    motion.drag = position - last_position_;
    last_position_ = position;
  }
  return motion;
}

MinimapMouse::Release MinimapMouse::release(const Qt::MouseButton button)
{
  if (!(taken_buttons_ & button)) {
    // The button went down outside of the minimap, so a tool of RViz waits for this release
    return {};
  }
  taken_buttons_ &= ~Qt::MouseButtons(button);

  Release release;
  release.consumed = true;
  if (button == drag_button_) {
    release.click = !dragging_ && button == Qt::LeftButton;
    drag_button_ = Qt::NoButton;
  }
  return release;
}

int MinimapMouse::wheel(const int angle_delta)
{
  if ((angle_delta > 0) != (wheel_angle_ > 0)) {
    // The direction changed
    wheel_angle_ = 0;
  }
  wheel_angle_ += angle_delta;
  const int steps = wheel_angle_ / wheel_step_angle;
  wheel_angle_ -= steps * wheel_step_angle;
  return steps;
}

void MinimapMouse::reset()
{
  *this = MinimapMouse();
}

}  // namespace autoware_minimap_overlay_rviz_plugin
