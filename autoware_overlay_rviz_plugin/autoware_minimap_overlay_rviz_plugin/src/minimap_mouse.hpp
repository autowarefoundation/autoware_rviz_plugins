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

#ifndef MINIMAP_MOUSE_HPP_
#define MINIMAP_MOUSE_HPP_

#include <QPointF>
#include <Qt>

namespace autoware_minimap_overlay_rviz_plugin
{

// Decides which mouse events of the RViz view belong to the minimap.
// The minimap takes a button that goes down on it, up to the release of that button. All other
// events stay with the tools of RViz, also when they occur over the minimap.
// The positions are in pixels of the minimap.
class MinimapMouse
{
public:
  struct Motion
  {
    // True if the tools of RViz must not get the event
    bool consumed = false;
    // Distance that the map moves with the mouse
    QPointF drag;
  };

  struct Release
  {
    bool consumed = false;
    // True if the left button went down and up at nearly the same position
    bool click = false;
  };

  // on_minimap tells if the position is inside the outline of the minimap.
  // Returns true if the tools of RViz must not get the event.
  bool press(Qt::MouseButton button, const QPointF & position, bool on_minimap);
  // `buttons` are the buttons that are down
  Motion motion(Qt::MouseButtons buttons, const QPointF & position);
  Release release(Qt::MouseButton button);
  // Number of zoom steps for a wheel event. One step is one notch of a mouse wheel. A touchpad
  // sends many events with small angles, which add up to a step.
  int wheel(int angle_delta);
  // Forgets all buttons, for a minimap that stops to handle the mouse
  void reset();

private:
  // Buttons that went down on the minimap and are not up again
  Qt::MouseButtons taken_buttons_ = Qt::NoButton;
  // Button that moves the map, or NoButton
  Qt::MouseButton drag_button_ = Qt::NoButton;
  bool dragging_ = false;
  QPointF press_position_;
  QPointF last_position_;
  int wheel_angle_ = 0;
};

}  // namespace autoware_minimap_overlay_rviz_plugin

#endif  // MINIMAP_MOUSE_HPP_
