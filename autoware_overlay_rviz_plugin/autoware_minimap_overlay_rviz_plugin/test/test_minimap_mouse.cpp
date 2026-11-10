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

#include <gtest/gtest.h>

using autoware_minimap_overlay_rviz_plugin::MinimapMouse;

TEST(MinimapMouse, ClickOnTheMinimapStaysWithTheMinimap)
{
  MinimapMouse mouse;
  EXPECT_TRUE(mouse.press(Qt::LeftButton, {50.0, 50.0}, true));
  // A movement of 2 pixels is still a click
  const auto motion = mouse.motion(Qt::LeftButton, {52.0, 50.0});
  EXPECT_TRUE(motion.consumed);
  EXPECT_EQ(motion.drag, QPointF());
  const auto release = mouse.release(Qt::LeftButton);
  EXPECT_TRUE(release.consumed);
  EXPECT_TRUE(release.click);
}

TEST(MinimapMouse, DragMovesTheMapAndIsNotAClick)
{
  for (const Qt::MouseButton button : {Qt::LeftButton, Qt::MiddleButton}) {
    MinimapMouse mouse;
    EXPECT_TRUE(mouse.press(button, {50.0, 50.0}, true));
    EXPECT_EQ(mouse.motion(button, {60.0, 50.0}).drag, QPointF(10.0, 0.0));
    // The drag goes on outside of the minimap
    const auto motion = mouse.motion(button, {-500.0, 45.0});
    EXPECT_TRUE(motion.consumed);
    EXPECT_EQ(motion.drag, QPointF(-560.0, -5.0));
    const auto release = mouse.release(button);
    EXPECT_TRUE(release.consumed);
    EXPECT_FALSE(release.click);
  }
}

TEST(MinimapMouse, MiddleButtonDoesNotClick)
{
  MinimapMouse mouse;
  mouse.press(Qt::MiddleButton, {50.0, 50.0}, true);
  EXPECT_FALSE(mouse.release(Qt::MiddleButton).click);
}

TEST(MinimapMouse, PressOutsideOfTheMinimapStaysWithRviz)
{
  // A tool of RViz gets the press, the moves and the release, also over the minimap
  MinimapMouse mouse;
  EXPECT_FALSE(mouse.press(Qt::LeftButton, {-200.0, 50.0}, false));
  EXPECT_FALSE(mouse.motion(Qt::LeftButton, {50.0, 50.0}).consumed);
  EXPECT_FALSE(mouse.release(Qt::LeftButton).consumed);
}

TEST(MinimapMouse, MoveWithNoButtonStaysWithRviz)
{
  MinimapMouse mouse;
  EXPECT_FALSE(mouse.motion(Qt::NoButton, {50.0, 50.0}).consumed);
}

TEST(MinimapMouse, RightButtonOnTheMinimapIsTakenButDoesNothing)
{
  MinimapMouse mouse;
  EXPECT_TRUE(mouse.press(Qt::RightButton, {50.0, 50.0}, true));
  EXPECT_FALSE(mouse.motion(Qt::RightButton, {90.0, 50.0}).consumed);
  const auto release = mouse.release(Qt::RightButton);
  EXPECT_TRUE(release.consumed);
  EXPECT_FALSE(release.click);
}

TEST(MinimapMouse, SecondButtonDoesNotEndTheDrag)
{
  MinimapMouse mouse;
  mouse.press(Qt::LeftButton, {50.0, 50.0}, true);
  // The right button goes down and up outside of the minimap during the drag
  EXPECT_FALSE(mouse.press(Qt::RightButton, {-200.0, 50.0}, false));
  EXPECT_FALSE(mouse.release(Qt::RightButton).consumed);
  EXPECT_EQ(mouse.motion(Qt::LeftButton, {70.0, 50.0}).drag, QPointF(20.0, 0.0));
  EXPECT_TRUE(mouse.release(Qt::LeftButton).consumed);
}

TEST(MinimapMouse, LostReleaseEndsTheDragAtTheNextMove)
{
  MinimapMouse mouse;
  mouse.press(Qt::LeftButton, {50.0, 50.0}, true);
  EXPECT_TRUE(mouse.motion(Qt::LeftButton, {70.0, 50.0}).consumed);
  // The release did not arrive: the next move shows that the button is up
  EXPECT_FALSE(mouse.motion(Qt::NoButton, {90.0, 50.0}).consumed);
  // A later release of a press outside of the minimap stays with RViz
  EXPECT_FALSE(mouse.release(Qt::LeftButton).consumed);
}

TEST(MinimapMouse, ResetForgetsTheButtons)
{
  MinimapMouse mouse;
  mouse.press(Qt::LeftButton, {50.0, 50.0}, true);
  mouse.reset();
  EXPECT_FALSE(mouse.motion(Qt::LeftButton, {90.0, 50.0}).consumed);
  EXPECT_FALSE(mouse.release(Qt::LeftButton).consumed);
}

TEST(MinimapMouse, WheelNotchIsOneZoomStep)
{
  MinimapMouse mouse;
  EXPECT_EQ(mouse.wheel(120), 1);
  EXPECT_EQ(mouse.wheel(-120), -1);
  EXPECT_EQ(mouse.wheel(240), 2);
}

TEST(MinimapMouse, SmallWheelAnglesAddUpToOneZoomStep)
{
  // A touchpad sends many events for one gesture
  MinimapMouse mouse;
  int steps = 0;
  for (int i = 0; i < 7; ++i) {
    steps += mouse.wheel(15);
  }
  EXPECT_EQ(steps, 0);
  EXPECT_EQ(mouse.wheel(15), 1);
  // A change of the direction starts from zero
  EXPECT_EQ(mouse.wheel(60), 0);
  EXPECT_EQ(mouse.wheel(-60), 0);
  EXPECT_EQ(mouse.wheel(-60), -1);
}
