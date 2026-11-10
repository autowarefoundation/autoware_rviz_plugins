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

#include "superellipse.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using autoware_minimap_overlay_rviz_plugin::Superellipse;

namespace
{
const QRectF square(0.0, 0.0, 200.0, 200.0);
const Superellipse circle{square, 2.0};
const Superellipse squircle{square, 4.0};
const Superellipse rectangle{square, std::numeric_limits<double>::infinity()};
}  // namespace

TEST(Superellipse, NormIsOneOnTheOutline)
{
  for (const Superellipse & shape : {circle, squircle, rectangle}) {
    EXPECT_DOUBLE_EQ(shape.norm(QPointF(100.0, 100.0)), 0.0);
    EXPECT_NEAR(shape.norm(QPointF(200.0, 100.0)), 1.0, 1e-12);
    EXPECT_NEAR(shape.norm(QPointF(100.0, 0.0)), 1.0, 1e-12);
  }
}

TEST(Superellipse, SquircleIsBetweenTheCircleAndTheRectangle)
{
  // A point near the corner, on the diagonal
  const QPointF near_corner(175.0, 175.0);
  EXPECT_FALSE(circle.contains(near_corner));
  EXPECT_TRUE(squircle.contains(near_corner));
  EXPECT_TRUE(rectangle.contains(near_corner));

  const QPointF at_corner(195.0, 195.0);
  EXPECT_FALSE(squircle.contains(at_corner));
  EXPECT_TRUE(rectangle.contains(at_corner));
}

TEST(Superellipse, PointInsideIsNotMoved)
{
  const QPointF point(120.0, 90.0);
  EXPECT_EQ(squircle.clamp(point), point);
}

TEST(Superellipse, PointOutsideMovesToTheOutline)
{
  const QPointF far(1100.0, -900.0);

  const QPointF on_circle = circle.clamp(far);
  EXPECT_NEAR(on_circle.x() - 100.0, 100.0 / std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(on_circle.y() - 100.0, -100.0 / std::sqrt(2.0), 1e-9);

  const QPointF on_rectangle = rectangle.clamp(far);
  EXPECT_NEAR(on_rectangle.x(), 200.0, 1e-9);
  EXPECT_NEAR(on_rectangle.y(), 0.0, 1e-9);

  const QPointF on_squircle = squircle.clamp(far);
  EXPECT_NEAR(squircle.norm(on_squircle), 1.0, 1e-9);
  EXPECT_GT(on_squircle.x(), on_circle.x());
  EXPECT_LT(on_squircle.x(), on_rectangle.x());
}

TEST(Superellipse, WidthAboveTheBottom)
{
  // A circle has no width at the bottom and the full width at the middle
  EXPECT_NEAR(circle.halfWidthAboveBottom(0.0), 0.0, 1e-9);
  EXPECT_NEAR(circle.halfWidthAboveBottom(100.0), 100.0, 1e-9);
  EXPECT_NEAR(circle.halfWidthAboveBottom(20.0), 60.0, 1e-9);
  // A squircle is wider than a circle near the bottom
  EXPECT_GT(squircle.halfWidthAboveBottom(20.0), 85.0);
  // A rectangle has the full width at each height
  EXPECT_NEAR(rectangle.halfWidthAboveBottom(0.0), 100.0, 1e-9);
}

TEST(Superellipse, ShrunkShapeKeepsTheCenter)
{
  const Superellipse small = squircle.shrunk(10.0);
  EXPECT_EQ(small.rect, QRectF(10.0, 10.0, 180.0, 180.0));
  EXPECT_DOUBLE_EQ(small.exponent, 4.0);
}

TEST(Superellipse, PathFollowsTheOutline)
{
  EXPECT_TRUE(squircle.path().contains(QPointF(175.0, 175.0)));
  EXPECT_FALSE(squircle.path().contains(QPointF(195.0, 195.0)));
  EXPECT_FALSE(circle.path().contains(QPointF(175.0, 175.0)));
  EXPECT_TRUE(rectangle.path().contains(QPointF(195.0, 195.0)));
  // The path is inside the rectangle of the shape, and touches its sides
  const QRectF bounds = squircle.path().boundingRect();
  EXPECT_NEAR(bounds.left(), 0.0, 1e-6);
  EXPECT_NEAR(bounds.right(), 200.0, 1e-6);
  EXPECT_NEAR(bounds.top(), 0.0, 1e-6);
  EXPECT_NEAR(bounds.bottom(), 200.0, 1e-6);
}
