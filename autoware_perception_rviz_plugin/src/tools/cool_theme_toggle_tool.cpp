// Copyright 2026 TIER IV, Inc.
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

#include "autoware_perception_rviz_plugin/tools/cool_theme_toggle_tool.hpp"

#include <QColor>
#include <QPainter>
#include <QPen>
#include <QPixmap>
#include <QPolygonF>
#include <QTimer>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

namespace autoware::rviz_plugins::tools
{

CoolThemeToggleTool::CoolThemeToggleTool()
{
  shortcut_key_ = 't';

  cool_mode_active_property_ = new rviz_common::properties::BoolProperty(
    "Cool Mode Active", false,
    "Whether the Cool (Cyberpunk) visualization preset is currently active.",
    getPropertyContainer());

  switch_camera_property_ = new rviz_common::properties::BoolProperty(
    "Switch Camera View", true,
    "Automatically switch between Cinematic Chase (Cool) and Default View when toggling.",
    getPropertyContainer());
}

void CoolThemeToggleTool::onInitialize()
{
  is_cool_mode_ = cool_mode_active_property_->getBool();
  setDescription(
    "One-click toggle between Default Autoware RViz style and Cool Hologram style (Shortcut: t)");
  setIcon(makeThemeIcon(is_cool_mode_));
  setName(is_cool_mode_ ? "Cool Mode: ON" : "Cool Mode: OFF");
}

void CoolThemeToggleTool::load(const rviz_common::Config & config)
{
  rviz_common::Tool::load(config);
  is_cool_mode_ = cool_mode_active_property_->getBool();
  setIcon(makeThemeIcon(is_cool_mode_));
  setName(is_cool_mode_ ? "Cool Mode: ON" : "Cool Mode: OFF");
  QTimer::singleShot(0, this, [this]() {
    if (is_cool_mode_) {
      for (const QStringList & ns_path :
           {QStringList{
              "Perception", "ObjectRecognition", "Tracking", "TrackedObjects", "Namespaces"},
            QStringList{
              "Perception", "ObjectRecognition", "Prediction", "PredictedObjects", "Namespaces"}}) {
        for (auto * ns_prop : findProps(ns_path)) {
          for (int i = 0; i < ns_prop->numChildren(); ++i) {
            if (auto * child = ns_prop->childAt(i)) {
              child->setValue(true);
            }
          }
        }
      }
      for (const auto & rule : buildRules()) {
        for (auto * prop : findProps(rule.path)) {
          prop->setValue(rule.cool_value);
        }
      }
    }
    if (context_ && context_->getToolManager()) {
      context_->getToolManager()->refreshTool(this);
    }
  });
}

void CoolThemeToggleTool::activate()
{
  is_cool_mode_ = !is_cool_mode_;
  cool_mode_active_property_->setValue(is_cool_mode_);
  applyTheme(is_cool_mode_);
  if (switch_camera_property_->getBool()) {
    applyCameraPreset(is_cool_mode_);
  }
  updateToolbarAppearance();

  if (context_) {
    context_->setStatus(
      is_cool_mode_
        ? "Switched to Cool Hologram Theme (press 't' or click button again for Default)"
        : "Switched to Default Autoware Theme (press 't' or click button again for Cool)");
    context_->queueRender();
  }

  QTimer::singleShot(0, this, [this]() { Q_EMIT close(); });
}

void CoolThemeToggleTool::deactivate()
{
}

QIcon CoolThemeToggleTool::makeThemeIcon(bool cool_on)
{
  QPixmap pix(32, 32);
  pix.fill(Qt::transparent);

  QPainter p(&pix);
  p.setRenderHint(QPainter::Antialiasing, true);

  const QColor edge_col = cool_on ? QColor(0, 225, 255) : QColor(120, 130, 145);
  const QColor accent_col = cool_on ? QColor(0, 235, 155) : QColor(150, 160, 175);
  const QColor fill_col = cool_on ? QColor(0, 195, 255, 85) : QColor(90, 100, 115, 35);

  QPolygonF top_face;
  top_face << QPointF(16, 4) << QPointF(27, 10) << QPointF(16, 16) << QPointF(5, 10);
  QPolygonF left_face;
  left_face << QPointF(5, 10) << QPointF(16, 16) << QPointF(16, 28) << QPointF(5, 22);
  QPolygonF right_face;
  right_face << QPointF(16, 16) << QPointF(27, 10) << QPointF(27, 22) << QPointF(16, 28);

  p.setPen(Qt::NoPen);
  p.setBrush(fill_col);
  p.drawPolygon(top_face);
  p.drawPolygon(left_face);
  p.drawPolygon(right_face);

  p.setBrush(Qt::NoBrush);
  p.setPen(QPen(edge_col, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  p.drawPolygon(top_face);
  p.drawPolygon(left_face);
  p.drawPolygon(right_face);

  // Draw a small trajectory ribbon accent underneath when ON
  if (cool_on) {
    p.setPen(QPen(accent_col, 2.4, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(4, 29), QPointF(28, 29));
  }

  p.end();
  return QIcon(pix);
}

void CoolThemeToggleTool::updateToolbarAppearance()
{
  setIcon(makeThemeIcon(is_cool_mode_));
  setName(is_cool_mode_ ? "Cool Mode: ON" : "Cool Mode: OFF");
  if (context_ && context_->getToolManager()) {
    context_->getToolManager()->refreshTool(this);
  }
}

std::vector<rviz_common::properties::Property *> CoolThemeToggleTool::findProps(
  const QStringList & path) const
{
  std::vector<rviz_common::properties::Property *> current;
  if (!context_ || path.isEmpty() || !context_->getRootDisplayGroup()) {
    return current;
  }
  current.push_back(context_->getRootDisplayGroup());

  for (const QString & seg : path) {
    std::vector<rviz_common::properties::Property *> next;
    for (auto * parent : current) {
      if (!parent) {
        continue;
      }
      for (int i = 0; i < parent->numChildren(); ++i) {
        if (auto * child = parent->childAt(i)) {
          if (child->getName() == seg) {
            next.push_back(child);
          }
        }
      }
    }
    if (next.empty()) {
      return {};
    }
    current = std::move(next);
  }
  return current;
}

std::vector<CoolThemeToggleTool::Rule> CoolThemeToggleTool::buildRules() const
{
  std::vector<Rule> rules = {
    // Global Options
    {{"Global Options", "Background Color"}, "0; 0; 0", "15; 20; 23"},
    {{"Global Options", "Frame Rate"}, 60, 30},

    // System / Vehicle
    {{"System", "Vehicle", "VehicleModel", "Alpha"}, 0.85, 0.3},
    {{"System", "Vehicle", "PolarGridDisplay"}, false, true},

    // Map (exclusive switch between standard MarkerArray and Cool custom plugin)
    {{"Map", "Lanelet2VectorMap"}, false, true},
    {{"Map", "Lanelet2VectorMap (Cool)"}, true, false},
    {{"Map", "Lanelet2VectorMap (Cool)", "Namespaces", "traffic_light"}, true, true},
    {{"Map", "Lanelet2VectorMap (Cool)", "Namespaces", "traffic_light_triangle"}, true, true},

    // Traffic Light recognized bulb size (snug inside 0.28m hologram lens ring in Cool mode)
    {{"Perception", "TrafficLight", "TrafficLight", "Bulb Radius"}, 0.30, 0.40},

    // PointClouds (Segmentation RGB)
    {{"Perception", "Segmentation", "NoGroundPointCloud", "Color"},
     "180; 230; 255",
     "200; 200; 200"},
    {{"Perception", "Segmentation", "NoGroundPointCloud", "Size (Pixels)"}, 1.5, 3.0},
    {{"Perception", "Segmentation", "SegmentedPointCloud (RGB)"}, true, false},
    {{"Perception", "Segmentation", "SegmentedPointCloud (Perception RGB)"}, true, false},

    // Detection BBox line widths
    {{"Perception", "ObjectRecognition", "Detection", "Centerpoint(red)", "Line Width"},
     0.025,
     0.03},
    {{"Perception", "ObjectRecognition", "Detection", "Stream PETR(yellow)", "Line Width"},
     0.025,
     0.03},
    {{"Perception", "ObjectRecognition", "Detection", "PTv3 Cluster(light_green)", "Line Width"},
     0.025,
     0.03},
    {{"Perception", "ObjectRecognition", "Detection", "Irregular object(pink)", "Line Width"},
     0.025,
     0.03},
    {{"Perception", "ObjectRecognition", "Detection", "PTv3 Object(magenta)", "Line Width"},
     0.025,
     0.03},
    {{"Perception", "ObjectRecognition", "Detection", "BEV Fusion(cyan)", "Line Width"},
     0.025,
     0.03},

    // Prediction enabled state
    {{"Perception", "ObjectRecognition", "Prediction", "PredictedObjects"}, true, false},

    // ScenarioTrajectory Hologram Ribbon
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Hologram Style"},
     true,
     false},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Alpha"}, 0.34, 0.999},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Center Alpha Ratio"},
     0.38,
     0.38},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Constant Width"},
     true,
     false},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Width"}, 1.5, 2.0},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Fade Out Distance"},
     18.0,
     0.0},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Z Offset"}, 0.03, 0.0},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Max Velocity Color"},
     "0; 230; 150",
     "0; 230; 120"},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Mid Velocity Color"},
     "0; 200; 165",
     "32; 138; 174"},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Path", "Min Velocity Color"},
     "20; 160; 170",
     "63; 46; 227"},
    {{"Planning", "ScenarioPlanning", "ScenarioTrajectory", "View Velocity"}, false, true},

    // Clutter / Debug overlays toggled OFF in Cool mode, ON in Default mode
    {{"Localization", "NDT", "MonteCarloInitialPose"}, false, true},
    {{"Localization", "EKF"}, false, true},
    {{"Perception", "ObjectRecognition", "MapFilteredPointCloud"}, false, true},
    {{"Perception", "TrafficLight", "MapBasedDetectionResult"}, false, true},
    {{"Planning", "MissionPlanning", "RouteArea"}, false, true},
    {{"Planning", "MissionPlanning", "MissionDetailsDisplay"}, false, true},
    {{"Planning", "ScenarioPlanning", "LaneDriving", "BehaviorPlanning", "Path"}, false, true},
    {{"Planning", "ScenarioPlanning", "Parking"}, false, true},
    {{"Planning", "ScenarioPlanning", "ModifiedGoal"}, false, true},
    {{"Planning", "NeuralNetworkBasedPlanner", "MLPlannerRouteMarker"}, false, true},
    {{"Planning", "NeuralNetworkBasedPlanner", "MLPlannerRoadBoarderMarker"}, false, true},
    {{"Planning", "NeuralNetworkBasedPlanner", "DiffusionPlannerRouteMarker"}, false, true},
    {{"Planning", "NeuralNetworkBasedPlanner", "DiffusionPlannerRoadBoarderMarker"}, false, true},
    {{"Planning", "NeuralNetworkBasedPlanner", "PredictedObjects"}, false, true},
    {{"Control", "Predicted Trajectory"}, false, true},
    {{"Debug", "Sensing", "PointcloudOnCamera"}, false, true},
    {{"TrajectoryValidatorProcessingTimeReport"}, false, true},
    {{"TrajectoryValidator(UncrossableBoundaryDepartureFilter)"}, false, true},
  };

  // Shared 3D Object BBox rules for TrackedObjects and PredictedObjects
  const std::vector<std::pair<QStringList, QString>> obj_targets = {
    {{"Perception", "ObjectRecognition", "Tracking", "TrackedObjects"}, "Skeleton 3D"},
    {{"Perception", "ObjectRecognition", "Prediction", "PredictedObjects"}, "Fill"},
  };

  for (const auto & [base, default_shape] : obj_targets) {
    auto p = [&](std::initializer_list<QString> tail) {
      QStringList full = base;
      for (const auto & t : tail) {
        full.append(t);
      }
      return full;
    };
    rules.push_back({p({"Alpha"}), 0.98, 0.999});
    rules.push_back({p({"Line Width"}), 0.035, 0.15});
    rules.push_back({p({"Shape", "Shape Type"}), "Hologram 3D", default_shape});
    rules.push_back({p({"Shape", "Fill Alpha"}), 0.36, 0.25});
    rules.push_back({p({"Text", "Label"}), false, true});
    rules.push_back({p({"Text", "UUID"}), false, true});
    rules.push_back({p({"Text", "Velocity"}), false, true});
    rules.push_back({p({"Text", "Acceleration"}), false, true});
    rules.push_back({p({"Vector", "Twist"}), false, true});
    rules.push_back({p({"Covariance", "Pose"}), false, true});
    rules.push_back({p({"Path", "Path Confidence"}), false, true});
    rules.push_back({p({"Per Class Color", "CAR"}), "0; 190; 255", "30; 144; 255"});
    rules.push_back({p({"Per Class Color", "BUS"}), "0; 220; 255", "30; 144; 255"});
    rules.push_back({p({"Per Class Color", "PEDESTRIAN"}), "255; 175; 0", "255; 192; 203"});
    rules.push_back({p({"Per Class Color", "CYCLIST"}), "255; 125; 0", "119; 11; 32"});
    rules.push_back({p({"Per Class Color", "MOTORCYCLE"}), "255; 125; 0", "119; 11; 32"});
    rules.push_back({p({"Per Class Color", "ANIMAL"}), "255; 200; 40", "255; 105; 180"});
    rules.push_back({p({"Per Class Color", "HAZARD"}), "255; 65; 65", "255; 69; 0"});
    rules.push_back({p({"Per Class Color", "UNKNOWN"}), "180; 200; 220", "255; 255; 255"});
  }

  return rules;
}

void CoolThemeToggleTool::applyTheme(bool enable_cool)
{
  const auto rules = buildRules();

  // Ensure MarkerArray namespace filters on Tracked/PredictedObjects never block toggled
  // sub-markers
  for (const QStringList & ns_path :
       {QStringList{"Perception", "ObjectRecognition", "Tracking", "TrackedObjects", "Namespaces"},
        QStringList{
          "Perception", "ObjectRecognition", "Prediction", "PredictedObjects", "Namespaces"}}) {
    for (auto * ns_prop : findProps(ns_path)) {
      for (int i = 0; i < ns_prop->numChildren(); ++i) {
        if (auto * child = ns_prop->childAt(i)) {
          child->setValue(true);
        }
      }
    }
  }

  if (enable_cool) {
    saved_default_snapshot_.clear();
    saved_default_snapshot_.reserve(rules.size());
    for (const auto & rule : rules) {
      for (auto * prop : findProps(rule.path)) {
        saved_default_snapshot_.emplace_back(prop, prop->getValue());
        prop->setValue(rule.cool_value);
      }
    }
    has_saved_default_snapshot_ = true;
  } else {
    if (has_saved_default_snapshot_) {
      for (const auto & [prop, val] : saved_default_snapshot_) {
        if (prop) {
          prop->setValue(val);
        }
      }
    } else {
      for (const auto & rule : rules) {
        for (auto * prop : findProps(rule.path)) {
          prop->setValue(rule.default_value);
        }
      }
    }
  }

  // Refresh SignalDisplay and TrafficLightDisplay subscriptions if enabled
  for (const QStringList & refresh_path :
       {QStringList{"System", "Vehicle", "SignalDisplay"},
        QStringList{"Perception", "TrafficLight", "TrafficLight"}}) {
    for (auto * prop : findProps(refresh_path)) {
      if (prop->getValue().toBool()) {
        prop->setValue(false);
        prop->setValue(true);
      }
    }
  }
}

void CoolThemeToggleTool::applyCameraPreset(bool enable_cool)
{
  if (!context_ || !context_->getViewManager()) {
    return;
  }
  rviz_common::ViewManager * vm = context_->getViewManager();
  const QStringList candidates =
    enable_cool ? QStringList{"Cinematic Chase (Cool)", "Cinematic Chase (Default)"}
                : QStringList{"Default View"};

  for (const QString & target_name : candidates) {
    for (int i = 0; i < vm->getNumViews(); ++i) {
      rviz_common::ViewController * v = vm->getViewAt(i);
      if (v && v->getName() == target_name) {
        vm->setCurrentFrom(v);
        return;
      }
    }
  }
}

}  // namespace autoware::rviz_plugins::tools

PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::tools::CoolThemeToggleTool, rviz_common::Tool)
