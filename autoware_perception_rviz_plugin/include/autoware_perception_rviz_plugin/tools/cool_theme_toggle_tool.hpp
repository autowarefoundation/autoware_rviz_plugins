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

#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TOOLS__COOL_THEME_TOGGLE_TOOL_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TOOLS__COOL_THEME_TOGGLE_TOOL_HPP_

#ifndef Q_MOC_RUN
#include <QIcon>
#include <QObject>
#include <QPointer>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <rviz_common/config.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/tool.hpp>

#include <utility>
#include <vector>
#endif

namespace autoware::rviz_plugins::tools
{

class CoolThemeToggleTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  CoolThemeToggleTool();
  ~CoolThemeToggleTool() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void load(const rviz_common::Config & config) override;

private:
  struct Rule
  {
    QStringList path;
    QVariant cool_value;
    QVariant default_value;
  };

  std::vector<Rule> buildRules() const;
  std::vector<rviz_common::properties::Property *> findProps(const QStringList & path) const;
  void applyTheme(bool enable_cool);
  void applyCameraPreset(bool enable_cool);
  void updateToolbarAppearance();
  static QIcon makeThemeIcon(bool cool_on);

  bool is_cool_mode_{false};
  bool has_saved_default_snapshot_{false};
  std::vector<std::pair<QPointer<rviz_common::properties::Property>, QVariant>>
    saved_default_snapshot_;

  rviz_common::properties::BoolProperty * cool_mode_active_property_{nullptr};
  rviz_common::properties::BoolProperty * switch_camera_property_{nullptr};
};

}  // namespace autoware::rviz_plugins::tools

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__TOOLS__COOL_THEME_TOGGLE_TOOL_HPP_
