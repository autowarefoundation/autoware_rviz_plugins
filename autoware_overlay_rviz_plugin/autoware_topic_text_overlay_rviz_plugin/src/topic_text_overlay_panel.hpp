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

#ifndef TOPIC_TEXT_OVERLAY_PANEL_HPP_
#define TOPIC_TEXT_OVERLAY_PANEL_HPP_

#include <QColor>
#include <rclcpp/rclcpp.hpp>
#include <rviz_2d_overlay_plugins/overlay_utils.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

class QCheckBox;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QSpinBox;
class QTimer;

namespace autoware::topic_text_overlay_rviz_plugin
{
class TopicTextOverlayPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TopicTextOverlayPanel(QWidget * parent = nullptr);
  ~TopicTextOverlayPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void refresh_topics();
  void render_overlay();
  void handle_topic_item_changed(QListWidgetItem * item);
  void handle_namespace_item_changed(QListWidgetItem * item);
  void rebuild_namespace_list();

private:
  enum class TopicKind {
    AutowareStringStamped,
    Tier4StringStamped,
    BoolStamped,
    Float32Stamped,
    Float64Stamped,
    Int32Stamped,
    Int64Stamped,
    Float32MultiArrayStamped,
    Float64MultiArrayStamped,
    Int32MultiArrayStamped,
    Int64MultiArrayStamped,
    Marker,
    MarkerArray
  };

  struct TopicState
  {
    TopicKind kind;
    std::string type_name;
    bool selected{false};
    bool has_message{false};
    std::string text;
    QColor color;
    std::set<std::string> marker_namespaces;
    std::set<std::string> selected_marker_namespaces;
    std::map<std::string, std::string> marker_namespace_texts;
    rclcpp::SubscriptionBase::SharedPtr subscription;
  };

  void rebuild_topic_list();
  void sync_subscriptions();
  void set_topic_text(const std::string & topic, const std::string & text);
  void set_marker_array_text_and_namespaces(
    const std::string & topic, const visualization_msgs::msg::MarkerArray & msg);
  void subscribe_topic(const std::string & topic, TopicState & state);
  std::vector<std::pair<std::string, TopicState>> snapshot_selected_topics() const;

  static bool is_supported_topic_type(const std::string & type_name, TopicKind & kind);
  static std::string display_type_name(TopicKind kind);
  static QColor topic_color(size_t index);

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::Node::SharedPtr raw_node_;
  rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;

  QCheckBox * enable_overlay_;
  QListWidget * topic_list_;
  QListWidget * namespace_list_;
  QLabel * status_label_;
  QSpinBox * left_spin_;
  QSpinBox * top_spin_;
  QSpinBox * font_size_spin_;
  QSpinBox * max_letter_spin_;
  QTimer * refresh_timer_;
  QTimer * render_timer_;

  mutable std::mutex mutex_;
  std::map<std::string, TopicState> topics_;
  std::set<std::string> configured_selected_topics_;
  std::map<std::string, std::set<std::string>> configured_selected_marker_namespaces_;
  bool rebuilding_topic_list_{false};
  bool rebuilding_namespace_list_{false};
};
}  // namespace autoware::topic_text_overlay_rviz_plugin

#endif  // TOPIC_TEXT_OVERLAY_PANEL_HPP_
