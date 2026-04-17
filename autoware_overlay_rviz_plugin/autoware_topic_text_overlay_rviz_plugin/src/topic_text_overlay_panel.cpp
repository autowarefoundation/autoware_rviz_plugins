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

#include "topic_text_overlay_panel.hpp"

#include <QCheckBox>
#include <QFont>
#include <QFontMetrics>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMetaObject>
#include <QPainter>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int64_multi_array_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int64_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace autoware::topic_text_overlay_rviz_plugin
{
namespace
{
constexpr const char * autoware_string_stamped_type =
  "autoware_internal_debug_msgs/msg/StringStamped";
constexpr const char * bool_stamped_type = "autoware_internal_debug_msgs/msg/BoolStamped";
constexpr const char * float32_stamped_type = "autoware_internal_debug_msgs/msg/Float32Stamped";
constexpr const char * float64_stamped_type = "autoware_internal_debug_msgs/msg/Float64Stamped";
constexpr const char * int32_stamped_type = "autoware_internal_debug_msgs/msg/Int32Stamped";
constexpr const char * int64_stamped_type = "autoware_internal_debug_msgs/msg/Int64Stamped";
constexpr const char * float32_multi_array_stamped_type =
  "autoware_internal_debug_msgs/msg/Float32MultiArrayStamped";
constexpr const char * float64_multi_array_stamped_type =
  "autoware_internal_debug_msgs/msg/Float64MultiArrayStamped";
constexpr const char * int32_multi_array_stamped_type =
  "autoware_internal_debug_msgs/msg/Int32MultiArrayStamped";
constexpr const char * int64_multi_array_stamped_type =
  "autoware_internal_debug_msgs/msg/Int64MultiArrayStamped";
constexpr const char * tier4_string_stamped_type = "tier4_debug_msgs/msg/StringStamped";
constexpr const char * marker_type = "visualization_msgs/msg/Marker";
constexpr const char * marker_array_type = "visualization_msgs/msg/MarkerArray";
constexpr int namespace_role = Qt::UserRole + 1;

std::string truncateText(const std::string & text, const int max_letters)
{
  if (max_letters <= 0 || static_cast<int>(text.size()) <= max_letters) {
    return text;
  }
  return text.substr(0, static_cast<size_t>(max_letters)) + "...";
}

size_t countLines(const std::string & text)
{
  return static_cast<size_t>(std::count(text.begin(), text.end(), '\n')) + 1U;
}

template <typename T>
std::string numberToString(const T value)
{
  std::ostringstream stream;
  stream << std::setprecision(6) << value;
  return stream.str();
}

template <typename Container>
std::string arrayToString(const Container & values)
{
  std::ostringstream stream;
  stream << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0U) {
      stream << ", ";
    }
    stream << std::setprecision(6) << values.at(i);
  }
  stream << "]";
  return stream.str();
}

std::string markerText(const visualization_msgs::msg::Marker & marker)
{
  if (
    marker.action == visualization_msgs::msg::Marker::DELETE ||
    marker.action == visualization_msgs::msg::Marker::DELETEALL ||
    marker.type != visualization_msgs::msg::Marker::TEXT_VIEW_FACING) {
    return {};
  }
  return marker.text;
}

std::string composeMarkerArrayText(
  const std::map<std::string, std::string> & namespace_texts,
  const std::set<std::string> & selected_namespaces)
{
  std::string text;
  for (const auto & [marker_namespace, namespace_text] : namespace_texts) {
    if (selected_namespaces.count(marker_namespace) == 0U || namespace_text.empty()) {
      continue;
    }
    if (!text.empty()) {
      text += "\n";
    }
    text += namespace_text;
  }
  return text;
}
}  // namespace

TopicTextOverlayPanel::TopicTextOverlayPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  enable_overlay_ = new QCheckBox("Enable overlay", this);
  enable_overlay_->setChecked(true);

  auto * refresh_button = new QPushButton("Refresh topics", this);
  topic_list_ = new QListWidget(this);
  topic_list_->setSelectionMode(QAbstractItemView::NoSelection);
  namespace_list_ = new QListWidget(this);
  namespace_list_->setSelectionMode(QAbstractItemView::NoSelection);

  left_spin_ = new QSpinBox(this);
  left_spin_->setRange(0, 10000);
  left_spin_->setValue(40);
  top_spin_ = new QSpinBox(this);
  top_spin_->setRange(0, 10000);
  top_spin_->setValue(80);
  font_size_spin_ = new QSpinBox(this);
  font_size_spin_->setRange(6, 96);
  font_size_spin_->setValue(16);
  max_letter_spin_ = new QSpinBox(this);
  max_letter_spin_->setRange(20, 5000);
  max_letter_spin_->setValue(1000);
  status_label_ = new QLabel("Waiting for RViz context", this);

  auto * position_layout = new QFormLayout;
  position_layout->addRow("Left", left_spin_);
  position_layout->addRow("Top", top_spin_);
  position_layout->addRow("Font size", font_size_spin_);
  position_layout->addRow("Max letters", max_letter_spin_);

  auto * top_layout = new QHBoxLayout;
  top_layout->addWidget(enable_overlay_);
  top_layout->addWidget(refresh_button);

  auto * layout = new QVBoxLayout(this);
  layout->addLayout(top_layout);
  layout->addWidget(new QLabel("Selectable topics", this));
  layout->addWidget(topic_list_);
  layout->addWidget(new QLabel("MarkerArray namespaces", this));
  layout->addWidget(namespace_list_);
  layout->addLayout(position_layout);
  layout->addWidget(status_label_);
  setLayout(layout);

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(1000);
  render_timer_ = new QTimer(this);
  render_timer_->setInterval(100);

  connect(refresh_button, &QPushButton::clicked, this, &TopicTextOverlayPanel::refreshTopics);
  connect(refresh_timer_, &QTimer::timeout, this, &TopicTextOverlayPanel::refreshTopics);
  connect(render_timer_, &QTimer::timeout, this, &TopicTextOverlayPanel::renderOverlay);
  connect(
    topic_list_, &QListWidget::itemChanged, this, &TopicTextOverlayPanel::handleTopicItemChanged);
  connect(
    namespace_list_, &QListWidget::itemChanged, this,
    &TopicTextOverlayPanel::handleNamespaceItemChanged);
  connect(enable_overlay_, &QCheckBox::stateChanged, this, &TopicTextOverlayPanel::renderOverlay);
  connect(
    left_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
    &TopicTextOverlayPanel::renderOverlay);
  connect(
    top_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
    &TopicTextOverlayPanel::renderOverlay);
  connect(
    font_size_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
    &TopicTextOverlayPanel::renderOverlay);
  connect(
    max_letter_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
    &TopicTextOverlayPanel::renderOverlay);

  connect(enable_overlay_, &QCheckBox::stateChanged, this, [this](int) { Q_EMIT configChanged(); });
  connect(left_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
    Q_EMIT configChanged();
  });
  connect(top_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
    Q_EMIT configChanged();
  });
  connect(font_size_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
    Q_EMIT configChanged();
  });
  connect(max_letter_spin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
    Q_EMIT configChanged();
  });
}

TopicTextOverlayPanel::~TopicTextOverlayPanel()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void TopicTextOverlayPanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  raw_node_ = rviz_ros_node_.lock()->get_raw_node();

  static int count = 0;
  std::ostringstream overlay_name;
  overlay_name << "TopicTextOverlayPanelObject" << count++;
  overlay_ = std::make_shared<OverlayObject>(
    getDisplayContext()->getSceneManager(), raw_node_->get_logger(), overlay_name.str());
  overlay_->show();

  refreshTopics();
  renderOverlay();
  refresh_timer_->start();
  render_timer_->start();
}

void TopicTextOverlayPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  bool enable_overlay = true;
  if (config.mapGetBool("Enable Overlay", &enable_overlay)) {
    enable_overlay_->setChecked(enable_overlay);
  }

  int left = 0;
  if (config.mapGetInt("Left", &left)) {
    left_spin_->setValue(left);
  }
  int top = 0;
  if (config.mapGetInt("Top", &top)) {
    top_spin_->setValue(top);
  }
  int font_size = 0;
  if (config.mapGetInt("Font Size", &font_size)) {
    font_size_spin_->setValue(font_size);
  }
  int max_letters = 0;
  if (config.mapGetInt("Max Letters", &max_letters)) {
    max_letter_spin_->setValue(max_letters);
  }

  std::set<std::string> selected_topics;
  const auto topics_config = config.mapGetChild("Selected Topics");
  for (int i = 0; i < topics_config.listLength(); ++i) {
    const auto topic = topics_config.listChildAt(i).getValue().toString().toStdString();
    if (!topic.empty()) {
      selected_topics.insert(topic);
    }
  }
  std::map<std::string, std::set<std::string>> selected_marker_namespaces;
  const auto namespaces_config = config.mapGetChild("Selected Marker Namespaces");
  for (int i = 0; i < namespaces_config.listLength(); ++i) {
    const auto namespace_config = namespaces_config.listChildAt(i);
    QString topic;
    QString marker_namespace;
    if (namespace_config.mapGetString("Topic", &topic)) {
      const auto topic_name = topic.toStdString();
      selected_marker_namespaces.try_emplace(topic_name);
      if (namespace_config.mapGetString("Namespace", &marker_namespace)) {
        selected_marker_namespaces[topic_name].insert(marker_namespace.toStdString());
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    configured_selected_topics_ = std::move(selected_topics);
    configured_selected_marker_namespaces_ = std::move(selected_marker_namespaces);
    for (auto & [topic, state] : topics_) {
      state.selected = configured_selected_topics_.count(topic) != 0U;
      if (const auto it = configured_selected_marker_namespaces_.find(topic);
          it != configured_selected_marker_namespaces_.end()) {
        state.selected_marker_namespaces = it->second;
      }
    }
  }

  refreshTopics();
}

void TopicTextOverlayPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  config.mapSetValue("Enable Overlay", enable_overlay_->isChecked());
  config.mapSetValue("Left", left_spin_->value());
  config.mapSetValue("Top", top_spin_->value());
  config.mapSetValue("Font Size", font_size_spin_->value());
  config.mapSetValue("Max Letters", max_letter_spin_->value());

  std::set<std::string> selected_topics;
  std::map<std::string, std::set<std::string>> selected_marker_namespaces;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    selected_topics = configured_selected_topics_;
    selected_marker_namespaces = configured_selected_marker_namespaces_;
    for (const auto & [topic, state] : topics_) {
      if (state.selected) {
        selected_topics.insert(topic);
      } else {
        selected_topics.erase(topic);
      }
      if (state.kind == TopicKind::MarkerArray) {
        selected_marker_namespaces[topic] = state.selected_marker_namespaces;
      }
    }
  }

  auto topics_config = config.mapMakeChild("Selected Topics");
  for (const auto & topic : selected_topics) {
    topics_config.listAppendNew().setValue(QString::fromStdString(topic));
  }

  auto namespaces_config = config.mapMakeChild("Selected Marker Namespaces");
  for (const auto & [topic, namespaces] : selected_marker_namespaces) {
    if (namespaces.empty()) {
      auto namespace_config = namespaces_config.listAppendNew();
      namespace_config.mapSetValue("Topic", QString::fromStdString(topic));
      continue;
    }
    for (const auto & marker_namespace : namespaces) {
      auto namespace_config = namespaces_config.listAppendNew();
      namespace_config.mapSetValue("Topic", QString::fromStdString(topic));
      namespace_config.mapSetValue("Namespace", QString::fromStdString(marker_namespace));
    }
  }
}

bool TopicTextOverlayPanel::isSupportedTopicType(const std::string & type_name, TopicKind & kind)
{
  if (type_name == autoware_string_stamped_type) {
    kind = TopicKind::AutowareStringStamped;
    return true;
  }
  if (type_name == tier4_string_stamped_type) {
    kind = TopicKind::Tier4StringStamped;
    return true;
  }
  if (type_name == bool_stamped_type) {
    kind = TopicKind::BoolStamped;
    return true;
  }
  if (type_name == float32_stamped_type) {
    kind = TopicKind::Float32Stamped;
    return true;
  }
  if (type_name == float64_stamped_type) {
    kind = TopicKind::Float64Stamped;
    return true;
  }
  if (type_name == int32_stamped_type) {
    kind = TopicKind::Int32Stamped;
    return true;
  }
  if (type_name == int64_stamped_type) {
    kind = TopicKind::Int64Stamped;
    return true;
  }
  if (type_name == float32_multi_array_stamped_type) {
    kind = TopicKind::Float32MultiArrayStamped;
    return true;
  }
  if (type_name == float64_multi_array_stamped_type) {
    kind = TopicKind::Float64MultiArrayStamped;
    return true;
  }
  if (type_name == int32_multi_array_stamped_type) {
    kind = TopicKind::Int32MultiArrayStamped;
    return true;
  }
  if (type_name == int64_multi_array_stamped_type) {
    kind = TopicKind::Int64MultiArrayStamped;
    return true;
  }
  if (type_name == marker_type) {
    kind = TopicKind::Marker;
    return true;
  }
  if (type_name == marker_array_type) {
    kind = TopicKind::MarkerArray;
    return true;
  }
  return false;
}

std::string TopicTextOverlayPanel::displayTypeName(const TopicKind kind)
{
  switch (kind) {
    case TopicKind::AutowareStringStamped:
      return "StringStamped";
    case TopicKind::Tier4StringStamped:
      return "StringStamped(tier4)";
    case TopicKind::BoolStamped:
      return "BoolStamped";
    case TopicKind::Float32Stamped:
      return "Float32Stamped";
    case TopicKind::Float64Stamped:
      return "Float64Stamped";
    case TopicKind::Int32Stamped:
      return "Int32Stamped";
    case TopicKind::Int64Stamped:
      return "Int64Stamped";
    case TopicKind::Float32MultiArrayStamped:
      return "Float32MultiArrayStamped";
    case TopicKind::Float64MultiArrayStamped:
      return "Float64MultiArrayStamped";
    case TopicKind::Int32MultiArrayStamped:
      return "Int32MultiArrayStamped";
    case TopicKind::Int64MultiArrayStamped:
      return "Int64MultiArrayStamped";
    case TopicKind::Marker:
      return "Marker TEXT_VIEW_FACING only";
    case TopicKind::MarkerArray:
      return "MarkerArray TEXT_VIEW_FACING only";
  }
  return "Unknown";
}

QColor TopicTextOverlayPanel::topicColor(const size_t index)
{
  static const std::vector<QColor> colors = {
    QColor(0, 220, 255),   QColor(255, 180, 0),   QColor(120, 255, 120), QColor(255, 110, 180),
    QColor(180, 150, 255), QColor(255, 255, 120), QColor(120, 180, 255), QColor(255, 140, 90)};
  return colors.at(index % colors.size());
}

void TopicTextOverlayPanel::refreshTopics()
{
  if (!raw_node_) {
    return;
  }

  std::set<std::string> configured_selected_topics;
  std::map<std::string, std::set<std::string>> configured_selected_marker_namespaces;
  std::map<std::string, TopicState> existing_topics;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    configured_selected_topics = configured_selected_topics_;
    configured_selected_marker_namespaces = configured_selected_marker_namespaces_;
    existing_topics = topics_;
  }

  const auto topic_names_and_types = raw_node_->get_topic_names_and_types();
  std::map<std::string, TopicState> refreshed_topics;
  size_t color_index = 0;
  for (const auto & [topic_name, type_names] : topic_names_and_types) {
    for (const auto & type_name : type_names) {
      TopicKind kind;
      if (!isSupportedTopicType(type_name, kind)) {
        continue;
      }
      auto state_it = existing_topics.find(topic_name);
      TopicState state = state_it != existing_topics.end() ? state_it->second : TopicState{};
      state.kind = kind;
      state.type_name = type_name;
      if (configured_selected_topics.count(topic_name) != 0U) {
        state.selected = true;
      }
      if (const auto namespaces_it = configured_selected_marker_namespaces.find(topic_name);
          namespaces_it != configured_selected_marker_namespaces.end()) {
        state.selected_marker_namespaces = namespaces_it->second;
      }
      if (!state.color.isValid()) {
        state.color = topicColor(color_index);
      }
      refreshed_topics.emplace(topic_name, std::move(state));
      break;
    }
    ++color_index;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    topics_ = std::move(refreshed_topics);
  }

  rebuildTopicList();
  rebuildNamespaceList();
  syncSubscriptions();
  status_label_->setText(QString("Found %1 supported topics").arg(topic_list_->count()));
  renderOverlay();
}

void TopicTextOverlayPanel::rebuildTopicList()
{
  rebuilding_topic_list_ = true;
  topic_list_->clear();

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [topic, state] : topics_) {
    const auto label = QString::fromStdString(topic + "  [" + displayTypeName(state.kind) + "]");
    auto * item = new QListWidgetItem(label, topic_list_);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(state.selected ? Qt::Checked : Qt::Unchecked);
    item->setData(Qt::UserRole, QString::fromStdString(topic));
    item->setForeground(state.color);
  }

  rebuilding_topic_list_ = false;
}

void TopicTextOverlayPanel::rebuildNamespaceList()
{
  rebuilding_namespace_list_ = true;
  namespace_list_->clear();

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [topic, state] : topics_) {
    if (state.kind != TopicKind::MarkerArray || !state.selected) {
      continue;
    }
    for (const auto & marker_namespace : state.marker_namespaces) {
      const auto label = QString::fromStdString(topic + "  [ns: " + marker_namespace + "]");
      auto * item = new QListWidgetItem(label, namespace_list_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(
        state.selected_marker_namespaces.count(marker_namespace) != 0U ? Qt::Checked
                                                                       : Qt::Unchecked);
      item->setData(Qt::UserRole, QString::fromStdString(topic));
      item->setData(namespace_role, QString::fromStdString(marker_namespace));
      item->setForeground(QColor(0, 255, 177));
    }
  }

  rebuilding_namespace_list_ = false;
}

void TopicTextOverlayPanel::handleTopicItemChanged(QListWidgetItem * item)
{
  if (rebuilding_topic_list_) {
    return;
  }

  const auto topic = item->data(Qt::UserRole).toString().toStdString();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      return;
    }
    it->second.selected = item->checkState() == Qt::Checked;
    if (it->second.selected) {
      configured_selected_topics_.insert(topic);
    } else {
      configured_selected_topics_.erase(topic);
    }
    if (!it->second.selected) {
      it->second.subscription.reset();
      it->second.has_message = false;
      it->second.text.clear();
    }
  }
  syncSubscriptions();
  rebuildNamespaceList();
  Q_EMIT configChanged();
  renderOverlay();
}

void TopicTextOverlayPanel::handleNamespaceItemChanged(QListWidgetItem * item)
{
  if (rebuilding_namespace_list_) {
    return;
  }

  const auto topic = item->data(Qt::UserRole).toString().toStdString();
  const auto marker_namespace = item->data(namespace_role).toString().toStdString();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      return;
    }
    if (item->checkState() == Qt::Checked) {
      it->second.selected_marker_namespaces.insert(marker_namespace);
    } else {
      it->second.selected_marker_namespaces.erase(marker_namespace);
    }
    it->second.text = composeMarkerArrayText(
      it->second.marker_namespace_texts, it->second.selected_marker_namespaces);
    it->second.has_message = true;
    configured_selected_marker_namespaces_[topic] = it->second.selected_marker_namespaces;
  }

  Q_EMIT configChanged();
  renderOverlay();
}

void TopicTextOverlayPanel::syncSubscriptions()
{
  std::vector<std::pair<std::string, TopicKind>> topics_to_subscribe;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [topic, state] : topics_) {
      if (!state.selected) {
        state.subscription.reset();
        continue;
      }
      if (!state.subscription) {
        topics_to_subscribe.emplace_back(topic, state.kind);
      }
    }
  }

  for (const auto & [topic, kind] : topics_to_subscribe) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    if (
      it != topics_.end() && it->second.selected && !it->second.subscription &&
      it->second.kind == kind) {
      subscribeTopic(topic, it->second);
    }
  }
}

void TopicTextOverlayPanel::subscribeTopic(const std::string & topic, TopicState & state)
{
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  switch (state.kind) {
    case TopicKind::AutowareStringStamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::StringStamped>(
          topic, qos,
          [this,
           topic](const autoware_internal_debug_msgs::msg::StringStamped::ConstSharedPtr msg) {
            setTopicText(topic, msg->data);
          });
      break;
    case TopicKind::Tier4StringStamped:
      state.subscription = raw_node_->create_subscription<tier4_debug_msgs::msg::StringStamped>(
        topic, qos, [this, topic](const tier4_debug_msgs::msg::StringStamped::ConstSharedPtr msg) {
          setTopicText(topic, msg->data);
        });
      break;
    case TopicKind::BoolStamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::BoolStamped>(
          topic, qos,
          [this, topic](const autoware_internal_debug_msgs::msg::BoolStamped::ConstSharedPtr msg) {
            setTopicText(topic, msg->data ? "true" : "false");
          });
      break;
    case TopicKind::Float32Stamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
          topic, qos,
          [this,
           topic](const autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr msg) {
            setTopicText(topic, numberToString(msg->data));
          });
      break;
    case TopicKind::Float64Stamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Float64Stamped>(
          topic, qos,
          [this,
           topic](const autoware_internal_debug_msgs::msg::Float64Stamped::ConstSharedPtr msg) {
            setTopicText(topic, numberToString(msg->data));
          });
      break;
    case TopicKind::Int32Stamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Int32Stamped>(
          topic, qos,
          [this, topic](const autoware_internal_debug_msgs::msg::Int32Stamped::ConstSharedPtr msg) {
            setTopicText(topic, std::to_string(msg->data));
          });
      break;
    case TopicKind::Int64Stamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Int64Stamped>(
          topic, qos,
          [this, topic](const autoware_internal_debug_msgs::msg::Int64Stamped::ConstSharedPtr msg) {
            setTopicText(topic, std::to_string(msg->data));
          });
      break;
    case TopicKind::Float32MultiArrayStamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>(
          topic, qos,
          [this, topic](
            const autoware_internal_debug_msgs::msg::Float32MultiArrayStamped::ConstSharedPtr msg) {
            setTopicText(topic, arrayToString(msg->data));
          });
      break;
    case TopicKind::Float64MultiArrayStamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>(
          topic, qos,
          [this, topic](
            const autoware_internal_debug_msgs::msg::Float64MultiArrayStamped::ConstSharedPtr msg) {
            setTopicText(topic, arrayToString(msg->data));
          });
      break;
    case TopicKind::Int32MultiArrayStamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Int32MultiArrayStamped>(
          topic, qos,
          [this, topic](
            const autoware_internal_debug_msgs::msg::Int32MultiArrayStamped::ConstSharedPtr msg) {
            setTopicText(topic, arrayToString(msg->data));
          });
      break;
    case TopicKind::Int64MultiArrayStamped:
      state.subscription =
        raw_node_->create_subscription<autoware_internal_debug_msgs::msg::Int64MultiArrayStamped>(
          topic, qos,
          [this, topic](
            const autoware_internal_debug_msgs::msg::Int64MultiArrayStamped::ConstSharedPtr msg) {
            setTopicText(topic, arrayToString(msg->data));
          });
      break;
    case TopicKind::Marker:
      state.subscription = raw_node_->create_subscription<visualization_msgs::msg::Marker>(
        topic, qos, [this, topic](const visualization_msgs::msg::Marker::ConstSharedPtr msg) {
          setTopicText(topic, markerText(*msg));
        });
      break;
    case TopicKind::MarkerArray:
      state.subscription = raw_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        topic, qos, [this, topic](const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          setMarkerArrayTextAndNamespaces(topic, *msg);
        });
      break;
  }
}

void TopicTextOverlayPanel::setMarkerArrayTextAndNamespaces(
  const std::string & topic, const visualization_msgs::msg::MarkerArray & msg)
{
  std::set<std::string> namespaces;
  std::map<std::string, std::string> namespace_texts;
  for (const auto & marker : msg.markers) {
    const auto text = markerText(marker);
    if (!text.empty()) {
      namespaces.insert(marker.ns);
      auto & namespace_text = namespace_texts[marker.ns];
      if (!namespace_text.empty()) {
        namespace_text += "\n";
      }
      namespace_text += text;
    }
  }

  bool namespaces_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      return;
    }
    for (const auto & marker_namespace : namespaces) {
      if (it->second.marker_namespaces.insert(marker_namespace).second) {
        namespaces_changed = true;
        if (configured_selected_marker_namespaces_.count(topic) == 0U) {
          it->second.selected_marker_namespaces.insert(marker_namespace);
        }
      }
    }
    it->second.marker_namespace_texts = std::move(namespace_texts);
    it->second.text = composeMarkerArrayText(
      it->second.marker_namespace_texts, it->second.selected_marker_namespaces);
    it->second.has_message = true;
  }

  if (namespaces_changed) {
    QMetaObject::invokeMethod(this, "rebuildNamespaceList", Qt::QueuedConnection);
  }
  QMetaObject::invokeMethod(this, "renderOverlay", Qt::QueuedConnection);
}

void TopicTextOverlayPanel::setTopicText(const std::string & topic, const std::string & text)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      return;
    }
    it->second.has_message = true;
    it->second.text = text;
  }

  QMetaObject::invokeMethod(this, "renderOverlay", Qt::QueuedConnection);
}

std::vector<std::pair<std::string, TopicTextOverlayPanel::TopicState>>
TopicTextOverlayPanel::snapshotSelectedTopics() const
{
  std::vector<std::pair<std::string, TopicState>> selected_topics;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [topic, state] : topics_) {
    if (state.selected) {
      selected_topics.emplace_back(topic, state);
    }
  }
  return selected_topics;
}

void TopicTextOverlayPanel::renderOverlay()
{
  if (!overlay_) {
    return;
  }
  if (!enable_overlay_->isChecked()) {
    overlay_->hide();
    return;
  }
  overlay_->show();

  const auto selected_topics = snapshotSelectedTopics();
  const int font_size = font_size_spin_->value();
  const int max_letters = max_letter_spin_->value();
  const int texture_width = std::max(320, font_size * 80);
  const int line_height = static_cast<int>(std::ceil(font_size * 1.35));
  const int margin = 10;
  int texture_height = margin;
  for (const auto & [topic, state] : selected_topics) {
    const auto text = state.has_message ? truncateText(state.text, max_letters)
                                        : std::string{"(waiting for message)"};
    texture_height += static_cast<int>(countLines(text) + 1U) * line_height + margin;
  }
  texture_height = std::max(texture_height + margin, 80);

  overlay_->updateTextureSize(
    static_cast<unsigned int>(texture_width), static_cast<unsigned int>(texture_height));
  overlay_->setPosition(left_spin_->value(), top_spin_->value());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  auto buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(*overlay_);
  image.fill(QColor(0, 0, 0, 0));

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QFont font = painter.font();
  font.setPixelSize(font_size);
  font.setBold(true);
  painter.setFont(font);

  int y = margin;
  if (selected_topics.empty()) {
    painter.setPen(QColor(220, 220, 220));
    painter.drawText(margin, y + line_height, "Select one or more supported text topics.");
    painter.end();
    overlay_->setPosition(left_spin_->value(), top_spin_->value());
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    getDisplayContext()->queueRender();
    return;
  }

  for (const auto & [topic, state] : selected_topics) {
    painter.setPen(QPen(QColor(0, 255, 177), 2, Qt::SolidLine));
    painter.drawText(
      margin, y, texture_width - 2 * margin, line_height, Qt::AlignLeft | Qt::AlignVCenter,
      QString::fromStdString("[" + displayTypeName(state.kind) + "] " + topic));
    y += line_height;

    painter.setPen(QColor(255, 255, 255));
    const auto text = state.has_message ? truncateText(state.text, max_letters)
                                        : std::string{"(waiting for message)"};
    const int text_height = static_cast<int>(countLines(text)) * line_height;
    painter.drawText(
      margin + 12, y, texture_width - 2 * margin - 12, std::max(text_height, line_height),
      Qt::AlignLeft | Qt::AlignTop, QString::fromStdString(text));
    y += std::max(text_height, line_height) + margin;
  }
  painter.end();
  overlay_->setPosition(left_spin_->value(), top_spin_->value());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  getDisplayContext()->queueRender();
}
}  // namespace autoware::topic_text_overlay_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::topic_text_overlay_rviz_plugin::TopicTextOverlayPanel, rviz_common::Panel)
