// Copyright 2024 TIER IV, Inc.
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

#include "string_stamped_overlay_display.hpp"

#include <QFontMetrics>
#include <QPainter>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_system.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace
{
constexpr int k_min_width = 1;
constexpr int k_min_height = 1;
}  // namespace

namespace autoware::string_stamped_rviz_plugin
{

StringStampedOverlayDisplay::StringStampedOverlayDisplay()
{
  // Topic property
  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "",
    rosidl_generator_traits::data_type<autoware_internal_debug_msgs::msg::StringStamped>(),
    "Topic to subscribe for StringStamped messages", this, SLOT(update_topic()), this);

  // Padding property (size is calculated automatically based on text)
  padding_property_ = new rviz_common::properties::IntProperty(
    "Padding", 5, "Padding around the text (pixels)", this);
  padding_property_->setMin(0);

  // Position properties
  left_property_ = new rviz_common::properties::IntProperty(
    "Left", 0, "Horizontal distance from alignment edge", this, SLOT(update_overlay_position()));

  top_property_ = new rviz_common::properties::IntProperty(
    "Top", 0, "Vertical distance from alignment edge", this, SLOT(update_overlay_position()));

  // Alignment properties
  hor_alignment_property_ = new rviz_common::properties::EnumProperty(
    "Horizontal Alignment", "Left", "Horizontal alignment of the overlay", this,
    SLOT(update_overlay_position()));
  hor_alignment_property_->addOption("Left", 0);
  hor_alignment_property_->addOption("Center", 1);
  hor_alignment_property_->addOption("Right", 2);

  ver_alignment_property_ = new rviz_common::properties::EnumProperty(
    "Vertical Alignment", "Top", "Vertical alignment of the overlay", this,
    SLOT(update_overlay_position()));
  ver_alignment_property_->addOption("Top", 0);
  ver_alignment_property_->addOption("Center", 1);
  ver_alignment_property_->addOption("Bottom", 2);

  // Font size property
  font_size_property_ =
    new rviz_common::properties::IntProperty("Font Size", 12, "Font size for text display", this);
  font_size_property_->setMin(1);

  // Color properties
  fg_color_property_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255), "Text color", this);

  fg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Text Alpha", 1.0, "Text transparency (0.0 - 1.0)", this);
  fg_alpha_property_->setMin(0.0);
  fg_alpha_property_->setMax(1.0);

  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", QColor(0, 0, 0), "Background color", this);

  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", 0.0, "Background transparency (0.0 - 1.0)", this);
  bg_alpha_property_->setMin(0.0);
  bg_alpha_property_->setMax(1.0);

  // Fade-out timing properties
  fade_delay_property_ = new rviz_common::properties::FloatProperty(
    "Fade Delay", 0.0, "Delay before fade-out starts after receiving empty message (seconds)",
    this);
  fade_delay_property_->setMin(0.0);

  fade_time_property_ = new rviz_common::properties::FloatProperty(
    "Fade Time", 1.0, "Duration of fade-out effect after keep time (seconds)", this);
  fade_time_property_->setMin(0.001);
}

StringStampedOverlayDisplay::~StringStampedOverlayDisplay()
{
  subscription_.reset();
  overlay_.reset();
}

void StringStampedOverlayDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  static int count = 0;
  std::stringstream ss;
  ss << "StringStampedOverlayDisplay" << count++;
  overlay_ = std::make_shared<rviz_2d_overlay_plugins::OverlayObject>(ss.str());

  overlay_->show();
  update_overlay_position();

  topic_property_->initialize(context_->getRosNodeAbstraction());
}

void StringStampedOverlayDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  update_topic();
}

void StringStampedOverlayDisplay::onDisable()
{
  subscription_.reset();
  if (overlay_) {
    overlay_->hide();
  }
}

void StringStampedOverlayDisplay::reset()
{
  rviz_common::Display::reset();
  if (overlay_) {
    overlay_->hide();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  text_.clear();
  last_msg_.reset();
  last_non_empty_msg_.reset();
  current_alpha_ = 255;
}

void StringStampedOverlayDisplay::update_topic()
{
  subscription_.reset();

  if (!isEnabled()) {
    return;
  }

  const std::string topic = topic_property_->getTopicStd();
  if (topic.empty()) {
    return;
  }

  try {
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    subscription_ = node->create_subscription<autoware_internal_debug_msgs::msg::StringStamped>(
      topic, rclcpp::QoS(10),
      std::bind(&StringStampedOverlayDisplay::process_message, this, std::placeholders::_1));
  } catch (const std::exception & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", e.what());
  }
}

void StringStampedOverlayDisplay::update_overlay_position()
{
  if (!overlay_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Get horizontal alignment
  auto hor_alignment = rviz_2d_overlay_plugins::HorizontalAlignment::LEFT;
  switch (hor_alignment_property_->getOptionInt()) {
    case 1:
      hor_alignment = rviz_2d_overlay_plugins::HorizontalAlignment::CENTER;
      break;
    case 2:
      hor_alignment = rviz_2d_overlay_plugins::HorizontalAlignment::RIGHT;
      break;
    default:
      break;
  }

  // Get vertical alignment
  auto ver_alignment = rviz_2d_overlay_plugins::VerticalAlignment::TOP;
  switch (ver_alignment_property_->getOptionInt()) {
    case 1:
      ver_alignment = rviz_2d_overlay_plugins::VerticalAlignment::CENTER;
      break;
    case 2:
      ver_alignment = rviz_2d_overlay_plugins::VerticalAlignment::BOTTOM;
      break;
    default:
      break;
  }

  overlay_->setPosition(
    left_property_->getInt(), top_property_->getInt(), hor_alignment, ver_alignment);
  queueRender();
}

void StringStampedOverlayDisplay::process_message(
  const autoware_internal_debug_msgs::msg::StringStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_ = msg;
  if (!msg->data.empty()) {
    last_non_empty_msg_ = msg;
    current_alpha_ = 255;
  }
  text_ = msg->data;
  queueRender();
}

void StringStampedOverlayDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  if (!overlay_ || !overlay_->isVisible()) {
    return;
  }

  std::string display_text;

  // Handle fade-out effect for empty messages
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_msg_ && last_msg_->data.empty() && last_non_empty_msg_) {
      auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
      const auto current_time = node->now();
      const double duration = (current_time - last_non_empty_msg_->stamp).seconds();

      const double fade_delay = fade_delay_property_->getFloat();
      const double fade_time = fade_time_property_->getFloat();

      if (duration < fade_delay + fade_time) {
        // Show last non-empty message with fade effect
        text_ = last_non_empty_msg_->data;
        const double fade_progress = std::max(0.0, duration - fade_delay) / fade_time;
        current_alpha_ = static_cast<int>((1.0 - fade_progress) * 255);
      } else {
        // Fully faded, clear text
        text_.clear();
        current_alpha_ = 255;
      }
    } else if (last_msg_ && !last_msg_->data.empty()) {
      // Reset alpha for non-empty messages
      current_alpha_ = 255;
    }
    display_text = text_;
  }

  // Calculate required size based on text
  const int padding = padding_property_->getInt();
  QFont font;
  font.setPointSize(font_size_property_->getInt());
  QFontMetrics fm(font);

  QString q_text = QString::fromStdString(display_text);

  QRect text_rect = fm.boundingRect(QRect(0, 0, 0, 0), Qt::AlignLeft, q_text);

  const int required_width = std::max(k_min_width, text_rect.width() + padding * 2);
  const int required_height = std::max(k_min_height, text_rect.height() + padding * 2);

  // Update overlay size if needed
  if (
    static_cast<int>(overlay_->getTextureWidth()) != required_width ||
    static_cast<int>(overlay_->getTextureHeight()) != required_height) {
    overlay_->updateTextureSize(required_width, required_height);
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  }
  // Draw overlay
  rviz_2d_overlay_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage image = buffer.getQImage(*overlay_);

  // Fill background
  QColor bg_color = bg_color_property_->getColor();
  bg_color.setAlphaF(bg_alpha_property_->getFloat());
  image.fill(bg_color);

  draw_text(image);
}

void StringStampedOverlayDisplay::draw_text(QImage & image)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (text_.empty()) {
    return;
  }

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);

  // Set font
  QFont font = painter.font();
  font.setPointSize(font_size_property_->getInt());
  painter.setFont(font);

  // Set text color with alpha
  QColor fg_color = fg_color_property_->getColor();
  const double alpha_ratio = static_cast<double>(current_alpha_) / 255.0;
  const int alpha = static_cast<int>(fg_alpha_property_->getFloat() * alpha_ratio * 255.0);
  fg_color.setAlpha(alpha);
  painter.setPen(fg_color);

  // Draw text
  QRect rect(0, 0, image.width(), image.height());
  painter.drawText(
    rect, Qt::AlignLeft | Qt::AlignVCenter | Qt::TextWordWrap, QString::fromStdString(text_));

  painter.end();
}

}  // namespace autoware::string_stamped_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::string_stamped_rviz_plugin::StringStampedOverlayDisplay, rviz_common::Display)
