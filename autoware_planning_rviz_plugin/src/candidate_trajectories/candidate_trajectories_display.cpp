// Copyright 2025 TIER IV, Inc.
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

#include "autoware_planning_rviz_plugin/candidate_trajectories/candidate_trajectories_display.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/validate_floats.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <algorithm>
#include <cmath>
#include <memory>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rviz_plugins
{

AutowareCandidateTrajectoriesDisplay::AutowareCandidateTrajectoriesDisplay()
{
  // Setup topic description
  this->property_topic_.setMessageType("autoware_internal_planning_msgs/msg/CandidateTrajectories");
  this->property_topic_.setDescription("Topic for candidate trajectories");

  // Connect all property signals to updateVisualization slot
  // Base class properties
  connect(&this->property_path_view_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_path_width_view_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_path_width_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_path_alpha_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_fade_out_distance_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(
    &this->property_velocity_color_min_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(
    &this->property_velocity_color_mid_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(
    &this->property_velocity_color_max_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_vel_max_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_velocity_view_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_velocity_alpha_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_velocity_scale_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(
    &this->property_velocity_color_view_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&this->property_velocity_color_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(
    &this->property_generator_text_view_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(
    &this->property_generator_text_scale_, SIGNAL(changed()), this, SLOT(updateVisualization()));

  // Derived class properties
  connect(&property_index_color_1_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&property_index_color_2_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&property_index_color_3_, SIGNAL(changed()), this, SLOT(updateVisualization()));
  connect(&property_index_color_4_, SIGNAL(changed()), this, SLOT(updateVisualization()));

  // Special handlers
  connect(
    &this->property_coloring_mode_, SIGNAL(changed()), this, SLOT(updateColoringModeVisibility()));
  connect(&this->property_topic_, SIGNAL(changed()), this, SLOT(onTopicChanged()));

  setupColoringModes();
}

AutowareCandidateTrajectoriesDisplay::~AutowareCandidateTrajectoriesDisplay()
{
  // Clean up any remaining resources
  index_colors_.clear();
}

void AutowareCandidateTrajectoriesDisplay::setupColoringModes()
{
  // Set up coloring mode enum for candidate trajectories
  this->property_coloring_mode_.addOption("Velocity Based", CANDIDATE_VELOCITY_BASED);
  this->property_coloring_mode_.addOption("Index Based", INDEX_BASED);

  // Set default to Velocity Based and trigger UI update
  this->property_coloring_mode_.setString("Velocity Based");
}

void AutowareCandidateTrajectoriesDisplay::updateVisualization()
{
  if (this->last_msg_ptr_) {
    processMessage(this->last_msg_ptr_);
  }
}

void AutowareCandidateTrajectoriesDisplay::updateColoringModeVisibility()
{
  // Update color mapping and trigger visualization
  updateIndexColors();
  updateVisualization();
}

void AutowareCandidateTrajectoriesDisplay::onTopicChanged()
{
  // Automatically subscribe when topic changes
  // This matches standard RViz behavior where topic selection enables visualization
  if (this->isEnabled()) {
    this->subscribe();
  }
}

std::unique_ptr<Ogre::ColourValue> AutowareCandidateTrajectoriesDisplay::setColorDependsOnIndex(
  const size_t index)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);

  // Use modulo to cycle through available colors
  QColor qt_color = index_colors_[index % index_colors_.size()];

  color_ptr->r = qt_color.redF();
  color_ptr->g = qt_color.greenF();
  color_ptr->b = qt_color.blueF();
  color_ptr->a = 1.0f;

  return color_ptr;
}

bool AutowareCandidateTrajectoriesDisplay::validateFloats(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr & msg_ptr)
{
  for (const auto & trajectory : msg_ptr->candidate_trajectories) {
    for (const auto & point : trajectory.points) {
      if (
        !rviz_common::validateFloats(point.pose) ||
        !rviz_common::validateFloats(point.longitudinal_velocity_mps)) {
        return false;
      }
    }
  }
  return true;
}

void AutowareCandidateTrajectoriesDisplay::processMessage(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg_ptr)
{
  if (!validateFloats(msg_ptr)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Handle frame transformation
  if (!msg_ptr->candidate_trajectories.empty()) {
    const auto & first_trajectory = msg_ptr->candidate_trajectories[0];
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(
          first_trajectory.header, position, orientation)) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Transform",
        QString("Error transforming from frame '%1' to frame '%2'")
          .arg(QString::fromStdString(first_trajectory.header.frame_id))
          .arg(fixed_frame_));
      return;
    }
    this->scene_node_->setPosition(position);
    this->scene_node_->setOrientation(orientation);
  }

  const size_t num_trajectories = msg_ptr->candidate_trajectories.size();

  // Use base class methods for common operations
  this->clearManualObjects();
  this->resizeManualObjects(num_trajectories);

  // Resize generator text objects
  while (this->generator_texts_.size() < msg_ptr->generator_info.size()) {
    auto * node = this->scene_node_->createChildSceneNode();
    auto * text = new rviz_rendering::MovableText("generator", "Liberation Sans", 1.0);
    text->setVisible(false);
    text->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
    text->setColor(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
    node->attachObject(text);
    this->generator_texts_.push_back(text);
    this->generator_text_nodes_.push_back(node);
  }
  while (this->generator_texts_.size() > msg_ptr->generator_info.size()) {
    auto * text = this->generator_texts_.back();
    auto * node = this->generator_text_nodes_.back();
    delete text;
    node->removeAndDestroyAllChildren();
    node->detachAllObjects();
    this->scene_manager_->destroySceneNode(node);
    this->generator_texts_.pop_back();
    this->generator_text_nodes_.pop_back();
  }

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);

  for (size_t traj_idx = 0; traj_idx < num_trajectories; ++traj_idx) {
    const auto & trajectory = msg_ptr->candidate_trajectories[traj_idx];

    if (trajectory.points.empty()) {
      continue;
    }

    auto * path_obj = this->path_manual_objects_[traj_idx];
    auto * vel_obj = this->velocity_manual_objects_[traj_idx];

    const size_t num_points = trajectory.points.size();
    path_obj->estimateVertexCount(num_points * 2);
    vel_obj->estimateVertexCount(num_points);
    path_obj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    vel_obj->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (size_t point_idx = 0; point_idx < num_points; ++point_idx) {
      const auto & point = trajectory.points[point_idx];

      // Path visualization
      if (this->property_path_view_.getBool()) {
        Ogre::ColourValue color;

        // Select coloring based on mode
        switch (this->property_coloring_mode_.getOptionInt()) {
          case CANDIDATE_VELOCITY_BASED: {
            std::unique_ptr<Ogre::ColourValue> velocity_color =
              this->setColorDependsOnVelocity(point.longitudinal_velocity_mps);
            color = *velocity_color;
            break;
          }
          case INDEX_BASED: {
            std::unique_ptr<Ogre::ColourValue> index_color = setColorDependsOnIndex(traj_idx);
            color = *index_color;
            break;
          }
          default: {
            std::unique_ptr<Ogre::ColourValue> velocity_color =
              this->setColorDependsOnVelocity(point.longitudinal_velocity_mps);
            color = *velocity_color;
            break;
          }
        }

        // Apply fade out distance alpha calculation
        float alpha = this->property_path_alpha_.getFloat();
        if (this->property_fade_out_distance_.getFloat() > 0.0) {
          // Calculate distance from end of trajectory
          float distance_from_end = 0.0;
          for (size_t i = point_idx; i < num_points - 1; ++i) {
            const auto & current_point = trajectory.points[i];
            const auto & next_point = trajectory.points[i + 1];
            distance_from_end += std::sqrt(
              std::pow(next_point.pose.position.x - current_point.pose.position.x, 2) +
              std::pow(next_point.pose.position.y - current_point.pose.position.y, 2) +
              std::pow(next_point.pose.position.z - current_point.pose.position.z, 2));
          }

          if (distance_from_end < this->property_fade_out_distance_.getFloat()) {
            float ratio = distance_from_end / this->property_fade_out_distance_.getFloat();
            alpha = this->property_path_alpha_.getFloat() * ratio;
          }
        }
        color.a = alpha;

        // Width calculation - use vehicle width unless constant width is enabled
        float half_width;
        if (this->property_path_width_view_.getBool()) {
          half_width = this->property_path_width_.getFloat() / 2.0f;
        } else {
          // Use vehicle width (fallback to property if unavailable)
          try {
            const auto vehicle_info =
              autoware::vehicle_info_utils::VehicleInfoUtils(
                *this->context_->getRosNodeAbstraction().lock()->get_raw_node())
                .getVehicleInfo();
            half_width = vehicle_info.vehicle_width_m / 2.0f;
          } catch (...) {
            half_width = this->property_path_width_.getFloat() / 2.0f;
          }
        }

        Eigen::Vector3f vec_in, vec_out;
        Eigen::Quaternionf quat(
          static_cast<float>(point.pose.orientation.w),
          static_cast<float>(point.pose.orientation.x),
          static_cast<float>(point.pose.orientation.y),
          static_cast<float>(point.pose.orientation.z));

        // Right side
        vec_in << 0, half_width, 0;
        vec_out = quat * vec_in;
        path_obj->position(
          point.pose.position.x + vec_out.x(), point.pose.position.y + vec_out.y(),
          point.pose.position.z + vec_out.z());
        path_obj->colour(color);

        // Left side
        vec_in << 0, -half_width, 0;
        vec_out = quat * vec_in;
        path_obj->position(
          point.pose.position.x + vec_out.x(), point.pose.position.y + vec_out.y(),
          point.pose.position.z + vec_out.z());
        path_obj->colour(color);
      }

      // Velocity visualization
      if (this->property_velocity_view_.getBool()) {
        Ogre::ColourValue color;
        if (this->property_velocity_color_view_.getBool()) {
          auto qt_color = this->property_velocity_color_.getColor();
          color.r = qt_color.redF();
          color.g = qt_color.greenF();
          color.b = qt_color.blueF();
          color.a = 1.0f;
        } else {
          std::unique_ptr<Ogre::ColourValue> velocity_color =
            this->setColorDependsOnVelocity(point.longitudinal_velocity_mps);
          color = *velocity_color;
        }
        color.a = this->property_velocity_alpha_.getFloat();

        vel_obj->position(
          point.pose.position.x, point.pose.position.y,
          point.pose.position.z +
            point.longitudinal_velocity_mps * this->property_velocity_scale_.getFloat());
        vel_obj->colour(color);
      }
    }

    path_obj->end();
    vel_obj->end();
  }

  // Visualize generator names
  if (this->property_generator_text_view_.getBool()) {
    for (size_t gen_idx = 0; gen_idx < msg_ptr->generator_info.size(); ++gen_idx) {
      const auto & gen_info = msg_ptr->generator_info[gen_idx];

      // Find trajectory with this generator ID
      bool found_trajectory = false;
      for (size_t traj_idx = 0; traj_idx < num_trajectories; ++traj_idx) {
        const auto & trajectory = msg_ptr->candidate_trajectories[traj_idx];
        if (trajectory.generator_id == gen_info.generator_id) {
          if (!trajectory.points.empty()) {
            const auto & last_point = trajectory.points.back();

            Ogre::Vector3 position;
            position.x = last_point.pose.position.x;
            position.y = last_point.pose.position.y;
            position.z = last_point.pose.position.z + 7.0;

            auto * node = this->generator_text_nodes_[gen_idx];
            node->setPosition(position);

            auto * text = this->generator_texts_[gen_idx];
            text->setCaption(gen_info.generator_name.data.c_str());
            text->setCharacterHeight(
              std::max(1.0f, this->property_generator_text_scale_.getFloat()));
            text->setColor(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
            text->setVisible(true);

            found_trajectory = true;
            break;
          }
        }
      }

      if (!found_trajectory && gen_idx < this->generator_texts_.size()) {
        auto * text = this->generator_texts_[gen_idx];
        text->setVisible(false);
      }
    }
  } else {
    for (auto * text : this->generator_texts_) {
      text->setVisible(false);
    }
  }

  this->last_msg_ptr_ = msg_ptr;
}

void AutowareCandidateTrajectoriesDisplay::updateIndexColors()
{
  index_colors_.clear();
  index_colors_.push_back(property_index_color_1_.getColor());
  index_colors_.push_back(property_index_color_2_.getColor());
  index_colors_.push_back(property_index_color_3_.getColor());
  index_colors_.push_back(property_index_color_4_.getColor());
}

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareCandidateTrajectoriesDisplay, rviz_common::Display)
