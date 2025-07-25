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

#include "autoware_planning_rviz_plugin/candidate_trajectories/display_base.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
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

template <typename MessageType>
CandidateTrajectoriesDisplayBase<MessageType>::CandidateTrajectoriesDisplayBase()
: property_topic_{"Topic", "", "", "", this},
  property_path_view_{"View Path", true, "", this},
  property_path_width_view_{"Constant Width", false, "", &property_path_view_},
  property_path_width_{"Width", 2.0, "", &property_path_view_},
  property_path_alpha_{"Alpha", 1.0, "", &property_path_view_},
  property_coloring_mode_{"Coloring Mode", 0, "", &property_path_view_},
  property_fade_out_distance_{"Fade Out Distance [m]", 0.0, "", &property_path_view_},
  property_velocity_color_min_{
    "Min Velocity Color", QColor("#3F2EE3"), "", &property_coloring_mode_},
  property_velocity_color_mid_{
    "Mid Velocity Color", QColor("#208AAE"), "", &property_coloring_mode_},
  property_velocity_color_max_{
    "Max Velocity Color", QColor("#00E678"), "", &property_coloring_mode_},
  property_vel_max_{"Vel Max [m/s]", 3.0, "", &property_coloring_mode_},
  property_velocity_view_{"View Velocity", true, "", this},
  property_velocity_alpha_{"Alpha", 1.0, "", &property_velocity_view_},
  property_velocity_scale_{"Scale", 0.3, "", &property_velocity_view_},
  property_velocity_color_view_{"Constant Color", false, "", &property_velocity_view_},
  property_velocity_color_{"Color", Qt::black, "", &property_velocity_view_},
  property_generator_text_view_{"View Generator Names", true, "", this},
  property_generator_text_scale_{"Generator Text Scale", 0.5, "", &property_generator_text_view_}
{
  // Initialize property constraints only - signal connections must be done in derived classes
  initializePropertyConstraints();
}

template <typename MessageType>
CandidateTrajectoriesDisplayBase<MessageType>::~CandidateTrajectoriesDisplayBase()
{
  if (this->initialized()) {
    for (auto * obj : path_manual_objects_) {
      this->scene_manager_->destroyManualObject(obj);
    }
    for (auto * obj : velocity_manual_objects_) {
      this->scene_manager_->destroyManualObject(obj);
    }
    for (auto * text : generator_texts_) {
      delete text;
    }
    for (auto * node : generator_text_nodes_) {
      node->removeAndDestroyAllChildren();
      node->detachAllObjects();
      this->scene_manager_->destroySceneNode(node);
    }
  }
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::onInitialize()
{
  Display::onInitialize();
  property_topic_.initialize(this->context_->getRosNodeAbstraction());
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::onEnable()
{
  subscribe();
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::onDisable()
{
  unsubscribe();
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::reset()
{
  Display::reset();
  for (auto * obj : path_manual_objects_) {
    obj->clear();
  }
  for (auto * obj : velocity_manual_objects_) {
    obj->clear();
  }
  for (auto * text : generator_texts_) {
    text->setVisible(false);
  }
}

template <typename MessageType>
std::unique_ptr<Ogre::ColourValue>
CandidateTrajectoriesDisplayBase<MessageType>::setColorDependsOnVelocity(const double velocity)
{
  const double vel_max = property_vel_max_.getFloat();
  const double ratio = std::min(std::max(velocity / vel_max, 0.0), 1.0);

  if (ratio < 0.5) {
    return gradation(
      property_velocity_color_min_.getColor(), property_velocity_color_mid_.getColor(),
      ratio * 2.0);
  } else {
    return gradation(
      property_velocity_color_mid_.getColor(), property_velocity_color_max_.getColor(),
      (ratio - 0.5) * 2.0);
  }
}

template <typename MessageType>
std::unique_ptr<Ogre::ColourValue> CandidateTrajectoriesDisplayBase<MessageType>::gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g =
    static_cast<float>(color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio));
  color_ptr->r = static_cast<float>(color_max.redF() * ratio + color_min.redF() * (1.0 - ratio));
  color_ptr->b = static_cast<float>(color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio));
  color_ptr->a = 1.0f;
  return color_ptr;
}

template <typename MessageType>
bool CandidateTrajectoriesDisplayBase<MessageType>::validateFloats(
  const typename MessageType::ConstSharedPtr & /* msg_ptr */)
{
  // This will be specialized for each message type
  return true;
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::subscribe()
{
  if (!this->isEnabled()) {
    return;
  }

  if (subscription_) {
    unsubscribe();
  }

  try {
    auto node = this->context_->getRosNodeAbstraction().lock()->get_raw_node();
    const auto topic_name = property_topic_.getTopicStd();
    if (topic_name.empty()) {
      return;
    }

    subscription_ = node->template create_subscription<MessageType>(
      topic_name, 10, [this](const typename MessageType::SharedPtr msg) {
        this->last_msg_ptr_ = msg;
        processMessage(msg);
      });
    this->setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (const std::exception & e) {
    this->setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      QString("Error subscribing: ") + e.what());
  }
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::unsubscribe()
{
  subscription_.reset();
}

// Helper method implementations

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::initializePropertyConstraints()
{
  // Set property value ranges
  property_path_width_.setMin(0.0);
  property_path_alpha_.setMin(0.0);
  property_path_alpha_.setMax(1.0);
  property_vel_max_.setMin(0.1);
  property_fade_out_distance_.setMin(0.0);
  property_velocity_alpha_.setMin(0.0);
  property_velocity_alpha_.setMax(1.0);
  property_velocity_scale_.setMin(0.1);
  property_velocity_scale_.setMax(10.0);
  property_generator_text_scale_.setMin(0.1);
  property_generator_text_scale_.setMax(10.0);
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::resizeManualObjects(size_t num_trajectories)
{
  // Add objects if we need more
  while (path_manual_objects_.size() < num_trajectories) {
    auto * path_obj = this->scene_manager_->createManualObject();
    auto * vel_obj = this->scene_manager_->createManualObject();
    path_obj->setDynamic(true);
    vel_obj->setDynamic(true);
    this->scene_node_->attachObject(path_obj);
    this->scene_node_->attachObject(vel_obj);
    path_manual_objects_.push_back(path_obj);
    velocity_manual_objects_.push_back(vel_obj);
  }

  // Remove excess objects
  while (path_manual_objects_.size() > num_trajectories) {
    auto * path_obj = path_manual_objects_.back();
    auto * vel_obj = velocity_manual_objects_.back();
    this->scene_manager_->destroyManualObject(path_obj);
    this->scene_manager_->destroyManualObject(vel_obj);
    path_manual_objects_.pop_back();
    velocity_manual_objects_.pop_back();
  }
}

template <typename MessageType>
void CandidateTrajectoriesDisplayBase<MessageType>::clearManualObjects()
{
  for (auto * obj : path_manual_objects_) {
    obj->clear();
  }
  for (auto * obj : velocity_manual_objects_) {
    obj->clear();
  }
}

// Slot implementations moved to derived classes for proper Qt MOC support

// Explicit instantiation for the message types we support
template class CandidateTrajectoriesDisplayBase<
  autoware_internal_planning_msgs::msg::CandidateTrajectories>;
template class CandidateTrajectoriesDisplayBase<
  autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>;

}  // namespace rviz_plugins
