// Copyright 2020 Tier IV, Inc.
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

#include "autoware_perception_rviz_plugin/object_detection/predicted_objects_display.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <set>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
PredictedObjectsDisplay::PredictedObjectsDisplay()
: ObjectPolygonDisplayBase("predicted_objects"),
  m_offset_property{"Display Offset", 0, "Visualize steps in predicted path", this}
{
  // A single background thread builds the markers off the RViz render thread.
  worker_thread_ = std::thread(&PredictedObjectsDisplay::workerThread, this);
}

void PredictedObjectsDisplay::workerThread()
{
  // Wait for the latest message, build its markers off the render thread, and hand them back to
  // update(). Only the most recent message is kept, so bursts collapse to the newest frame.
  while (true) {
    PredictedObjects::ConstSharedPtr current_msg;
    {
      std::unique_lock<std::mutex> lock(mutex);
      condition.wait(lock, [this] { return msg != nullptr || should_terminate; });
      if (should_terminate) {
        return;
      }
      current_msg = msg;
      msg.reset();
    }

    auto tmp_markers = createMarkers(current_msg);

    std::unique_lock<std::mutex> lock(mutex);
    markers = tmp_markers;
  }
}

std::vector<visualization_msgs::msg::Marker::SharedPtr> PredictedObjectsDisplay::createMarkers(
  PredictedObjects::ConstSharedPtr msg)
{
  update_id_map(msg);

  std::vector<visualization_msgs::msg::Marker::SharedPtr> markers;

  for (const auto & object : msg->objects) {
    auto object_pose = &(object.kinematics.initial_pose_with_covariance.pose);
    if (m_offset_property.getInt() > 0) {
      // get best confidence path
      auto best_path = std::max_element(
        object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
        [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
      if (best_path != object.kinematics.predicted_paths.end() && !best_path->path.empty()) {
        // clamp offset
        auto offset =
          std::min(static_cast<size_t>(m_offset_property.getInt()), best_path->path.size() - 1);
        // get offset pose
        object_pose = &(best_path->path.at(offset));
      }
    }
    // Get markers for shape (supports single marker or wireframe+fill in Hologram 3D mode)
    const auto shape_markers = get_shape_marker_ptrs(
      object.shape, object_pose->position, object_pose->orientation, object.classification,
      get_line_width(), true);
    int32_t shape_sub_idx = 0;
    for (const auto & marker_ptr : shape_markers) {
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id) + shape_sub_idx * 1000000;
      ++shape_sub_idx;
      markers.push_back(marker_ptr);
    }

    auto mesh_marker = get_mesh_marker_ptr(
      object.shape, object_pose->position, object_pose->orientation, object.classification);
    if (mesh_marker) {
      auto mesh_marker_ptr = mesh_marker.value();
      mesh_marker_ptr->header = msg->header;
      mesh_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(mesh_marker_ptr);
    }

    // Get marker for label
    auto label_marker =
      get_label_marker_ptr(object_pose->position, object_pose->orientation, object.classification);
    if (label_marker) {
      auto marker_ptr = label_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(marker_ptr);
    }

    // Get marker for id
    geometry_msgs::msg::Point uuid_vis_position;
    uuid_vis_position.x = object_pose->position.x - 0.5;
    uuid_vis_position.y = object_pose->position.y;
    uuid_vis_position.z = object_pose->position.z - 0.5;

    auto id_marker =
      get_uuid_marker_ptr(object.object_id, uuid_vis_position, object.classification);
    if (id_marker) {
      auto id_marker_ptr = id_marker.value();
      id_marker_ptr->header = msg->header;
      id_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(id_marker_ptr);
    }

    // Get marker for pose covariance
    auto pose_with_covariance_marker =
      get_pose_covariance_marker_ptr(object.kinematics.initial_pose_with_covariance);
    if (pose_with_covariance_marker) {
      auto marker_ptr = pose_with_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(marker_ptr);
    }

    // Get marker for yaw covariance
    auto yaw_covariance_marker = get_yaw_covariance_marker_ptr(
      object.kinematics.initial_pose_with_covariance, object.shape.dimensions.x * 0.65,
      get_line_width() * 0.5);
    if (yaw_covariance_marker) {
      auto marker_ptr = yaw_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(marker_ptr);
    }

    // Get marker for existence probability
    geometry_msgs::msg::Point existence_probability_position;
    existence_probability_position.x = object_pose->position.x + 0.5;
    existence_probability_position.y = object_pose->position.y;
    existence_probability_position.z = object_pose->position.z + 0.5;
    const float existence_probability = object.existence_probability;
    auto existence_prob_marker = get_existence_probability_marker_ptr(
      existence_probability_position, object_pose->orientation, existence_probability,
      object.classification);
    if (existence_prob_marker) {
      auto existence_prob_marker_ptr = existence_prob_marker.value();
      existence_prob_marker_ptr->header = msg->header;
      existence_prob_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(existence_prob_marker_ptr);
    }
    // Get marker for velocity text
    geometry_msgs::msg::Point vel_vis_position;
    vel_vis_position.x = uuid_vis_position.x - 0.5;
    vel_vis_position.y = uuid_vis_position.y;
    vel_vis_position.z = uuid_vis_position.z - 0.5;
    auto velocity_text_marker = get_velocity_text_marker_ptr(
      object.kinematics.initial_twist_with_covariance.twist, vel_vis_position,
      object.classification);
    if (velocity_text_marker) {
      auto velocity_text_marker_ptr = velocity_text_marker.value();
      velocity_text_marker_ptr->header = msg->header;
      velocity_text_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(velocity_text_marker_ptr);
    }

    // Get marker for acceleration text
    geometry_msgs::msg::Point acc_vis_position;
    acc_vis_position.x = uuid_vis_position.x - 1.0;
    acc_vis_position.y = uuid_vis_position.y;
    acc_vis_position.z = uuid_vis_position.z - 1.0;
    auto acceleration_text_marker = get_acceleration_text_marker_ptr(
      object.kinematics.initial_acceleration_with_covariance.accel, acc_vis_position,
      object.classification);
    if (acceleration_text_marker) {
      auto acceleration_text_marker_ptr = acceleration_text_marker.value();
      acceleration_text_marker_ptr->header = msg->header;
      acceleration_text_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(acceleration_text_marker_ptr);
    }

    // Get marker for twist
    auto twist_marker = get_twist_marker_ptr(
      object.kinematics.initial_pose_with_covariance,
      object.kinematics.initial_twist_with_covariance, get_line_width());
    if (twist_marker) {
      auto twist_marker_ptr = twist_marker.value();
      twist_marker_ptr->header = msg->header;
      twist_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(twist_marker_ptr);
    }

    // Get marker for twist covariance
    auto twist_covariance_marker = get_twist_covariance_marker_ptr(
      object.kinematics.initial_pose_with_covariance,
      object.kinematics.initial_twist_with_covariance);
    if (twist_covariance_marker) {
      auto marker_ptr = twist_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(marker_ptr);
    }

    // Get marker for yaw rate
    auto yaw_rate_marker = get_yaw_rate_marker_ptr(
      object.kinematics.initial_pose_with_covariance,
      object.kinematics.initial_twist_with_covariance, get_line_width() * 0.4);
    if (yaw_rate_marker) {
      auto marker_ptr = yaw_rate_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(marker_ptr);
    }

    // Get marker for yaw rate covariance
    auto yaw_rate_covariance_marker = get_yaw_rate_covariance_marker_ptr(
      object.kinematics.initial_pose_with_covariance,
      object.kinematics.initial_twist_with_covariance, get_line_width() * 0.3);
    if (yaw_rate_covariance_marker) {
      auto marker_ptr = yaw_rate_covariance_marker.value();
      marker_ptr->header = msg->header;
      marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(marker_ptr);
    }

    // Add marker for each candidate path
    int32_t path_count = 0;
    if (is_hologram_path_style()) {
      const auto & paths = object.kinematics.predicted_paths;
      if (!paths.empty()) {
        std::vector<size_t> sorted_indices(paths.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        std::stable_sort(
          sorted_indices.begin(), sorted_indices.end(),
          [&paths](size_t a, size_t b) { return paths[a].confidence > paths[b].confidence; });

        const float max_conf = std::max(1e-4F, paths[sorted_indices.front()].confidence);
        const float min_conf = get_path_min_confidence();
        const double horizon_sec = get_path_horizon_sec();
        const auto class_color = get_color_rgba(object.classification);

        struct PathSignature
        {
          double mid_x;
          double mid_y;
          double end_x;
          double end_y;
        };
        std::vector<PathSignature> accepted_sigs;
        accepted_sigs.reserve(3);

        auto compute_sig = [horizon_sec](
                             const autoware_perception_msgs::msg::PredictedPath & p) -> PathSignature {
          const size_t n = p.path.size();
          const double dt =
            static_cast<double>(p.time_step.sec) + static_cast<double>(p.time_step.nanosec) * 1e-9;
          size_t end_idx = n - 1;
          if (dt > 1e-4 && horizon_sec > 0.0) {
            end_idx = std::min(end_idx, static_cast<size_t>(std::ceil(horizon_sec / dt)));
          }
          const size_t mid_idx = end_idx / 2;
          return {
            p.path[mid_idx].position.x, p.path[mid_idx].position.y, p.path[end_idx].position.x,
            p.path[end_idx].position.y};
        };

        for (const size_t idx : sorted_indices) {
          if (accepted_sigs.size() >= 3) {
            break;
          }
          const auto & predicted_path = paths[idx];
          if (predicted_path.path.size() < 2) {
            continue;
          }
          if (!accepted_sigs.empty() && predicted_path.confidence < min_conf) {
            continue;
          }

          const auto sig = compute_sig(predicted_path);
          bool is_duplicate = false;
          for (const auto & prev : accepted_sigs) {
            if (
              std::hypot(sig.mid_x - prev.mid_x, sig.mid_y - prev.mid_y) < 0.40 &&
              std::hypot(sig.end_x - prev.end_x, sig.end_y - prev.end_y) < 0.45) {
              is_duplicate = true;
              break;
            }
          }
          if (is_duplicate) {
            continue;
          }

          const bool is_primary = accepted_sigs.empty();
          const float rel_conf = std::clamp(predicted_path.confidence / max_conf, 0.0F, 1.0F);
          auto predicted_path_marker = get_hologram_predicted_path_marker_ptr(
            object.shape, predicted_path, class_color, is_primary, rel_conf);
          if (predicted_path_marker) {
            accepted_sigs.push_back(sig);
            auto predicted_path_marker_ptr = predicted_path_marker.value();
            predicted_path_marker_ptr->header = msg->header;
            predicted_path_marker_ptr->id =
              uuid_to_marker_id(object.object_id) + path_count * PATH_ID_CONSTANT;
            path_count++;
            markers.push_back(predicted_path_marker_ptr);
          }
        }
      }
    } else {
      for (const auto & predicted_path : object.kinematics.predicted_paths) {
        // Get marker for predicted path
        auto predicted_path_marker =
          get_predicted_path_marker_ptr(object.object_id, object.shape, predicted_path);
        if (predicted_path_marker) {
          auto predicted_path_marker_ptr = predicted_path_marker.value();
          predicted_path_marker_ptr->header = msg->header;
          predicted_path_marker_ptr->id =
            uuid_to_marker_id(object.object_id) + path_count * PATH_ID_CONSTANT;
          path_count++;
          markers.push_back(predicted_path_marker_ptr);
        }
      }
    }

    // Add bounding box footprint marker for each candidate path
    path_count = 0;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      auto predicted_path_footprint_marker =
        get_predicted_path_footprint_marker_ptr(object.object_id, object.shape, predicted_path);
      if (predicted_path_footprint_marker) {
        auto predicted_path_footprint_marker_ptr = predicted_path_footprint_marker.value();
        predicted_path_footprint_marker_ptr->header = msg->header;
        predicted_path_footprint_marker_ptr->id =
          uuid_to_marker_id(object.object_id) + path_count * PATH_ID_CONSTANT;
        path_count++;
        markers.push_back(predicted_path_footprint_marker_ptr);
      }
    }

    // Add confidence text marker for each candidate path
    path_count = 0;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (predicted_path.path.empty()) {
        continue;
      }
      auto path_confidence_marker =
        get_path_confidence_marker_ptr(object.object_id, predicted_path);
      if (path_confidence_marker) {
        auto path_confidence_marker_ptr = path_confidence_marker.value();
        path_confidence_marker_ptr->header = msg->header;
        path_confidence_marker_ptr->id =
          uuid_to_marker_id(object.object_id) + path_count * PATH_ID_CONSTANT;
        path_count++;
        markers.push_back(path_confidence_marker_ptr);
      }
    }
  }

  return markers;
}

void PredictedObjectsDisplay::processMessage(PredictedObjects::ConstSharedPtr msg)
{
  {
    std::unique_lock<std::mutex> lock(mutex);
    this->msg = msg;
  }
  condition.notify_one();
}

void PredictedObjectsDisplay::update(float wall_dt, float ros_dt)
{
  std::unique_lock<std::mutex> lock(mutex);

  if (!markers.empty()) {
    std::set new_marker_ids = std::set<rviz_default_plugins::displays::MarkerID>();
    for (const auto & marker : markers) {
      rviz_default_plugins::displays::MarkerID marker_id =
        rviz_default_plugins::displays::MarkerID(marker->ns, marker->id);
      add_marker(marker);
      new_marker_ids.insert(marker_id);
    }
    for (auto itr = existing_marker_ids.begin(); itr != existing_marker_ids.end(); itr++) {
      if (new_marker_ids.find(*itr) == new_marker_ids.end()) {
        deleteMarker(*itr);
      }
    }
    existing_marker_ids = new_marker_ids;

    markers.clear();
  }

  lock.unlock();

  ObjectPolygonDisplayBase<autoware_perception_msgs::msg::PredictedObjects>::update(
    wall_dt, ros_dt);
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::PredictedObjectsDisplay, rviz_common::Display)
