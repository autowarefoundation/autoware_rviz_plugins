# autoware_perception_rviz_plugin

## Purpose

It is an rviz plugin for visualizing the result from perception module. This package is based on the implementation of the rviz plugin developed by Autoware.Auto.

See Autoware.Auto design documentation for the original design philosophy. [[1]](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/visualization/autoware_rviz_plugins)

<!-- Write the purpose of this package and briefly describe the features.

Example:
  {package_name} is a package for planning trajectories that can avoid obstacles.
  This feature consists of two steps: obstacle filtering and optimizing trajectory.
-->

## Input Types / Visualization Results

### DetectedObjects

#### Input Types

| Name | Type                                             | Description            |
| ---- | ------------------------------------------------ | ---------------------- |
|      | `autoware_perception_msgs::msg::DetectedObjects` | detection result array |

#### Visualization Result

![detected-object-visualization-description](./images/detected-object-visualization-description.jpg)

### TrackedObjects

#### Input Types

| Name | Type                                            | Description           |
| ---- | ----------------------------------------------- | --------------------- |
|      | `autoware_perception_msgs::msg::TrackedObjects` | tracking result array |

#### Visualization Result

Overwrite tracking results with detection results.

![tracked-object-visualization-description](./images/tracked-object-visualization-description.jpg)

### PredictedObjects

#### Input Types

| Name | Type                                              | Description             |
| ---- | ------------------------------------------------- | ----------------------- |
|      | `autoware_perception_msgs::msg::PredictedObjects` | prediction result array |

#### Visualization Result

Overwrite prediction results with tracking results.

![predicted-object-visualization-description](./images/predicted-object-visualization-description.jpg)

### TrafficLight

#### Input Types

| Name                | Type                                                    | Description                                 |
| ------------------- | ------------------------------------------------------- | ------------------------------------------- |
| Lanelet Map         | `autoware_map_msgs::msg::LaneletMapBin`                 | Map data containing traffic light positions |
| Traffic Light State | `autoware_perception_msgs::msg::TrafficLightGroupArray` | Traffic light recognition results           |

#### Visualization Result

The plugin visualizes traffic light states in two ways:

1. Text display showing the current state (RED, AMBER, GREEN, directional arrows)
2. 3D spheres representing the light bulbs, colored according to their state

![traffic-light-visualization-description](./images/traffic-light-visualization-description.jpg)

##### Customizable Features

- Text position offset (X, Y, Z)
- Text font size and color
- Text prefix
- Show/hide text display
- Show/hide bulb visualization
- Timeout for stale data
- Topic names for both input types

## References/External links

[1] <https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/tools/visualization/autoware_rviz_plugins>

## Future extensions / Unimplemented parts
