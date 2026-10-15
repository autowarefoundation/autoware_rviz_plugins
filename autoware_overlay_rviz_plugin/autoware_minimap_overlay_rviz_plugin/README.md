# autoware_minimap_overlay_rviz_plugin

Plugin for displaying a minimap overlay in RViz2 with vehicle position, goal pose, and route information.

Based on the [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization) package, under the 3-Clause BSD license.

## Purpose

This plugin provides a visual representation of the vehicle's position, the goal pose, and the planned route on a minimap overlay within RViz2. It is designed to enhance situational awareness by displaying relevant navigation information.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                                              |
| ------------------------- | ------------------------------------------------ | -------------------------------------------------------- |
| `/api/vehicle/kinematics` | `autoware_adapi_v1_msgs::msg::VehicleKinematics` | The geographic pose of the vehicle                       |
| `/api/routing/route`      | `autoware_adapi_v1_msgs::msg::Route`             | The route, the goal pose is taken from it                |
| `/api/routing/state`      | `autoware_adapi_v1_msgs::msg::RouteState`        | The routing state, goal and path show when it's SET      |
| `/planning/trajectory`    | `autoware_planning_msgs::msg::Trajectory`        | The planned trajectory, drawn as the path                |
| `/map/map_projector_info` | `autoware_map_msgs::msg::MapProjectorInfo`       | Converts the goal and the path to latitude and longitude |

## Parameters

### Core Parameters

#### VehicleMapDisplay

| Name               | Type   | Default Value   | Description                         |
| ------------------ | ------ | --------------- | ----------------------------------- |
| `Width`            | int    | 256             | Width of the overlay (max: 500 px)  |
| `Height`           | int    | 256             | Height of the overlay (max: 500 px) |
| `Left`             | int    | 10              | Left position of the overlay        |
| `Top`              | int    | 10              | Top position of the overlay         |
| `Alpha`            | float  | 0.0             | Transparency of the overlay         |
| `Background Color` | QColor | QColor(0, 0, 0) | Background color of the overlay     |
| `Zoom`             | int    | 15              | Zoom level of the map (15-18)       |
| `Latitude`         | float  | 0.0             | Latitude of the ego vehicle         |
| `Longitude`        | float  | 0.0             | Longitude of the ego vehicle        |
| `Goal Latitude`    | float  | 0.0             | Latitude of the goal pose           |
| `Goal Longitude`   | float  | 0.0             | Longitude of the goal pose          |

## Important Note

- The goal pose and the path are converted to geographic coordinates with the map projector info of the loaded map. All projector types except `Local` are supported.
- The Zoom parameter should be set according to the map tiles available for the specified zoom levels between 15-18.
- The Latitude, Longitude, Goal Latitude and Goal Longitude parameters update automatically from the vehicle's position and the route, and are there for debug purposes.
- The Width and Height parameters should be set according to the desired size of the overlay, the maximum value is 500 px.
- The Left and Top parameters should be set according to the desired position of the overlay, considering it is relative to the bottom-left corner of the RViz2 window.

## Assumptions / Known Limits

- The plugin assumes that the vehicle position and goal pose topics are being published.
- Map tiles must be fetched and available for the specified zoom levels.
- The position icon should rotate correctly according to the vehicle's orientation.

## Usage

1. Start `rviz2` and click the `Add` button under the `Displays` panel.

   ![select_add](./icons/add_map_display.png)

2. Under the `By display type` tab, select `autoware_minimap_overlay_rviz_plugin/VehicleMapDisplay` and press OK.

3. Edit size, position or zoom parameters as needed.

   ![select_topic_name](./icons/params.png)

## Example Configuration

Here is an example configuration for the VehicleMapDisplay:

```yaml
VehicleMapDisplay:
  Width: 256
  Height: 256
  Left: 10
  Top: 10
  Alpha: 0.5
  Background Color: [0, 0, 0]
  Zoom: 16
  Latitude: 35.0
  Longitude: 139.0
  Goal Latitude: 35.1
  Goal Longitude: 139.1
```

## License

This project is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.

The `goal.png` and `pos.png` icons are derived from [Material Symbols](https://fonts.google.com/icons), which are available under the [Apache License Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).
