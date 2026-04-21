# autoware_topic_text_overlay_rviz_plugin

RViz panel plugin for overlaying text from selected ROS topics on top of the 3D view.

## Purpose

This plugin provides an RViz panel that lists supported text-like topics and overlays the selected
topic contents in the RViz render window. Multiple topics can be selected at the same time, and each
overlay block is labeled by its topic name.

## Supported Topics

The panel supports the following topic types:

- `autoware_internal_debug_msgs/msg/StringStamped`
- `autoware_internal_debug_msgs/msg/BoolStamped`
- `autoware_internal_debug_msgs/msg/Float32Stamped`
- `autoware_internal_debug_msgs/msg/Float64Stamped`
- `autoware_internal_debug_msgs/msg/Int32Stamped`
- `autoware_internal_debug_msgs/msg/Int64Stamped`
- `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped`
- `autoware_internal_debug_msgs/msg/Float64MultiArrayStamped`
- `autoware_internal_debug_msgs/msg/Int32MultiArrayStamped`
- `autoware_internal_debug_msgs/msg/Int64MultiArrayStamped`
- `visualization_msgs/msg/Marker`
- `visualization_msgs/msg/MarkerArray`

For `Marker` and `MarkerArray`, only markers with `TEXT_VIEW_FACING` type are displayed. For
`MarkerArray`, detected marker namespaces can be selected independently in the panel.

## Usage

1. Open RViz.
2. Add the `TopicTextOverlayPanel` panel.
3. Click `Refresh topics` or wait for the topic list to update.
4. Check the topics to overlay.
5. For `MarkerArray` topics, check the namespaces to display.

The overlay position, font size, maximum displayed text length, selected topics, and selected marker
namespaces are stored in the RViz config.
