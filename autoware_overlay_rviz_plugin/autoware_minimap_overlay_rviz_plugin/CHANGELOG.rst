^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_minimap_overlay_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-11-27)
------------------
* feat(autoware_minimap_overlay_rviz_plugin): add the minimap overlay plugin (`#49 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/49>`_)
  * feat: new minimap overlay
  * move to visualization
  * remove unused demo
  * simplify cmakelists
  * clean up includes
  * fix cppcheck issues
  * cppcheck const
  * improve images
  * build(autoware_minimap_overlay_rviz_plugin): treat OGRE headers as system headers
  On Jazzy, autoware_package() turns on -Werror and the OGRE headers of
  rviz_ogre_vendor fail with -Wunused-parameter. Use the same fix as
  autoware_traffic_light_rviz_plugin in Universe.
  * fix(autoware_minimap_overlay_rviz_plugin): show the goal and the path at their map positions
  - Project local points with autoware_geography_utils and the received
  autoware_map_msgs/MapProjectorInfo instead of the removed tier4_map_msgs
  and a hard-coded map origin.
  - Compute overlay pixels from the full Web Mercator position. The old code
  used only the offset inside one tile, so the goal stayed near a fixed place.
  - Take the goal from /api/routing/route and the path from /planning/trajectory.
  - Draw the full path polyline and center the vehicle icon.
  * test(autoware_minimap_overlay_rviz_plugin): add tests for the Web Mercator pixel math
  * docs(autoware_minimap_overlay_rviz_plugin): update the inputs and the icon license
  Also set the package version to the version of this repository.
  * fix(autoware_minimap_overlay_rviz_plugin): fix the tile cache, rendering, and precision issues
  - Fix the deadlock in the TileField destructor (double lock of tile_mutex\_).
  TileField now uses one QNetworkAccessManager with a disk cache, evicts
  tiles far from the center, and retries failed tiles with a backoff.
  - Keep the ego position in double precision. Latitude and Longitude are
  now read-only properties, so setting them does not write a float back.
  - Redraw only when an input changes, load the position icon once, and
  repaint the tile image only when a tile arrives or the center moves.
  - Place the tiles with the same Web Mercator formula as the goal and path.
  - Show the OpenStreetMap attribution and add a contact to the User-Agent.
  - Clear the goal and the path when the route or the projector becomes
  unusable, skip non-finite positions, and stop fetching while disabled.
  - Remove the unused smoothing timer and the Tile class.
  - Declare the BSD-3-Clause license of overlay_utils, fix the dependencies,
  and correct the README parameter table.
  * refactor(autoware_minimap_overlay_rviz_plugin): use rviz_2d_overlay_plugins for the overlay
  Use rviz_2d_overlay_plugins::OverlayObject, like the other overlay
  plugins in this repository, and remove the copied overlay_utils and its
  BSD-3-Clause LICENSE file.
  * fix(autoware_minimap_overlay_rviz_plugin): name the plugin class and handle reset
  - Give the plugin class the name autoware_minimap_overlay_rviz_plugin/VehicleMapDisplay
  and a description, like the other overlay plugins.
  - Compute the heading so that a zero quaternion does not give NaN.
  - Clear the route, the path, and the goal on reset, and clear the goal
  properties when there is no goal.
  - Let only the unique_ptr own TileField, add CHANGELOG.rst, and drop tf2.
  * feat(autoware_minimap_overlay_rviz_plugin): rewrite the minimap as a navigation display
  The display class is now MinimapDisplay. The code has one class for each job: TileLoader downloads the map tiles, MinimapPainter draws the minimap, and MinimapDisplay connects them to RViz and to the topics.
  - The route line follows the centerlines of the route lanelets from the vehicle to the goal. Before, the minimap showed only the short planned trajectory.
  - The views are 3D, Heading Up, and North Up.
  - The map styles are Navigation Dark, Navigation Light, and OpenStreetMap. The navigation styles recolor the standard tiles in the Autoware colors, so that mainly the roads show.
  - Buttons on the minimap change the map style, the view, and the zoom level. The mouse wheel changes the zoom level, a drag moves the map, and a recenter button moves it back to the vehicle.
  - The shape is a squircle, a circle, or a rectangle.
  - A marker at the edge shows the direction of a goal that is outside the minimap.
  - All icons are SVG files, in a dark and a light set. The PNG icons are gone.
  - The design scales with the size of the minimap, and a large minimap loads the tiles of a higher zoom level.
  - The Displays panel shows status lines for the vehicle position, the map projector, the route, and the map tiles.
  - RViz Reset subscribes again, so that the transient local messages arrive again.
  - The tests cover the projection, the shape, the map styles, the route line, the tile loader, and the painter.
  * docs(autoware_minimap_overlay_rviz_plugin): describe the views, the map styles, and the controls
  The README now has a screenshot of the three views, the controls on the minimap, the inputs, the properties, and the known limits.
  * style(autoware_minimap_overlay_rviz_plugin): fit the dark map style to the RViz theme
  The dark navigation style used the navy of the brand colors, which looked purple beside the charcoal of the Autoware RViz theme and of the other overlays.
  - The land, the buttons, and the border of the dark style are now charcoal and grey.
  - The route, the goal pin, and the goal marker of the dark style are green, the color of the route in the 3D view.
  - The light style keeps the brand colors.
  * feat(autoware_minimap_overlay_rviz_plugin): move the map with the middle button and show purple icons
  - A drag with the middle mouse button moves the map, as it does in the 3D view of RViz. The left button still moves the map and clicks the buttons.
  - The vehicle arrow, the goal pin, and the goal marker of the dark style are purple with a white outline, so that they stand out from the green route.
  * docs(autoware_minimap_overlay_rviz_plugin): update the screenshot to the new colors
  * style(autoware_minimap_overlay_rviz_plugin): remove the border of the minimap
  The other Autoware overlays in the RViz view have no border. The grey border made the minimap look different from them.
  * style(autoware_minimap_overlay_rviz_plugin): add a thin charcoal border with a clean edge
  - The border has the background color of the other Autoware overlays, so it gives a little distance to the scene and no bright line.
  - The painter now cuts the shape out after it draws the border. With the clip path before, the light map styles left a thin bright line outside of the border.
  * style(autoware_minimap_overlay_rviz_plugin): set the default alpha to 0.6
  The scene of RViz now shows through the minimap a little, as it does through the other Autoware overlays.
  * chore(autoware_minimap_overlay_rviz_plugin): fix the spell check
  - Add "squircle" and "superellipse", the names of the minimap shape, to a local cspell file.
  - Use other words for the remaining unknown words, and setenv() in place of qputenv().
  * docs(autoware_minimap_overlay_rviz_plugin): update the screenshot to the border and the clean edge
  * style(autoware_minimap_overlay_rviz_plugin): set the default zoom level to 17
  Zoom level 17 shows the roads around the vehicle larger than level 16.
  * test(autoware_minimap_overlay_rviz_plugin): make the recenter button test independent of the font
  The test read a pixel below the recenter button. On Humble the attribution text is taller, and its label covered that pixel, so the test failed in CI. The test now reads a pixel in the upper part of the button, and the button is 6 pixels higher, away from the text.
  * fix(autoware_minimap_overlay_rviz_plugin): fix the findings of the self-review
  Mouse:
  - A new class `MinimapMouse` holds the mouse state, with tests. The minimap takes only a button that goes down on it. Before, it took the release of a button that went down in the 3D scene, so an RViz tool did not get the end of its drag.
  - A release that gets lost ends the drag at the next mouse move.
  - The wheel angles add up to one zoom step for each 120 units, so one touchpad gesture does not go through all zoom levels.
  Tiles:
  - One view needs at most 8 x 8 tiles. A minimap that needs more uses the tiles of a lower zoom level. Before, a minimap of 64 x 1024 pixels requested about 2700 tiles.
  - The new `Tile URL` property holds the address of the tile server.
  - A disabled display sends no requests. A request has a timeout of 30 seconds. A tile that the server refuses is not requested again while it stays in the view. A change of the map style keeps the delay of a tile that did not load.
  Display:
  - The subscription to the vector map and the map stay over a reset, so that RViz does not convert the map again at each loop of a rosbag.
  - A position outside of the latitude and longitude range, or an orientation that is not finite, gives a status error.
  - With a local projector, the minimap shows a message that names the projector.
  - The attribution text keeps a height of 9 pixels on a small minimap.
  Tests:
  - The tiles of the painter tests have a color that is not a background color, and new tests give 4 tiles different colors to check the place of the tiles in each view and at each tile zoom level.
  - A test reads a map message with `to_lanelet_map()`.
  * fix(autoware_minimap_overlay_rviz_plugin): fix the findings of the second review round
  - A wheel event with no vertical angle does not change the sum of the wheel angles.
  - A press outside of the minimap ends a drag whose release got lost.
  - New tests for `TileLoader::pause()`, `TileLoader::setUrlTemplate()`, and the delay of a failed tile after a change of the map style.
  - The README says that the attribution text stays the same for each tile address.
  ---------
  Co-authored-by: KhalilSelyan <khalil@leodrive.ai>
* Contributors: Mete Fatih Cırıt

0.6.0 (2026-06-30)
------------------

0.5.0 (2026-05-01)
------------------

0.4.0 (2026-02-14)
------------------

0.3.0 (2025-11-16)
------------------

0.2.0 (2025-08-14)
------------------

0.1.0 (2025-06-28)
------------------
