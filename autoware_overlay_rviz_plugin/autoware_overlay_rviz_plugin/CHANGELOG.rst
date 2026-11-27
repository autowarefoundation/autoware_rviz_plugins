^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_overlay_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* feat: cool hologram theme (`#57 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/57>`_)
  * feat: add hologram 3D bounding box, trajectory ribbon, custom vector map display, and cool mode toggle tool
  Co-authored-by: Cursor <cursoragent@cursor.com>
  * feat(autoware_perception_rviz_plugin): add Hologram tapered ribbon style for predicted paths
  * fix(map): hide bicycle lane arrows, soften blue pavement fill, and deduplicate road_borders against lane bounds
  * feat(autoware_planning_rviz_plugin): add MissionRouteGoal hologram circle display
  * style(pre-commit): autofix
  * refactor: address review comments (remove theme toggle tool, support concave polygons, simplify hologram path, preserve defaults)
  * style(pre-commit): autofix
  * fix: fix polygon mesh shading/winding/fallback, preserve Fill polygon outline, and reset cached path
  * style(pre-commit): autofix
  * style: add missing <utility> and <memory> headers for cpplint
  ---------
  Co-authored-by: Cursor <cursoragent@cursor.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: kotaro-hihara

0.6.0 (2026-06-30)
------------------

0.5.0 (2026-05-01)
------------------

0.4.0 (2026-02-14)
------------------
* feat: use rviz_2d_overlay_plugins package for OverlayObject class (`#29 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/29>`_)
  * feat: use rviz_2d_overlay_plugins package for OverlayObject class
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome

0.3.0 (2025-11-16)
------------------

0.2.0 (2025-08-14)
------------------

0.1.0 (2025-06-28)
------------------
* feat: port autoware_overlay_rviz_plugin from Autoware Universe (`#7 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/7>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome
