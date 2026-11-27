^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_planning_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* refactor(autoware_rviz_plugins): use autoware_ament_auto_package (`#35 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/35>`_)
  Co-authored-by: github-actions <github-actions@github.com>
* Contributors: Vishal Chauhan

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
* fix(autoware_planning_rviz_plugin): memory leak in scene node management (`#19 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/19>`_)
* feat: visualize candidate trajectories (`#16 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/16>`_)
* Contributors: Yukihiro Saito, Yukinari Hisaki

0.1.0 (2025-06-28)
------------------
* feat: move autoware_planning_rviz_plugins from Autoware Universe (`#3 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/3>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome
