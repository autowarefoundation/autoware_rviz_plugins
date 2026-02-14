^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_perception_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* feat(autoware_perception_rviz_plugin): object mesh marker visualization (`#32 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/32>`_)
  * Add meshes with license
  * Modify cmake
  * Modify code for mesh, vehicle light and prediction with offset visualization
  * Update license and readme
  * Update license reference in readme
  * Update license in package.xml
  * Fix casting style
  * Add missing import
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  * Ignore certain spell checks
  * fix skipping of cspell check for LICENSE file
  ---------
  Co-authored-by: Jonas Krug <jonas.krug@dai-labor.de>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <ryohsuke.mitsudome@tier4.jp>
* feat(autoware_perception_rviz_plugin): replace fromBinMsg (`#31 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/31>`_)
* fix(perception object): fix orientation of position covariance (`#28 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/28>`_)
  * fix(autoware_perception_rviz_plugin): refine marker pose and format existence probability text
  * fix(autoware_perception_rviz_plugin): update marker pose handling to correctly set position
  * fix(autoware_perception_rviz_plugin): increase precision of existence probability text in marker
  ---------
* Contributors: JonasKrug, Sarun MUKDAPITAK, Taekjin LEE

0.3.0 (2025-11-16)
------------------
* fix(autoware_perception_rviz_plugin): fix `calc_path_line_list` (`#25 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/25>`_)
  * Fixed `calc_path_line_list`
  * Fixed is_valid_orientation
  * Added `#include <limits>`
  * Added a new condition `is_default_orientation`
  ---------
* Contributors: SakodaShintaro

0.2.0 (2025-08-14)
------------------
* fix(autoware_perception_rviz_plugin): improve traffic light visualization transform handling (`#17 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/17>`_)
* feat(autoware_perception_rviz_plugin): add traffic light display (`#15 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/15>`_)
  * feat(autoware_traffic_light_rviz_plugin): add traffic light display
  * update README.md
  * fix copyright
  * support without buble
  ---------
* Contributors: Yukinari Hisaki

0.1.0 (2025-06-28)
------------------
* feat: port autoware_perception_rviz_plugin from Autoware Universe (`#6 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/6>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome
