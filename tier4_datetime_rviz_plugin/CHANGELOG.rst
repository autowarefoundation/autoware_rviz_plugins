^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tier4_datetime_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-11-27)
------------------
* feat: move autoware_polar_grid and tier4_datetime_rviz_plugin from Autoware Universe (`#48 <https://github.com/autowarefoundation/autoware_rviz_plugins/issues/48>`_)
  * feat: move autoware_polar_grid and tier4_datetime_rviz_plugin from Autoware Universe
  Copy the two packages from autoware_universe main at
  1258c0c4afe3c2cd6978aa17e2f2bc4c0c4ae3c1 without changes, except:
  - remove CHANGELOG.rst
  - set the version in package.xml to the version of this repository
  The package names and the plugin class names stay the same, so the
  existing RViz configuration files keep working.
  * fix: declare the dependencies of the moved packages
  - declare pluginlib in both packages. The code uses it, but only
  rviz_common brought it in.
  - autoware_polar_grid: declare rclcpp, which the code uses for
  rclcpp::Time. Remove rviz_default_plugins, which it does not use.
  - autoware_polar_grid: remove the unused rviz_rendering include.
  - autoware_polar_grid: remove the install(TARGETS) block with the ROS 1
  CATKIN_PACKAGE\_*_DESTINATION variables. They are empty in ament, and
  ament_auto_package already installs the library.
  - autoware_polar_grid: apply clang-format v19.1.6 of this repository.
  ---------
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
