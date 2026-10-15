cmake_minimum_required(VERSION 3.10)
project(autoware_minimap_overlay_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED COMPONENTS Core Gui Network Widgets)

set(CMAKE_AUTOMOC ON)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/goal_pose.cpp
  src/minimap.cpp
  src/path_overlay.cpp
  src/tile_field.cpp
  src/tile_provider.cpp
  src/include/goal_pose.hpp
  src/include/minimap.hpp
  src/include/path_overlay.hpp
  src/include/tile_field.hpp
  src/include/tile_provider.hpp
  src/include/web_mercator.hpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Network
  Qt5::Widgets
)

# Treat OGRE headers as system headers to suppress warnings
if(TARGET rviz_ogre_vendor::OgreMain)
  get_target_property(OGRE_INCLUDE_DIRS rviz_ogre_vendor::OgreMain INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OGRE_INCLUDE_DIRS})
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

if(BUILD_TESTING)
  ament_auto_add_gtest(test_web_mercator test/test_web_mercator.cpp)
  target_link_libraries(test_web_mercator Qt5::Core)
endif()

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(INSTALL_TO_SHARE icons)
