cmake_minimum_required(VERSION 3.14)
project(autoware_minimap_overlay_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED COMPONENTS Gui Network Svg)
set(CMAKE_AUTOMOC ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_style.cpp
  src/minimap_display.cpp
  src/minimap_mouse.cpp
  src/minimap_painter.cpp
  src/route_line.cpp
  src/tile_loader.cpp
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Gui
  Qt5::Network
  Qt5::Svg
)

# autoware_package() turns on -Werror, and the OGRE headers have warnings
if(TARGET rviz_ogre_vendor::OgreMain)
  get_target_property(OGRE_INCLUDE_DIRS rviz_ogre_vendor::OgreMain INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${OGRE_INCLUDE_DIRS})
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
  ament_add_gtest(test_${PROJECT_NAME}
    test/main.cpp
    test/test_map_style.cpp
    test/test_minimap_mouse.cpp
    test/test_minimap_painter.cpp
    test/test_route_line.cpp
    test/test_superellipse.cpp
    test/test_tile_loader.cpp
    test/test_web_mercator.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE
    "ICON_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}/icons\""
  )
endif()

ament_auto_package(INSTALL_TO_SHARE icons)
