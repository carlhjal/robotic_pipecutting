cmake_minimum_required(VERSION 3.8)
project(reach_ros_display_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(reach REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(Boost REQUIRED)
set(REACH_DIR "${CMAKE_INSTALL_PREFIX}/../reach")
set(REACH_ROS_DIR "${CMAKE_INSTALL_PREFIX}/../reach_ros")
set(BOOST_PLUGIN_LOADER_DIR "${CMAKE_INSTALL_PREFIX}/../boost_plugin_loader")

include_directories(${REACH_DIR}/include ${REACH_ROS_DIR}/include)

find_library(REACH_LIB NAMES reach PATHS ${REACH_DIR}/lib)
find_library(REACH_ROS_LIB NAMES reach_ros PATHS ${REACH_ROS_DIR}/lib)
find_library(BOOST_PLUGIN_LOADER_LIB NAMES boost_plugin_loader PATHS ${BOOST_PLUGIN_LOADER_DIR}/lib)

add_library(reach_ros_display_custom SHARED src/display/ros_display_custom.cpp)
target_include_directories(reach_ros_display_custom PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${boost_plugin_loader_INCLUDE_DIRS}§
)

find_path(boost_plugin_loader_INCLUDE_DIRS
  NAMES boost_plugin_loader/macros.h
  PATHS ${BOOST_PLUGIN_LOADER_DIR}/include
)

target_link_libraries(reach_ros_display_custom
  ${boost_plugin_loader_LIBRARIES}
  ${reach_LIBRARIES}
  ${rclcpp_LIBRARIES}
)
ament_target_dependencies(reach_ros_display_custom
  pluginlib
  rclcpp
  tf2_eigen
  yaml-cpp
  interactive_markers
  moveit_msgs
  reach
  boost_plugin_loader
)

pluginlib_export_plugin_description_file(reach_ros reach_ros_display_custom.xml)

install(TARGETS reach_ros_display_custom
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_ros_display_custom.xml
  DESTINATION share/reach_ros_display_custom
)

ament_export_dependencies(pluginlib rclcpp tf2_eigen yaml-cpp)
ament_export_include_directories(include)
ament_export_libraries(reach_ros_display_custom)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
