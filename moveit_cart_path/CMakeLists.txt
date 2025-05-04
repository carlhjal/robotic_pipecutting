cmake_minimum_required(VERSION 3.8)
project(moveit_cart_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(waypoint_server REQUIRED)

set(INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(moveit_cart_path_test 
  src/moveit_cart_path_test.cpp
)
# add_executable(cartesian_path_waypoint src/cartesian_path_waypoint)
target_include_directories(moveit_cart_path_test PUBLIC ${INCLUDE_DIRECTORIES})
target_compile_features(moveit_cart_path_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(moveit_cart_path_test)
ament_target_dependencies(
  moveit_cart_path_test
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(cart_path_waypoints
  src/cart_path_waypoints.cpp)

target_include_directories(cart_path_waypoints PUBLIC ${INCLUDE_DIRECTORIES})
target_compile_features(cart_path_waypoints PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cart_path_waypoints
  waypoint_server
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS 
  moveit_cart_path_test
  cart_path_waypoints
  DESTINATION lib/${PROJECT_NAME})

  

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
