cmake_minimum_required(VERSION 3.8)
project(pilz_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(trajectory_seed_generator REQUIRED)

add_executable(pilz_driver src/pilz_driver.cpp)
target_include_directories(pilz_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(pilz_driver jsoncpp)
target_compile_features(pilz_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pilz_driver
  "rclcpp"
  "moveit_core"
  "jsoncpp"
  "trajectory_seed_generator"
)

install(TARGETS pilz_driver
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


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
