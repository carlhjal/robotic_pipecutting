cmake_minimum_required(VERSION 3.8)
project(reach_utilizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(tinyxml2 REQUIRED)

# find_package(Boost REQUIRED COMPONENTS serialization)
# find_package(reach_ros REQUIRED)
# find_package(reach REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# set(REACH_DIR "${CMAKE_INSTALL_PREFIX}/../reach")

# include_directories(${REACH_DIR}/include)

# Find reach libraries manually
# find_library(REACH_LIB NAMES reach PATHS ${REACH_DIR}/lib)
# if(EXISTS "${REACH_DIR}/lib/cmake/reach/reach-config.cmake")
#     find_package(reach REQUIRED CONFIG PATHS "${REACH_DIR}/lib/cmake/reach")
# endif()

set(INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# add_executable(reach_db_server src/reach_db_server.cpp)
# target_include_directories(reach_db_server PUBLIC ${INCLUDE_DIRECTORIES})
# target_compile_features(reach_db_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(db_parser src/test.cpp)
target_include_directories(db_parser PUBLIC ${INCLUDE_DIRECTORIES})
target_compile_features(db_parser PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(db_parser tinyxml2)
ament_target_dependencies(db_parser
  rclcpp
  geometry_msgs
  Eigen3
  tf2
  Boost
  tinyxml2
  # "Boost REQUIRED COMPONENTS serialization"
)
# target_link_libraries(db_parser PRIVATE Boost::serialization)

install(TARGETS db_parser
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY results
  DESTINATION lib/${PROJECT_NAME}
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
