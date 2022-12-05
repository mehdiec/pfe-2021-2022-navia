cmake_minimum_required(VERSION 3.8)
project(visualisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rcpputils REQUIRED)
find_package(image_geometry REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Local includes
include_directories(
  include 
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} 
  ${tf2_geometry_msgs_INCLUDE_DIRS} 
  ${visualization_msgs_INCLUDE_DIRS} 
  ${cv_bridge_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
)
# executables
add_executable(drone_viz src/drone_view.cpp)
ament_target_dependencies(drone_viz rclcpp std_msgs sensor_msgs cv_bridge tf2_geometry_msgs visualization_msgs image_geometry)

add_executable(trajectory_viz src/trajectory.cpp)
ament_target_dependencies(trajectory_viz rclcpp sensor_msgs)




install(TARGETS
  drone_viz
  trajectory_viz
  DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()