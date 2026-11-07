cmake_minimum_required(VERSION 3.8)
project(mars_rover_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)  # Add this line

# Include directories
include_directories(include)

add_executable(subscriber_obstacle_detector_executable src/subscriber_obstacle_detector.cpp)
ament_target_dependencies(subscriber_obstacle_detector_executable
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_executable(publish_mars_rover_move_executable src/publish_mars_rover_move.cpp)
ament_target_dependencies(publish_mars_rover_move_executable rclcpp geometry_msgs)

# Build plant detector library
add_library(plant_detector_lib src/plant_detector.cpp)
target_link_libraries(plant_detector_lib ${OpenCV_LIBRARIES})

add_executable(image_plant_detector_executable src/plant_detector_node.cpp)
ament_target_dependencies(image_plant_detector_executable
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  cv_bridge
  OpenCV
  custom_interfaces  # Add this line
)
target_link_libraries(image_plant_detector_executable plant_detector_lib)

add_executable(autonomous_exploration_executable src/autonomous_exploration.cpp)
ament_target_dependencies(autonomous_exploration_executable
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  subscriber_obstacle_detector_executable
  publish_mars_rover_move_executable            
  image_plant_detector_executable            
  autonomous_exploration_executable            
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()