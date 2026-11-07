cmake_minimum_required(VERSION 3.8)
project(basics_ros2_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)   # CHANGED: needed for the Trigger service
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(plant_detector_basic src/plant_detector.cpp)
add_executable(plant_detector_multithreading src/plant_detector_multithreading.cpp)
add_executable(plant_detector_multithreading_callbackgroups src/plant_detector_multithreading_callbackgroups.cpp)
add_executable(green_detector_node src/green_detector_node.cpp)
add_executable(green_test src/green_test.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(plant_detector_basic
  rclcpp
  std_msgs
  std_srvs   # CHANGED
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(plant_detector_multithreading
  rclcpp
  std_msgs
  std_srvs   # CHANGED
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(plant_detector_multithreading_callbackgroups
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(green_detector_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_target_dependencies(green_test
  rclcpp
  std_msgs
  cv_bridge
  OpenCV
)

# Install executables
install(TARGETS
  plant_detector_basic
  plant_detector_multithreading
  plant_detector_multithreading_callbackgroups
  green_detector_node
  green_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()