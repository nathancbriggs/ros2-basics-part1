cmake_minimum_required(VERSION 3.8)
project(mars_rover_systems)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Create executable
add_executable(heartbeat_executable src/heartbeat.cpp)
add_executable(heartbeat_executable2 src/heartbeat2.cpp)
add_executable(temperaturemonitor_executable src/temperature_monitor.cpp)

# Add dependencies
ament_target_dependencies(heartbeat_executable rclcpp)
ament_target_dependencies(heartbeat_executable2 rclcpp)
ament_target_dependencies(temperaturemonitor_executable rclcpp)

# Install executables
install(TARGETS
  heartbeat_executable
  heartbeat_executable2
  temperaturemonitor_executable
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

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