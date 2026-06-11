cmake_minimum_required(VERSION 3.8)
project(rover_base)

# Set C++ standard explicitly for ROS2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings (directory level for ROS2 best practice)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)


# Print OpenCV include dirs for debugging
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")

# camera_lift_control
add_executable(camera_lift_control src/camera_lift.cpp)
ament_target_dependencies(camera_lift_control
  rclcpp
  rclcpp_action
  can_msgs
  std_msgs
  rover_msgs
)
target_include_directories(camera_lift_control PUBLIC include)

# rover_lift_control
add_executable(rover_lift_control src/rover_lift.cpp)
ament_target_dependencies(rover_lift_control
  rclcpp
  rclcpp_action
  can_msgs
  std_msgs
  rover_msgs
)
target_include_directories(rover_lift_control PUBLIC include)

# rover_wheel_control
add_executable(rover_wheel_control src/rover_wheel.cpp)
ament_target_dependencies(rover_wheel_control
  rclcpp
  can_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)
target_include_directories(rover_wheel_control PUBLIC include)

# guidance_mode (with OpenCV and ArUco)
add_executable(guidance_mode src/guidance_mode.cpp)
ament_target_dependencies(guidance_mode
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2
  tf2_geometry_msgs
  image_transport

  OpenCV
)
target_include_directories(guidance_mode PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(guidance_mode
  ${OpenCV_LIBRARIES}
)

# Install executables
install(TARGETS
  camera_lift_control
  rover_lift_control
  rover_wheel_control
  guidance_mode
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers if you have any
install(DIRECTORY include/
  DESTINATION include/
)

# Install config and launch files if present
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
