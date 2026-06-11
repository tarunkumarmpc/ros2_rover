cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

ament_package()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)
