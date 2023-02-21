cmake_minimum_required(VERSION 3.8)
project(alpha_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY gazebo meshes launch urdf rviz ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
