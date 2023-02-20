cmake_minimum_required(VERSION 3.8)
project(alpha_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo meshes launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
