cmake_minimum_required(VERSION 3.8)
project(alpha_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config gazebo meshes moveit2 ros2_control rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
