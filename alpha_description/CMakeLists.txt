cmake_minimum_required(VERSION 3.8)
project(alpha_description)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY config meshes moveit2 rviz xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
