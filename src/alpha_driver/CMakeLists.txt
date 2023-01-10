cmake_minimum_required(VERSION 3.8)
project(alpha_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/cobs.cpp
  src/crc.cpp
  src/packet.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(serial_driver_node src/serial_client.cpp)
target_include_directories(serial_driver_node PRIVATE include)
target_link_libraries(serial_driver_node ${PROJECT_NAME})
ament_target_dependencies(serial_driver_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Run linters found in package.xml
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Setup unit tests
  ament_add_gtest(
    test_crc
    test/test_crc.cpp
  )
  target_link_libraries(test_crc ${PROJECT_NAME})

  ament_add_gmock(
    test_cobs
    test/test_cobs.cpp
  )
  target_link_libraries(test_cobs ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
