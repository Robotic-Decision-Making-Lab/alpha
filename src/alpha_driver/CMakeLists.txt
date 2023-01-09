cmake_minimum_required(VERSION 3.8)
project(alpha_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
)

set(library_name
  alpha_serial_driver
)

set(node_name
  serial_driver_node
)

add_library(${library_name} SHARED
  src/cobs.cpp
  src/crc.cpp
  src/packet.cpp
)
target_include_directories(${library_name} PRIVATE include)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${node_name} src/serial_client.cpp)
target_include_directories(${node_name} PRIVATE include)
target_link_libraries(${node_name} ${library_name})
ament_target_dependencies(${node_name} ${dependencies})

install(TARGETS ${library_name}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Run linters found in package.xml
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Specify unit tests to run
  ament_add_gtest(
    test_crc
    test/test_crc.cpp
  )
  target_link_libraries(test_crc ${library_name})
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
