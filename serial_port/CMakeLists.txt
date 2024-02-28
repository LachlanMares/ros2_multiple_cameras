cmake_minimum_required(VERSION 3.8)
project(serial_port)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/headers
)

add_executable(serial_port_test src/serial_port_test.cpp)
ament_target_dependencies(serial_port_test rclcpp)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  serial_port_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
