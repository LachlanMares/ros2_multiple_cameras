cmake_minimum_required(VERSION 3.8)
project(multiple_cameras)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(multiple_cameras REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraStatus.msg"
  "msg/MultipleCameraStatus.msg"
  "srv/ModifyCameraSettings.srv"
  "srv/ModifyCameraSettingsString.srv"
  "srv/CameraId.srv"
  DEPENDENCIES 
)

include_directories(include)

add_executable(multiple_camera_publisher src/multiple_camera_publisher.cpp)
ament_target_dependencies(multiple_camera_publisher rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV multiple_cameras)

add_executable(compressed_to_image_publisher src/compressed_to_image_publisher.cpp)
ament_target_dependencies(compressed_to_image_publisher rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV multiple_cameras)

add_executable(modify_camera_settings_client src/modify_camera_settings_client.cpp)
ament_target_dependencies(modify_camera_settings_client rclcpp multiple_cameras)

add_executable(modify_camera_settings_client_string src/modify_camera_settings_client_string.cpp)
ament_target_dependencies(modify_camera_settings_client_string rclcpp multiple_cameras)

# Export the launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  multiple_camera_publisher
  compressed_to_image_publisher
  modify_camera_settings_client
  modify_camera_settings_client_string
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/multiple_camera_publisher_python.py
  scripts/compressed_to_image_publisher_python.py
  scripts/define_camera_parameters.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
