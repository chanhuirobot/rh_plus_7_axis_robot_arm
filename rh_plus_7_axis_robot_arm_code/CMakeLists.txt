################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(rh_plus_7_axis_robot_arm_code)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rh_plus_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(servo_control_node
  src/servo/servo_command.cpp
  src/servo/servo_control.cpp)
ament_target_dependencies(servo_control_node
  "rh_plus_interface" "rclcpp" "sensor_msgs")

add_executable(servo_info_client_node
  src/servo/servo_info_client.cpp)
ament_target_dependencies(servo_info_client_node
  "rh_plus_interface" "rclcpp")

add_executable(servo_info_server_node
  src/servo/servo_command.cpp
  src/servo/servo_info_server.cpp)
ament_target_dependencies(servo_info_server_node
  "rh_plus_interface" "rclcpp")

# install cpp executables
install(TARGETS
  servo_control_node
  servo_info_client_node
  servo_info_server_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()
