cmake_minimum_required(VERSION 3.5)
project(robotarm_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig)

pkg_check_modules(hidapi-hidraw REQUIRED IMPORTED_TARGET hidapi-hidraw)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/robotarm_system.cpp
  src/robotarm.cpp
  #src/robotarm_usb.cpp
  src/robotarm_serial.cpp
  src/serial_servo_bus.cpp
)

# 순수 조인트 테스트... 절대로 로봇에 물려서 작동하면 안됨!!!
# 모터 7개 단독으로 있을 때만 사용
add_executable(test_node
  src/test.cpp

  src/robotarm_system.cpp
  src/robotarm.cpp
  src/robotarm_serial.cpp
  src/serial_servo_bus.cpp
)

target_include_directories(
  test_node
  PRIVATE
  include
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)


ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  hidapi-hidraw
)

ament_target_dependencies(
  test_node
  hardware_interface
  rclcpp
)

target_link_libraries(${PROJECT_NAME} PkgConfig::hidapi-hidraw)

pluginlib_export_plugin_description_file(hardware_interface robotarm_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
