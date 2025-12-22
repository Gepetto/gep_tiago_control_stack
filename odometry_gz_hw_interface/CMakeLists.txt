cmake_minimum_required(VERSION 3.8)
project(odometry_gz_hw_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_ros2_control REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/odometry_hardware.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  pluginlib
  hardware_interface
  gz_ros2_control
)

pluginlib_export_plugin_description_file(gz_ros2_control plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
