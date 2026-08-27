cmake_minimum_required(VERSION 3.8)
project(avf_15_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  avf_15_hardware
  SHARED
  src/avf_15_system.cpp
)

target_compile_features(
  avf_15_hardware
  PUBLIC
  cxx_std_17
)

target_include_directories(
  avf_15_hardware
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  avf_15_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  avf_15_hardware_plugin.xml
)

install(
  TARGETS avf_15_hardware
  EXPORT export_avf_15_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES avf_15_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_avf_15_hardware
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()