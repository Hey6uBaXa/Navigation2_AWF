cmake_minimum_required(VERSION 3.8)
project(avf_15_pallet_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_library(avf_15_pallet_bt_plugins SHARED
  src/set_fork_position_action.cpp
  src/get_pallet_pose_action.cpp
  src/dock_at_pallet_action.cpp
  src/undock_from_pallet_action.cpp
)

target_include_directories(avf_15_pallet_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(avf_15_pallet_bt_plugins
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_behavior_tree
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav2_msgs
)

install(TARGETS avf_15_pallet_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_behavior_tree bt_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(avf_15_pallet_bt_plugins)
ament_export_dependencies(
  rclcpp rclcpp_action behaviortree_cpp nav2_behavior_tree pluginlib
  std_msgs sensor_msgs geometry_msgs nav2_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
