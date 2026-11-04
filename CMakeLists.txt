cmake_minimum_required(VERSION 3.8)
project(rover_diff_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(rover_diff_controller SHARED src/rover_diff_controller.cpp)
target_include_directories(rover_diff_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(rover_diff_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_msgs)

pluginlib_export_plugin_description_file(controller_interface rover_diff_controller_plugin.xml)

install(TARGETS rover_diff_controller
  EXPORT export_rover_diff_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_export_targets(export_rover_diff_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface hardware_interface pluginlib
  rclcpp_lifecycle realtime_tools geometry_msgs nav_msgs std_msgs tf2 tf2_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
