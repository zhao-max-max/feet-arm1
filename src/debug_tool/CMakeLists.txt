cmake_minimum_required(VERSION 3.8)
project(debug_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(debug_tool_node
  src/debug_tool_node.cpp
)
target_compile_features(debug_tool_node PUBLIC cxx_std_17)
ament_target_dependencies(debug_tool_node
  rclcpp
  rclcpp_action
  robot_msgs
  std_msgs
)

install(TARGETS
  debug_tool_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(joint_slider_ui
  src/joint_slider_ui.cpp
)
target_compile_features(joint_slider_ui PUBLIC cxx_std_17)
ament_target_dependencies(joint_slider_ui
  rclcpp
  rclcpp_action
  robot_msgs
  std_msgs
)
target_link_libraries(joint_slider_ui
  Qt5::Widgets
)

install(TARGETS
  joint_slider_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
