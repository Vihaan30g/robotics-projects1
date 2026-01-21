cmake_minimum_required(VERSION 3.22)
project(manipulator1_moveit_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


#---
add_executable(joy_to_twist_node src/joy_to_twist.cpp)

ament_target_dependencies(joy_to_twist_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  joy_to_twist_node
  DESTINATION lib/${PROJECT_NAME}
)
#---



ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
