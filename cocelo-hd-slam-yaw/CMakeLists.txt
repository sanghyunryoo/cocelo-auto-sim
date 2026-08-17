cmake_minimum_required(VERSION 3.16)

project(autonomy_light)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WallAngle.msg"
  DEPENDENCIES std_msgs
)

add_executable(front_wall_angle_estimator
  src/front_wall_angle_estimator.cpp
)
ament_target_dependencies(front_wall_angle_estimator
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(front_wall_angle_estimator "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(front_wall_angle_estimator ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

add_executable(live_occupancy_mapper
  src/live_occupancy_mapper.cpp
)
ament_target_dependencies(live_occupancy_mapper
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(nav2_command_user_bridge
  src/nav2_command_user_bridge.cpp
)
ament_target_dependencies(nav2_command_user_bridge
  core
  geometry_msgs
  rclcpp
)

install(TARGETS front_wall_angle_estimator live_occupancy_mapper nav2_command_user_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  build.sh
  launch.sh
  scripts/monitor_wall_state.py
  scripts/nav2_goal_bridge.py
  scripts/nav2_wait_and_launch.sh
  scripts/nav_goal.sh
  scripts/frontier_validation.py
  scripts/setup_nav2.sh
  scripts/setup_livox_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
