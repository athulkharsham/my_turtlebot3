cmake_minimum_required(VERSION 3.8)
project(robot_docking_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(robot_docking_server src/robot_docking_server.cpp)
ament_target_dependencies(robot_docking_server rclcpp rclcpp_action robot_interfaces geometry_msgs tf2_ros nav2_msgs tf2_geometry_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  robot_docking_server
  DESTINATION lib/${PROJECT_NAME}/
)



ament_package()
