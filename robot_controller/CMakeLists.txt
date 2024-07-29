cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(robot_state_machine src/robot_state_machine.cpp)
ament_target_dependencies(robot_state_machine rclcpp std_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  robot_state_machine
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
