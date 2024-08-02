cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Docking.action"
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  DEPENDENCIES std_msgs
)

# Install message files
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
