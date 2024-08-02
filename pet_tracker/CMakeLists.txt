cmake_minimum_required(VERSION 3.8)
project(pet_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)

include_directories(include)


add_executable(pet_tracker src/pet_tracker.cpp)
ament_target_dependencies(pet_tracker rclcpp OpenCV cv_bridge geometry_msgs sensor_msgs std_msgs yolov8_msgs)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  pet_tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
