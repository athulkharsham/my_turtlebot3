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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)


add_executable(pet_tracker src/pet_tracker.cpp)
add_executable(pet_tf_publisher src/pet_tf_publisher.cpp)

ament_target_dependencies(pet_tracker rclcpp OpenCV cv_bridge geometry_msgs sensor_msgs std_msgs yolov8_msgs)
ament_target_dependencies(pet_tf_publisher rclcpp OpenCV cv_bridge geometry_msgs sensor_msgs std_msgs yolov8_msgs tf2_ros tf2_geometry_msgs)


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  pet_tracker
  pet_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
