cmake_minimum_required(VERSION 3.8)
project(robot_bt_pet_tracker)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(dependencies
    rclcpp
    rclcpp_action
    behaviortree_cpp_v3
    action_msgs
    geometry_msgs
    nav2_msgs
    std_msgs
    ament_index_cpp
    yolov8_msgs
    sensor_msgs
    OpenCV
    cv_bridge
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(robot_begin_tacker_bt_node SHARED src/robot_bt_pet_tracker/BeginTracker.cpp)
add_library(robot_patrol_bt_node SHARED src/robot_bt_pet_tracker/Patrol.cpp)
add_library(robot_move_bt_node SHARED src/robot_bt_pet_tracker/Move.cpp)
add_library(robot_get_waypoint_bt_node SHARED src/robot_bt_pet_tracker/GetWaypoint.cpp)
add_library(robot_follow_bt_node SHARED src/robot_bt_pet_tracker/Follow.cpp)
add_library(robot_find_pet_bt_node SHARED src/robot_bt_pet_tracker/FindPet.cpp)

list(APPEND plugin_libs
  robot_begin_tacker_bt_node
  robot_patrol_bt_node
  robot_find_pet_bt_node
  robot_move_bt_node
  robot_get_waypoint_bt_node
  robot_follow_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(pet_tracker_main src/pet_tracker_main.cpp)
ament_target_dependencies(pet_tracker_main ${dependencies})
target_link_libraries(pet_tracker_main ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  pet_tracker_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
