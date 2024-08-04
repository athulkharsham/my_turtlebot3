# my_turtlebot3
Autonomous exploration, navigation and docking of turtlebot3 in gazebo simulation

<!-- USAGE -->

## Usage New : with new bringup method
1. Start the robot stack
```sh
ros2 launch turtlebot_bringup simulated_robot.launch.py
```
2. Start Navigation and chose the rviz file from rviz directory in the package
```sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/turtlebot3_gazebo/maps/map_my_house.yaml
```
3. Initialize AMCL position with robot initial pose
```sh
ros2 run robot_controller robot_state_machine
```

## Usage Old : with most of the ros2 run to check most of the node independently
1. Start the robot stack
```sh
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
2. Start Navigation
```sh
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=my_house.yaml
```
3. Initialize AMCL position with robot initial pose
```sh
python3 src/turtlebot3_gazebo/scripts/nav2_command.py
```
4. Run the docking action server
```sh
ros2 run robot_docking_server robot_docking_server
```
5. Send the action goal
```sh
ros2 action send_goal /dock_robot robot_interfaces/action/Docking "start_docking: true"
```
6. Make the robot move using keyboard
```sh
ros2 run turtlebot3_teleop teleop_keyboard
```
7. Run aruco node
```sh
ros2 run ros2_aruco aruco_node
```
8. Robot state machine
```sh
ros2 run robot_controller robot_state_machine
```

## Frontier exploration
explore lite is used for autonomous exploration

To begin with

1. Start the robot stack
```sh
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
```
2. Start slam
```sh
ros2 launch nav2_bringup slam_launch.py use_sim_time:=True
```
3. Start Navigation
```sh
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
4. Run the robot control statemachine 
```sh
ros2 run robot_controller robot_state_machine
```
5. Launch explore lite 
```sh
ros2 launch explore_lite explore.launch.py
```
6. Resume exploration if stuck 
```sh
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: true"
```
## Tracker
Follow person, pet using yolov8 and cv2

To begin with

1. Start the human world with turtlebot3
```sh
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
2. Start yolov8 with tracker
```sh
ros2 launch yolobot_recognition launch_yolov8.launch.py
```
3. Start Per tracker node
```sh
ros2 run pet_tracker pet_tracker
```
4. Start RVIZ2
```sh
rviz2
```
4. To move person in gazebo
```sh
ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: -0.15
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```