# my_turtlebot3
Autonomous exploration, navigation and docking of turtlebot3 in gazebo simulation

<!-- USAGE -->

## Usage New : with new bringup method

We can now start the robot with one script.
If no map then it automatically explores the environment, creates the map, saves it and goes to dock.
If map exists robot will load the map and will be ready for navigation
#### Run the script
```sh
./src/robot_controller/script/start_simulation.sh
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
## Pet Tracker using Behaviour Tree
Track a pet such as cat or dog using yolov8 and cv2, It can follow a person too.

1. Start Gazebo simulation and start yolov8 with tracker
```sh
ros2 launch turtlebot_bringup simulated_robot.launch.py
```
2. Start navigation using a known map.
```sh
ros2 launch turtlebot3_gazebo navigation2.launch.py map:=src/turtlebot3_gazebo/maps/map_my_house.yaml
```
3. Run groot to see the behaviour tree working
```sh
ros2 run groot Groot
```
4. Start Pet tracker node
```sh
ros2 run robot_bt_pet_tracker pet_tracker_main
```
5. To move a pet in Gazebo. First add the cat model in Gazebo.
```sh
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap /cmd_vel:=/demo/cmd_demo
```