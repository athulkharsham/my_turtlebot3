# my_turtlebot3
Autonomous exploration, navigation and docking of turtlebot3 in gazebo simulation

<!-- USAGE -->
## Usage
1. Start the robot stack
```sh
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
2. Start Navigation
```sh
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=my_house.yaml
```
3. Initialize AMCL position with robot initial pose
```sh
$ python3 src/turtlebot3_gazebo/scripts/nav2_command.py
```
4. Run the docking action server
```sh
$ ros2 run robot_docking_server robot_docking_server
```
5. Send the action goal
```sh
$ ros2 action send_goal /dock_robot robot_interfaces/action/Docking "start_docking: true"
```
6. Make the robot move using keyboard
```sh
$ ros2 run turtlebot3_teleop teleop_keyboard
```
7. Run aruco node
```sh
$ ros2 run ros2_aruco aruco_node
```
## Frontier exploration
explore lite is used for autonomous exploration

To begin with

1. Start the robot stack
```sh
$ ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
2. Start slam
```sh
$ ros2 launch nav2_bringup slam_launch.py
```
3. Start Navigation
```sh
$ ros2 launch nav2_bringup navigation_launch.py
```
4. Run explore lite with param values
```sh
$ ros2 run explore_lite explore --ros-args --params-file src/m-explore-ros2/explore/config/params.yaml
```
5. Launch explore lite 
```sh
$ ros2 launch explore_lite explore.launch.py
```
6. Resume exploration if stuck 
```sh
$ ros2 topic pub /explore/resume std_msgs/msg/Bool "data: true"
```