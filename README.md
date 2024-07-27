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
3. Intialize AMCL position with robot initial pose
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