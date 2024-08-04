#!/bin/bash

# Define the path to the map file
MAP_FILE="src/turtlebot3_gazebo/maps/robot_house.yaml"

# Define the delay time in seconds
DELAY=2

# Launch the simulated robot in a new terminal
gnome-terminal -- bash -c "ros2 launch turtlebot_bringup simulated_robot.launch.py; exec bash"

# Wait for the specified delay time
sleep $DELAY

# Check if the map file exists
if [ -f "$MAP_FILE" ]; then
    # Map file exists, launch navigation with the map in a new terminal
    echo "Map file found. Launching navigation with the map."
    gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo navigation2.launch.py use_sim_time:=True map:=$MAP_FILE; exec bash"
else
    # Map file does not exist, launch SLAM and exploration in new terminals
    echo "Map file not found. Launching SLAM and exploration."
    gnome-terminal -- bash -c "ros2 launch nav2_bringup slam_launch.py use_sim_time:=True; exec bash"
    gnome-terminal -- bash -c "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True; exec bash"
    gnome-terminal -- bash -c "ros2 launch explore_lite explore.launch.py; exec bash"
fi
