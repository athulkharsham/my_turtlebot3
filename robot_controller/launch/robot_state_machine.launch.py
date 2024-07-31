import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    statemachine_node = Node(
        package='robot_controller',
        executable='robot_state_machine',
    )

    return LaunchDescription([
        statemachine_node
    ])
