import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    docking_node = Node(
        package='robot_docking_server',
        executable='robot_docking_server',
    )

    return LaunchDescription([
        docking_node
    ])
