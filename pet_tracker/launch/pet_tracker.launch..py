import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pet_tracker_node = Node(
        package='pet_tracker',
        executable='pet_tracker',
    )

    return LaunchDescription([
        pet_tracker_node
    ])
