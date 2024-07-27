import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", os.path.join(
                    get_package_share_directory("turtlebot3_gazebo"),
                    "rviz",
                    "turtlebot3.rviz"
                )
            ],
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])