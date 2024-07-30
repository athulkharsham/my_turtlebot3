import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turtlebot3_gazebo"),
            "launch",
            "turtlebot3_my_world.launch.py"
        ),
    )
    
    docking = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_docking_server"),
            "launch",
            "robot_docking_server.launch.py"
        ),
    )
    
    visualization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("turtlebot3_gazebo"),
                "rviz",
                "turtlebot3.rviz"
            )
        ],
        parameters=[{"use_sim_time": True}],
    )

    aruco = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ros2_aruco"),
            "launch",
            "aruco_recognition.launch.py"
        ),
    )
    return LaunchDescription([
        gazebo,
        docking,
        visualization,
        TimerAction(
            period=3.0,
            actions=[aruco]
        ),
    ])