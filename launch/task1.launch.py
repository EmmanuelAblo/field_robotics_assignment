#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # turtlebot3_gazebo package
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    # path to turtlebot3 world launcher
    gazebo_launch = os.path.join(tb3_pkg, 'launch', 'empty_world.launch.py')

    # Our controller node
    controller_node = Node(
        package='field_robotics_assignment',
        executable='pose_circle_controller',
        name='pose_circle_controller',
        output='screen',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'gui': 'true'}.items() 
        ),
        controller_node
    ])
