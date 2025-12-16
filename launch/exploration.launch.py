from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
import os

def generate_launch_description():
    # Paths to resources
    rviz_path = os.path.expanduser('~/phd_assignment_2842-2025/src/field_robotics_assignment/resource/Config_assignment.rviz')
    nav2_params_path = os.path.expanduser('~/phd_assignment_2842-2025/src/field_robotics_assignment/resource/nav2_params.yaml')
    mapper_params_path = os.path.expanduser('~/phd_assignment_2842-2025/src/field_robotics_assignment/resource/mapper_params_online_async.yaml')

    # TurtleBot3 Gazebo package
    tb3_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = os.path.join(tb3_pkg, 'launch', 'turtlebot3_world.launch.py')

    # Nav2 launch
    nav2_pkg = get_package_share_directory('nav2_bringup')
    nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')

    return LaunchDescription([

        # Launch Gazebo empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'gui': 'true'}.items()
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_params_path],
            output='screen'
        ),

        # Launch Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_path,
                'autostart': 'true'
            }.items()
        ),

        # Launch frontier explorer node
        Node(
            package='field_robotics_assignment',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen'
        ),

        # Launch RViz
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_path],
            shell=True
        ),
    ])
