"""
Autonomous navigation on a saved map: simulation + map_server/AMCL + Nav2.

Usage:
  ros2 launch industrial_amr_bringup navigation.launch.py [map:=/path/map.yaml]

AMCL starts at the default spawn pose.  Send goals with RViz "Nav2 Goal", or run
the waypoint circuit:
  ros2 run industrial_amr_exploration waypoint_runner --ros-args --params-file \\
    $(ros2 pkg prefix industrial_amr_exploration)/share/industrial_amr_exploration/config/waypoints.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('industrial_amr_bringup')
    pkg_gazebo = get_package_share_directory('industrial_amr_gazebo')
    pkg_nav = get_package_share_directory('industrial_amr_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run Gazebo without GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav, 'maps', 'industrial_room.yaml'),
            description='Map yaml to localise against.'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'nav2.launch.py')),
        launch_arguments={
            'slam': 'false',
            'map': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(declare_args + [sim, nav2, rviz])
