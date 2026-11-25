"""
Autonomous mapping: simulation + slam_toolbox + Nav2 + frontier explorer.

Usage:
  ros2 launch industrial_amr_bringup mapping.launch.py [explore:=false] [headless:=true]

With explore:=false, drive manually instead (teleop publishes cmd_vel directly):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the finished map into the navigation package:
  ros2 run nav2_map_server map_saver_cli \\
    -f src/industrial_amr_navigation/maps/industrial_room \\
    --ros-args -p save_map_timeout:=10.0

The default 2 s timeout can expire before DDS discovery finishes
("Failed to spin map subscription").
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
    pkg_explore = get_package_share_directory('industrial_amr_exploration')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'headless', default_value='false', description='Run Gazebo without GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='Start the frontier explorer.'),
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
        launch_arguments={'slam': 'true', 'use_sim_time': use_sim_time}.items(),
    )

    explorer = Node(
        package='industrial_amr_exploration',
        executable='frontier_explorer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('explore')),
        parameters=[
            os.path.join(pkg_explore, 'config', 'frontier_explorer.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(declare_args + [sim, nav2, explorer, rviz])
