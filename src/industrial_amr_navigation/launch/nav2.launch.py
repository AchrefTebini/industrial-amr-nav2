"""
Start the Nav2 navigation servers plus a map/localisation source.

  slam:=true   slam_toolbox builds the map online (mapping / exploration)
  slam:=false  map_server + AMCL localise against a saved map

Both modes share config/nav2_params.yaml, so the robot drives with the
same planner, controller and safety settings while mapping and navigating.

Both modes also start the keepout filter servers, which mark obstacles the
LIDAR can't see (the charger's base plate) in the costmaps.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('industrial_amr_navigation')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    slam_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')

    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_args = [
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Map with slam_toolbox instead of localising with AMCL.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav, 'maps', 'industrial_room.yaml'),
            description='Map yaml to localise against (slam:=false only).'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file.'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox parameters file (slam:=true only).'),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.join(pkg_nav, 'maps', 'charger_keepout.yaml'),
            description='Keepout mask yaml for the costmaps\' KeepoutFilter.'),
    ]

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'online_async_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': use_sim_time,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'localization_launch.py')),
        condition=UnlessCondition(slam),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
        }.items(),
    )

    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[params_file, {
            'yaml_filename': LaunchConfiguration('keepout_mask'),
            'use_sim_time': use_sim_time,
        }],
    )
    filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    filter_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
        }],
    )

    return LaunchDescription(declare_args + [
        slam_toolbox, localization, navigation,
        filter_mask_server, filter_info_server, filter_lifecycle_manager,
    ])
