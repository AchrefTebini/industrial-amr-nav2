"""
Start Gazebo Harmonic with the industrial room, spawn the AMR and bridge its topics.

Includes robot_state_publisher, so this is everything needed for a simulated
robot that the navigation stack can drive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('industrial_amr_gazebo')
    pkg_description = get_package_share_directory('industrial_amr_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_gazebo, 'worlds', 'industrial_room.sdf'),
            description='Full path to the SDF world file.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run the Gazebo server only, without the GUI.'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m].'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m].'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad].'),
    ]

    # Lets Gazebo resolve package:// URIs in the URDF.
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_description))

    # Server and GUI are started separately instead of via gz_sim.launch.py:
    # that file runs `gz sim` through a shell, so Ctrl-C reaches the shell but
    # not Gazebo, leaving orphaned gz processes behind.  The gzserver node
    # runs the simulation inside a regular ROS process instead.
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_server.launch.py')),
        launch_arguments={'world_sdf_file': LaunchConfiguration('world')}.items(),
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v', '2'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'use_sim': 'true'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'industrial_amr',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_gazebo, 'config', 'bridge.yaml'),
            'use_sim_time': use_sim_time,
        }],
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declare_args + [
        resource_path,
        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn_robot,
        bridge,
        image_bridge,
    ])
