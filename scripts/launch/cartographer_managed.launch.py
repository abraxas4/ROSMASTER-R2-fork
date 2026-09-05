"""2D Cartographer mapping for R2. Lidar only. No camera, no wheel-odom TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_launch_path = os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'launch'
    )
    script_dir = os.path.dirname(__file__)
    lua_dir = os.path.abspath(os.path.join(script_dir, '..', 'params'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_launch_path, '/laser_bringup_no_odom_launch.py']
        )
    )

    speed_guard = ExecuteProcess(
        cmd=['python3', os.path.join(script_dir, '..', 'mapping_speed_guard.py')],
        output='screen',
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', lua_dir,
            '-configuration_basename', 'r2_cartographer.lua',
        ],
        remappings=[('scan', '/scan')],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='0.5'),
        laser_bringup_launch,
        speed_guard,
        cartographer_node,
        occupancy_grid_node,
    ])
