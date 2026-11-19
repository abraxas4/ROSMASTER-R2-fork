"""Lidar + odom bringup for AMCL patrol. No camera, no Cartographer, no RTAB-Map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_launch_path = os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'launch'
    )
    script_dir = os.path.dirname(__file__)

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_launch_path, '/laser_bringup_launch.py']
        )
    )

    speed_guard = ExecuteProcess(
        cmd=['python3', os.path.join(script_dir, '..', 'mapping_speed_guard.py')],
        output='screen',
    )
    slip_stop = ExecuteProcess(
        cmd=['python3', os.path.join(script_dir, '..', 'slip_stop.py')],
        output='screen',
    )

    return LaunchDescription([
        laser_bringup_launch,
        speed_guard,
        slip_stop,
    ])
