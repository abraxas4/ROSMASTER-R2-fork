"""RTAB-Map mapping with configurable database path and delete-on-start."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_launch_path = os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'launch'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    database_path = LaunchConfiguration('database_path')
    delete_db = LaunchConfiguration('delete_db')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'database_path': database_path,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin': '0.2',
        'Optimizer/GravitySigma': '0',
    }

    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('odom', '/odom'),
    ]

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_launch_path, '/laser_bringup_launch.py']
        )
    )

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output='screen',
        parameters=[
            {
                'approx_sync': True,
                'approx_sync_max_interval': 0.01,
                'use_sim_time': use_sim_time,
                'qos': qos,
            }
        ],
        remappings=remappings,
    )

    slam_fresh_node = Node(
        condition=IfCondition(delete_db),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'],
    )

    slam_continue_node = Node(
        condition=UnlessCondition(delete_db),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[parameters],
        remappings=remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
        DeclareLaunchArgument(
            'database_path',
            default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        ),
        DeclareLaunchArgument(
            'delete_db',
            default_value='false',
            description='true = delete DB on start (new map). false = continue.',
        ),
        laser_bringup_launch,
        rgbd_sync_node,
        slam_fresh_node,
        slam_continue_node,
    ])