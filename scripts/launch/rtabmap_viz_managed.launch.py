"""RTAB-Map viz — subscribe to rgbd_sync output (not raw camera topics)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'True',
        'Grid/RangeMin': '0.2',
        'Optimizer/GravitySigma': '0',
    }

    remappings = [
        ('rgbd_image', '/rgbd_image'),
        ('scan', '/scan'),
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),
    ])