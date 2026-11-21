#!/usr/bin/env python3
"""
Headless test launch for r2_autonomy.
Publishes fake sensor data and TF to verify nodes start correctly.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    r2_autonomy_dir = get_package_share_directory('r2_autonomy')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    test_mode = LaunchConfiguration('test_mode')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False'
    )
    
    declare_test_mode_cmd = DeclareLaunchArgument(
        'test_mode',
        default_value='map',
        choices=['map', 'patrol'],
        description='Which mode to test'
    )
    
    # Fake scan publisher
    fake_scan_node = Node(
        package='r2_autonomy',
        executable='fake_scan_publisher.py',
        name='fake_scan',
        output='screen'
    )
    
    # Fake odom publisher
    fake_odom_node = Node(
        package='r2_autonomy',
        executable='fake_odom_publisher.py',
        name='fake_odom',
        output='screen'
    )
    
    # Static TF publishers for test
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    static_tf_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link',
        arguments=['0', '0', '0.0215', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0.0435', '0', '0.11', '0', '0', '0', 'base_link', 'laser']
    )
    
    # Slip detector
    slip_detector_node = Node(
        package='r2_autonomy',
        executable='slip_detector.py',
        name='slip_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Battery monitor
    battery_monitor_node = Node(
        package='r2_autonomy',
        executable='battery_monitor.py',
        name='battery_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'voltage_file': '/tmp/fake_battery',
            'low_battery_threshold': 10.5
        }]
    )
    
    # Path reverse recovery
    path_reverse_recovery_node = Node(
        package='r2_autonomy',
        executable='path_reverse_recovery.py',
        name='path_reverse_recovery',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_test_mode_cmd)
    
    ld.add_action(fake_scan_node)
    ld.add_action(fake_odom_node)
    ld.add_action(static_tf_map_odom)
    ld.add_action(static_tf_odom_base)
    ld.add_action(static_tf_base_link)
    ld.add_action(static_tf_laser)
    
    ld.add_action(slip_detector_node)
    ld.add_action(battery_monitor_node)
    ld.add_action(path_reverse_recovery_node)
    
    return ld
