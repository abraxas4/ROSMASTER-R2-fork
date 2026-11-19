#!/usr/bin/env python3
"""
Patrol mode launch: Localization on saved map + patrol waypoints.
Uses AMCL for localization and waypoint following for patrol.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    r2_autonomy_dir = get_package_share_directory('r2_autonomy')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(r2_autonomy_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(r2_autonomy_dir, 'config', 'nav2_params_ackermann.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr',
        default_value='True',
        description='Whether to launch the lifecycle manager'
    )
    
    # Slip detector
    slip_detector_node = Node(
        package='r2_autonomy',
        executable='slip_detector.py',
        name='slip_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'slip_threshold_linear': 0.10,
            'slip_threshold_angular': 0.15,
            'slip_duration_limit': 3.0,
            'check_rate': 10.0,
            'odom_topic': '/odom',
            'wheel_odom_topic': '/ackermann_steering_controller/odometry'
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
            'voltage_file': '/dev/shm/r2_battery_v',
            'low_battery_threshold': 10.5,
            'check_interval': 5.0,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0
        }]
    )
    
    # Patrol waypoint generator (auto-generate waypoints from map)
    patrol_waypoint_node = Node(
        package='r2_autonomy',
        executable='patrol_waypoint_generator.py',
        name='patrol_waypoint_generator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_waypoints': 10,
            'min_distance_between_waypoints': 1.5,
            'loop_patrol': True
        }]
    )
    
    # Nav2 bringup with localization
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_lifecycle_mgr': use_lifecycle_mgr
        }.items()
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_mgr_cmd)
    
    ld.add_action(slip_detector_node)
    ld.add_action(battery_monitor_node)
    ld.add_action(patrol_waypoint_node)
    ld.add_action(nav2_bringup_cmd)
    
    return ld
