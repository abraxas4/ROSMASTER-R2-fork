#!/usr/bin/env python3
"""
Map mode launch: SLAM + manual or autonomous exploration.
Uses slam_toolbox for mapping and Nav2 for navigation.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    r2_autonomy_dir = get_package_share_directory('r2_autonomy')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    yahboom_r2_bringup_dir = get_package_share_directory('yahboom_r2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox')
    use_cartographer = LaunchConfiguration('use_cartographer')
    autostart = LaunchConfiguration('autostart')
    use_odom_sanity = LaunchConfiguration('use_odom_sanity')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(r2_autonomy_dir, 'config', 'nav2_params_ackermann.yaml'),
        description='Full path to the ROS2 parameters file for Nav2'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(r2_autonomy_dir, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to SLAM toolbox parameters'
    )
    
    declare_use_slam_toolbox_cmd = DeclareLaunchArgument(
        'use_slam_toolbox',
        default_value='True',
        description='Whether to use slam_toolbox (true) or Cartographer (false)'
    )
    
    declare_use_cartographer_cmd = DeclareLaunchArgument(
        'use_cartographer',
        default_value='False',
        description='Whether to use Cartographer instead of slam_toolbox'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_odom_sanity_cmd = DeclareLaunchArgument(
        'use_odom_sanity',
        default_value='True',
        description='Use odom sanity checker to prevent large odom jumps'
    )
    
    # Odom sanity checker
    odom_sanity_checker_node = Node(
        package='r2_autonomy',
        executable='odom_sanity_checker.py',
        name='odom_sanity_checker',
        output='screen',
        condition=IfCondition(use_odom_sanity),
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_jump_distance': 5.0,
            'max_position': 100.0,
            'input_topic': '/odom',
            'output_topic': '/odom_safe'
        }]
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
    
    # Path reverse recovery
    path_reverse_recovery_node = Node(
        package='r2_autonomy',
        executable='path_reverse_recovery.py',
        name='path_reverse_recovery',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'path_buffer_duration': 5.0,
            'reverse_distance': 0.50,
            'reverse_speed': 0.15
        }]
    )
    
    # SLAM Toolbox (online async)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(use_slam_toolbox),
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Nav2 bringup (without localization, as SLAM provides map->odom)
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_slam_toolbox_cmd)
    ld.add_action(declare_use_cartographer_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_odom_sanity_cmd)
    
    ld.add_action(odom_sanity_checker_node)
    ld.add_action(slip_detector_node)
    ld.add_action(battery_monitor_node)
    ld.add_action(path_reverse_recovery_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(nav2_bringup_cmd)
    
    return ld
