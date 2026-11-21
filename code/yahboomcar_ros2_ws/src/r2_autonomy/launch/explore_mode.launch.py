#!/usr/bin/env python3
"""Explore mode: frontier exploration on the current SLAM + Nav2 Ackermann stack.

Uses explore_lite (m-explore-ros2) if installed. This file only prepares the
launch; it must not be started with motors unless the operator explicitly
asks to depart.

Departure / recovery (already in nav2_params_ackermann.yaml, noted here
because they are easy to turn the wrong way on an Ackermann robot):

- use_rotate_to_heading must stay false. R2 cannot spin in place
  (vx=0 makes wz=0 at the MCU). "Turn toward the goal first" is done by
  the Hybrid-A* / Reeds-Shepp planner (reverse_penalty 1.8 prefers a
  forward arc), not by an in-place rotate behavior.
- allow_reversing may stay true so Nav2 can back up to escape. Reverse
  is for recovery, not the default departure.
- Battery: explore_battery_guard stops exploration below stop_voltage
  (default 10.5 V) on /battery_voltage. It pauses explore_lite and
  cancels Nav2 goals. It does not publish /cmd_vel.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import os


def _explore_node(context, *args, **kwargs):
    use_explore = LaunchConfiguration('use_explore').perform(context)
    if use_explore.lower() not in ('true', '1'):
        return []

    try:
        get_package_share_directory('explore_lite')
    except PackageNotFoundError:
        return [
            LogInfo(msg=(
                'explore_lite is not installed. Install m-explore-ros2 '
                '(Humble branch) and rebuild before launching explore mode. '
                'Battery guard still starts.'
            ))
        ]

    return [
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'costmap_topic': '/global_costmap/costmap',
                'costmap_updates_topic': '/global_costmap/costmap_updates',
                'visualize': True,
                'planner_frequency': LaunchConfiguration('explore_planner_frequency'),
                'progress_timeout': 30.0,
                'potential_scale': 3.0,
                'orientation_scale': 0.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.5,
                'min_frontier_size': LaunchConfiguration('min_frontier_size'),
                'return_to_init': False,
            }],
        )
    ]


def generate_launch_description():
    r2_autonomy_dir = get_package_share_directory('r2_autonomy')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    start_nav_stack = LaunchConfiguration('start_nav_stack')
    stop_voltage = LaunchConfiguration('stop_voltage')

    declarations = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(r2_autonomy_dir, 'config', 'nav2_params_ackermann.yaml'),
            description='Nav2 parameters (Ackermann RPP by default)',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(r2_autonomy_dir, 'config', 'slam_toolbox_params.yaml'),
            description='slam_toolbox parameters (mapping mode)',
        ),
        DeclareLaunchArgument(
            'start_nav_stack',
            default_value='True',
            description='Include map_mode (slam_toolbox + Nav2). Set false if the stack is already running.',
        ),
        DeclareLaunchArgument(
            'use_explore',
            default_value='True',
            description='Start explore_lite when the package is installed',
        ),
        DeclareLaunchArgument(
            'use_battery_guard',
            default_value='True',
            description='Pause explore and cancel Nav2 goals below stop_voltage',
        ),
        DeclareLaunchArgument(
            'stop_voltage',
            default_value='10.5',
            description='Battery voltage (V) on /battery_voltage at which exploration stops',
        ),
        DeclareLaunchArgument(
            'robot_base_frame',
            default_value='base_footprint',
            description='Robot base frame (must match Nav2 / SLAM)',
        ),
        DeclareLaunchArgument(
            'explore_planner_frequency',
            default_value='0.15',
            description='How often explore_lite picks a new frontier (Hz). Conservative default.',
        ),
        DeclareLaunchArgument(
            'min_frontier_size',
            default_value='0.75',
            description='Ignore frontiers smaller than this (m). Conservative default.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Autostart Nav2 when start_nav_stack is true',
        ),
    ]

    map_mode_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(r2_autonomy_dir, 'launch', 'map_mode.launch.py')
        ),
        condition=IfCondition(start_nav_stack),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam_params_file': slam_params_file,
            'autostart': LaunchConfiguration('autostart'),
            'use_slam_toolbox': 'True',
            'use_cartographer': 'False',
        }.items(),
    )

    battery_guard = Node(
        package='r2_autonomy',
        executable='explore_battery_guard.py',
        name='explore_battery_guard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_battery_guard')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'battery_topic': '/battery_voltage',
            'stop_voltage': stop_voltage,
            'explore_resume_topic': '/explore/resume',
            'cancel_nav_goals': True,
        }],
    )

    return LaunchDescription(
        declarations + [
            map_mode_cmd,
            OpaqueFunction(function=_explore_node),
            battery_guard,
        ]
    )
