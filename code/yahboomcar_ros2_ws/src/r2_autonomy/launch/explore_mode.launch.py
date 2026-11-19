#!/usr/bin/env python3
"""
Explore mode launch: Autonomous frontier exploration with SLAM.
Uses explore_lite for frontier-based exploration.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    r2_autonomy_dir = get_package_share_directory('r2_autonomy')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    # Include map mode (which includes SLAM + Nav2)
    map_mode_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(r2_autonomy_dir, 'launch', 'map_mode.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Explore node (explore_lite)
    # Note: explore_lite may need to be installed separately
    # For now, we provide params. If not available, user must install m-explore-ros2
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_base_frame': 'base_footprint',
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.33,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.5,
            'min_frontier_size': 0.5
        }],
        remappings=[
            ('/explore/goal', '/goal_pose')
        ]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(map_mode_cmd)
    ld.add_action(explore_node)
    
    return ld
