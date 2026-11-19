"""2D Cartographer mapping for R2. Lidar only. No camera, no wheel-odom TF.

Set load_state_filename to continue an existing pbstream (patrol map update).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    load_state_filename = LaunchConfiguration('load_state_filename')
    load_frozen_state = LaunchConfiguration('load_frozen_state')
    start_traj = LaunchConfiguration('start_trajectory_with_default_topics')

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_launch_path, '/laser_bringup_no_odom_launch.py']
        )
    )

    speed_guard = ExecuteProcess(
        cmd=['python3', os.path.join(script_dir, '..', 'mapping_speed_guard.py')],
        output='screen',
    )

    tf_to_odom = ExecuteProcess(
        cmd=['python3', os.path.join(script_dir, '..', 'tf_to_odom.py')],
        output='screen',
    )
    slip_stop = ExecuteProcess(
        cmd=['python3', os.path.join(script_dir, '..', 'slip_stop.py')],
        output='screen',
    )

    carto_base = [
        '-configuration_directory', lua_dir,
        '-configuration_basename', 'r2_cartographer.lua',
    ]

    cartographer_new = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=carto_base,
        remappings=[('scan', '/scan')],
        condition=UnlessCondition(
            PythonExpression(["'", load_state_filename, "' != ''"])
        ),
    )

    cartographer_continue = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=carto_base + [
            '-load_state_filename', load_state_filename,
            '-load_frozen_state', load_frozen_state,
            '-start_trajectory_with_default_topics', start_traj,
        ],
        remappings=[('scan', '/scan')],
        condition=IfCondition(
            PythonExpression(["'", load_state_filename, "' != ''"])
        ),
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
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument(
            'load_frozen_state',
            default_value='false',
            description='true keeps old submaps frozen; false lets occupancy update',
        ),
        DeclareLaunchArgument('start_trajectory_with_default_topics', default_value='true'),
        laser_bringup_launch,
        speed_guard,
        tf_to_odom,
        slip_stop,
        cartographer_new,
        cartographer_continue,
        occupancy_grid_node,
    ])
