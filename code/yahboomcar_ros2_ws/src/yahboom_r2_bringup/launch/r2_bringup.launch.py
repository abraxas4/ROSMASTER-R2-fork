"""ROSMASTER R2 full bringup on the common platform: ros2_control + sensors + EKF + Cartographer.

Each sensor can be switched on/off. TF owner of odom -> base_footprint is exactly one of:
  use_ekf:=true                    -> robot_localization EKF
  use_ekf:=false, use_slam:=true   -> Cartographer (lidar-only lua, as scripts/start_cartographer.sh)
  use_ekf:=false, use_slam:=false  -> ackermann_steering_controller
map -> odom is published only by Cartographer (use_slam:=true).

Example:
  ros2 launch yahboom_r2_bringup r2_bringup.launch.py use_lidar:=true use_slam:=true
  ros2 launch yahboom_r2_bringup r2_bringup.launch.py use_mock_hardware:=true use_lidar:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _b(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _s(context, name):
    return LaunchConfiguration(name).perform(context)


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('yahboom_r2_bringup')
    cfg = os.path.join(pkg, 'config')

    use_imu = _b(context, 'use_imu')
    use_ekf = _b(context, 'use_ekf')
    use_lidar = _b(context, 'use_lidar')
    use_camera = _b(context, 'use_camera')
    use_vo = _b(context, 'use_visual_odom')
    use_depth_scan = _b(context, 'use_depth_scan')
    use_slam = _b(context, 'use_slam')
    use_joy = _b(context, 'use_joy')
    lidar_type = _s(context, 'lidar_type')

    if (use_vo or use_depth_scan) and not use_camera:
        raise RuntimeError('use_visual_odom / use_depth_scan need use_camera:=true')
    if use_vo and not use_ekf:
        raise RuntimeError('use_visual_odom is fused by the EKF: set use_ekf:=true')
    if use_slam and not use_lidar:
        raise RuntimeError('use_slam needs use_lidar:=true')

    controller_odom_tf = (not use_ekf) and (not use_slam)
    actions = []

    # ---------------- ros2_control (step 1) ----------------
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'r2_control.launch.py')),
        launch_arguments={
            'use_mock_hardware': _s(context, 'use_mock_hardware'),
            'serial_device': _s(context, 'serial_device'),
            'use_imu': str(use_imu).lower(),
            'controller_odom_tf': str(controller_odom_tf).lower(),
            'controller_odom_to_odom': str(not use_ekf).lower(),
        }.items()))

    # ---------------- EKF: odom -> base_footprint ----------------
    if use_ekf:
        ekf_params = [os.path.join(cfg, 'ekf.yaml')]
        if use_vo:
            ekf_params.append({
                'odom1': '/vo',
                # twist only (vx, yaw rate); VO pose drifts and jumps on tracking loss
                'odom1_config': [False, False, False, False, False, False,
                                 True, False, False, False, False, True,
                                 False, False, False],
                'odom1_queue_size': 5,
                'odom1_differential': False,
                'odom1_twist_rejection_threshold': 2.0,
            })
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=ekf_params,
            remappings=[('odometry/filtered', '/odom')]))

    # ---------------- LiDAR (/scan) ----------------
    if use_lidar:
        if lidar_type == '4ROS':
            # same as yahboomcar_nav/launch/laser_bringup_launch.py (YDLIDAR 4ROS)
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_raw_launch.py'))))
            yaw = '0'
        else:
            name = 'sllidar_launch.py' if lidar_type == 'a1' else 'sllidar_s2_launch.py'
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sllidar_ros2'), 'launch', name))))
            yaw = '3.14'
        # static TF copied from laser_bringup_launch.py (frame 'laser', not URDF 'laser_link')
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_link_to_laser',
            arguments=['--x', '0.0435', '--y', '5.258E-05', '--z', '0.11', '--yaw', yaw,
                       '--frame-id', 'base_link', '--child-frame-id', 'laser']))

    # ---------------- Depth camera (Orbbec Astra Plus) ----------------
    if use_camera:
        # same as scripts/start_camera.sh
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('astra_camera'), 'launch', 'astro_pro_plus.launch.xml'))))

    if use_depth_scan:
        actions.append(Node(
            package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
            name='depth_to_scan', output='screen',
            parameters=[{'output_frame': _s(context, 'depth_frame'), 'range_min': 0.45,
                         'range_max': 4.0, 'scan_height': 10}],
            remappings=[('depth', '/camera/depth/image_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info'),
                        ('scan', '/depth_scan')]))

    if use_vo:
        # topics as yahboomcar_nav/launch/rtabmap_sync_launch.py
        actions.append(Node(
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
            parameters=[{'frame_id': 'base_footprint', 'odom_frame_id': 'vo_odom',
                         'publish_tf': False, 'approx_sync': True, 'wait_for_transform': 0.2,
                         'Reg/Force3DoF': 'true'}],
            remappings=[('rgb/image', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image', '/camera/depth/image_raw'),
                        ('odom', '/vo')]))

    # ---------------- Cartographer: map -> odom ----------------
    if use_slam:
        if use_ekf:
            basename = 'r2_cartographer_fused_depthscan.lua' if use_depth_scan else 'r2_cartographer_fused.lua'
        else:
            basename = 'r2_cartographer_lidar_only.lua'
        remaps = [('odom', '/odom')]
        if use_ekf and use_depth_scan:
            remaps += [('scan_1', '/scan'), ('scan_2', '/depth_scan')]
        else:
            remaps += [('scan', '/scan')]
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', cfg, '-configuration_basename', basename],
            remappings=remaps))
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node', output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '0.5']))

    # ---------------- Joystick (/cmd_vel) ----------------
    if use_joy:
        actions.append(Node(package='joy', executable='joy_node',
                            parameters=[{'deadzone': 0.05, 'autorepeat_rate': 30.0}]))
        actions.append(Node(package='yahboomcar_ctrl', executable='yahboom_joy_R2'))

    return actions


def generate_launch_description():
    args = [
        ('use_mock_hardware', 'false', 'mock_components/GenericSystem instead of the serial plugin'),
        ('serial_device', '/dev/myserial', 'STM32 expansion board USB serial'),
        ('use_imu', 'true', 'imu_sensor_broadcaster -> /imu/data_raw'),
        ('use_ekf', 'true', 'robot_localization EKF owns odom->base_footprint'),
        ('use_lidar', 'true', 'LiDAR driver -> /scan'),
        ('lidar_type', os.environ.get('RPLIDAR_TYPE', '4ROS'), '4ROS | a1 | s2'),
        ('use_camera', 'false', 'Orbbec Astra Plus (astra_camera)'),
        ('use_visual_odom', 'false', 'rtabmap rgbd_odometry -> /vo, fused by EKF'),
        ('use_depth_scan', 'false', 'depthimage_to_laserscan -> /depth_scan (2nd Cartographer scan)'),
        ('depth_frame', 'camera_depth_frame', 'frame of the depth image for depth_scan'),
        ('use_slam', 'false', 'Cartographer 2D: map->odom'),
        ('use_joy', 'false', 'joy_node + yahboom_joy_R2 -> /cmd_vel'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in args]
        + [OpaqueFunction(function=launch_setup)])
