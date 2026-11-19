"""Step 1: robot_state_publisher + ros2_control_node + controller spawners for ROSMASTER R2.

ros2 launch yahboom_r2_bringup r2_control.launch.py use_mock_hardware:=true
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def launch_setup(context, *args, **kwargs):
    distro = os.environ.get('ROS_DISTRO', 'jazzy')
    is_humble = distro in ('humble', 'iron')
    pkg = get_package_share_directory('yahboom_r2_bringup')
    controllers_yaml = os.path.join(
        pkg, 'config', 'controllers_humble.yaml' if is_humble else 'controllers_jazzy.yaml')

    use_mock = LaunchConfiguration('use_mock_hardware')
    use_imu = _as_bool(context, 'use_imu')
    odom_tf = _as_bool(context, 'controller_odom_tf')

    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('yahboomcar_description'), 'urdf', 'yahboomcar_R2.urdf.xacro']),
        ' use_ros2_control:=true',
        ' use_mock_hardware:=', use_mock,
        ' serial_device:=', LaunchConfiguration('serial_device'),
    ]), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='both',
        parameters=[{'robot_description': robot_description}])

    # topic remaps for controllers (they live inside ros2_control_node)
    ctrl_remaps = {
        'ackermann_steering_controller': [('~/tf_odometry', '/tf')],
        'imu_sensor_broadcaster': [('~/imu', '/imu/data_raw')],
    }
    if is_humble:
        ctrl_remaps['ackermann_steering_controller'].append(('~/reference_unstamped', '/cmd_vel'))
    if _as_bool(context, 'controller_odom_to_odom'):
        # no EKF: wheel odometry itself becomes /odom (Nav2 / patrol scripts read /odom)
        ctrl_remaps['ackermann_steering_controller'].append(('~/odometry', '/odom'))

    # node-name keyed override file: only enable_odom_tf changes at launch time
    override = tempfile.NamedTemporaryFile('w', prefix='r2_ctrl_override_', suffix='.yaml', delete=False)
    override.write('ackermann_steering_controller:\n  ros__parameters:\n'
                   f'    enable_odom_tf: {str(odom_tf).lower()}\n')
    override.close()
    cm_params = [controllers_yaml, override.name]

    node_remaps = []
    if is_humble:
        # Humble: controller_manager reads robot_description as a parameter, and has no
        # spawner --controller-ros-args, so remaps go on the process (absolute names).
        cm_params.insert(0, {'robot_description': robot_description})
        for ctrl, pairs in ctrl_remaps.items():
            node_remaps += [(f'/{ctrl}/{a[2:]}', b) for a, b in pairs]
    else:
        node_remaps.append(('~/robot_description', '/robot_description'))

    control_node = Node(
        package='controller_manager', executable='ros2_control_node', output='both',
        parameters=cm_params, remappings=node_remaps)

    def spawner(name):
        args = [name, '--controller-manager', '/controller_manager']
        if not is_humble and ctrl_remaps.get(name):
            ros_args = ' '.join(f'-r {a}:={b}' for a, b in ctrl_remaps[name])
            args += ['--controller-ros-args', ros_args]
        return Node(package='controller_manager', executable='spawner', output='screen', arguments=args)

    actions = [rsp, control_node,
               spawner('joint_state_broadcaster'),
               spawner('ackermann_steering_controller')]
    if use_imu:
        actions.append(spawner('imu_sensor_broadcaster'))
    if not is_humble:
        # Jazzy: controller wants TwistStamped on ~/reference
        actions.append(Node(
            package='yahboom_r2_bringup', executable='cmd_vel_stamper.py', output='screen',
            remappings=[('cmd_vel_in', '/cmd_vel'),
                        ('cmd_vel_out', '/ackermann_steering_controller/reference')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='true: mock_components/GenericSystem, false: Yahboom serial plugin'),
        DeclareLaunchArgument('serial_device', default_value='/dev/myserial'),
        DeclareLaunchArgument('use_imu', default_value='true', description='spawn imu_sensor_broadcaster'),
        DeclareLaunchArgument('controller_odom_tf', default_value='true',
                              description='ackermann controller publishes odom->base_footprint TF '
                                          '(set false when EKF or SLAM owns it)'),
        DeclareLaunchArgument('controller_odom_to_odom', default_value='true',
                              description='remap controller odometry to /odom (false when EKF publishes /odom)'),
        OpaqueFunction(function=launch_setup),
    ])
