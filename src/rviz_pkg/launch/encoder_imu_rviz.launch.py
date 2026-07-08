import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_share = get_package_share_directory('imu_pkg')
    encoder_share = get_package_share_directory('encoder_pkg')
    odom_share = get_package_share_directory('odom_pkg')
    rviz_share = get_package_share_directory('rviz_pkg')

    default_imu_params = os.path.join(imu_share, 'config', 'imu_params.yaml')
    default_encoder_params = os.path.join(encoder_share, 'config', 'encoder_params.yaml')
    default_odom_params = os.path.join(odom_share, 'config', 'odom_params.yaml')
    default_rviz = os.path.join(rviz_share, 'config', 'imu_odometry.rviz')

    imu_params_file = LaunchConfiguration('imu_params_file')
    encoder_params_file = LaunchConfiguration('encoder_params_file')
    odom_params_file = LaunchConfiguration('odom_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    imu_serial_port = LaunchConfiguration('imu_serial_port')
    imu_baudrate = LaunchConfiguration('imu_baudrate')
    encoder_serial_port = LaunchConfiguration('encoder_serial_port')
    encoder_baudrate = LaunchConfiguration('encoder_baudrate')

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_params_file',
            default_value=default_imu_params,
            description='Parameter file for the IMU node.',
        ),
        DeclareLaunchArgument(
            'encoder_params_file',
            default_value=default_encoder_params,
            description='Parameter file for the encoder node.',
        ),
        DeclareLaunchArgument(
            'odom_params_file',
            default_value=default_odom_params,
            description='Parameter file for odometry nodes.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config for encoder+IMU odometry visualization.',
        ),
        DeclareLaunchArgument(
            'imu_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial device for the IMU sensor.',
        ),
        DeclareLaunchArgument(
            'imu_baudrate',
            default_value='115200',
            description='Serial baudrate for the IMU sensor.',
        ),
        DeclareLaunchArgument(
            'encoder_serial_port',
            default_value='/dev/ttyACM0',
            description='Serial device for the encoder board.',
        ),
        DeclareLaunchArgument(
            'encoder_baudrate',
            default_value='115200',
            description='Serial baudrate for the encoder board.',
        ),
        Node(
            package='imu_pkg',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[
                imu_params_file,
                {
                    'serial_port': imu_serial_port,
                    'baudrate': ParameterValue(imu_baudrate, value_type=int),
                },
            ],
        ),
        Node(
            package='encoder_pkg',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen',
            parameters=[
                encoder_params_file,
                {
                    'serial_port': encoder_serial_port,
                    'baudrate': ParameterValue(encoder_baudrate, value_type=int),
                },
            ],
        ),
        Node(
            package='odom_pkg',
            executable='encoder_imu_odometry',
            name='encoder_imu_odometry',
            output='screen',
            parameters=[odom_params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
