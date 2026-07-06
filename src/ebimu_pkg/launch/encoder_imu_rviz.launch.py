import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ebimu_pkg')
    default_params = os.path.join(package_share, 'config', 'imu_only_params.yaml')
    default_rviz = os.path.join(package_share, 'config', 'imu_odometry.rviz')

    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for encoder and odometry nodes.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config for encoder+IMU odometry visualization.',
        ),
        Node(
            package='ebimu_pkg',
            executable='encoder_publisher',
            name='encoder_publisher',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='ebimu_pkg',
            executable='encoder_imu_odometry',
            name='encoder_imu_odometry',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
