import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mppi_bringup')
    default_params = os.path.join(package_share, 'config', 'mppi_controller.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_path_client = LaunchConfiguration('start_path_client')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mppi',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['planner_server', 'controller_server'],
        }],
    )

    path_client = Node(
        package='mppi_bringup',
        executable='mppi_path_client',
        name='mppi_path_client',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(start_path_client),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the MPPI controller parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.',
        ),
        DeclareLaunchArgument(
            'start_path_client',
            default_value='true',
            description='Start goal/csv path FollowPath client.',
        ),
        planner_server,
        controller_server,
        lifecycle_manager,
        path_client,
    ])
