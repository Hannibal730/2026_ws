import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("utm_pkg")
    default_params = os.path.join(package_share, "config", "utm_params.yaml")
    default_rviz = os.path.join(package_share, "rviz", "utm_visualization.rviz")

    csv_file = LaunchConfiguration("csv_file")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    start_rviz = LaunchConfiguration("start_rviz")
    play_bag = LaunchConfiguration("play_bag")
    bag_path = LaunchConfiguration("bag_path")
    loop_bag = LaunchConfiguration("loop_bag")
    playback_rate = LaunchConfiguration("playback_rate")

    map_node = Node(
        package="utm_pkg",
        executable="dual_gnss_map",
        name="dual_gnss_map",
        output="screen",
        parameters=[params_file, {"csv_file": csv_file}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="utm_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )
    bag_player_once = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--rate",
                    playback_rate,
                    "--topics",
                    "/f9p/fix",
                    "/f9r/fix",
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            play_bag,
                            "'.lower() == 'true' and '",
                            loop_bag,
                            "'.lower() != 'true'",
                        ]
                    )
                ),
            )
        ],
    )
    bag_player_loop = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--rate",
                    playback_rate,
                    "--topics",
                    "/f9p/fix",
                    "/f9r/fix",
                    "--loop",
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            play_bag,
                            "'.lower() == 'true' and '",
                            loop_bag,
                            "'.lower() == 'true'",
                        ]
                    )
                ),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_file",
                default_value="",
                description="CSV produced by bag_to_enu_csv (required)",
            ),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("play_bag", default_value="false"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("loop_bag", default_value="false"),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            map_node,
            rviz_node,
            bag_player_once,
            bag_player_loop,
        ]
    )
