import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    inland_share = get_package_share_directory('inland_loam')
    mapconversion_share = get_package_share_directory('mapconversion')

    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')
    start_delay = LaunchConfiguration('start_delay')
    config_file = LaunchConfiguration('config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    shutdown_on_bag_end = LaunchConfiguration('shutdown_on_bag_end')

    inland = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inland_share, 'launch', 'run.launch.py')
        ),
        launch_arguments={
            'config_file': config_file,
            'use_rviz': use_rviz,
        }.items(),
    )
    converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mapconversion_share, 'launch', 'map_conversion.launch.py'
            )
        )
    )
    bag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--rate', rate,
            '--read-ahead-queue-size', '1000',
        ],
        output='screen',
    )
    playback = TimerAction(
        period=start_delay,
        actions=[bag_player],
    )
    automatic_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=[EmitEvent(event=Shutdown(reason='Demo bag finished'))],
        ),
        condition=IfCondition(shutdown_on_bag_end),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            description='Absolute path to the extracted Area A demo bag',
        ),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='ROS 2 bag playback rate',
        ),
        DeclareLaunchArgument(
            'start_delay', default_value='8.0',
            description='Seconds allowed for ROS discovery before playback',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                inland_share, 'config', 'loam_config.yaml'
            ),
            description='Absolute path to the Inland-LOAM YAML file',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz for the demo',
        ),
        DeclareLaunchArgument(
            'shutdown_on_bag_end', default_value='false',
            description='Stop all demo nodes when bag playback finishes',
        ),
        inland,
        converter,
        playback,
        automatic_shutdown,
    ])
