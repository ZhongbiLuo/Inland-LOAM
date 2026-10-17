"""Start Inland-LOAM, structural map conversion and RViz; play bags separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    inland_share = get_package_share_directory('inland_loam')
    conversion_share = get_package_share_directory('mapconversion')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(inland_share, 'config', 'loam_config.yaml'),
            description='Absolute path to the Inland-LOAM parameter YAML file',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the supplied display configuration',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(inland_share, 'launch', 'run.launch.py')
            ),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(conversion_share, 'launch', 'map_conversion.launch.py')
            ),
        ),
    ])
