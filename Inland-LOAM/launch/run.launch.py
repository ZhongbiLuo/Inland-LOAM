import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser


def shutdown_on_odometry_exit(event, context):
  if context.is_shutdown:
    return []
  return [EmitEvent(event=Shutdown(
    reason=f'Inland-LOAM stopped (exit code {event.returncode}); check its error above.'))]


def generate_launch_description():

  # Configure environment
  stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
  stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

  # Nodes Configurations
  default_config_file = os.path.join(get_package_share_directory('inland_loam'), 'config', 'loam_config.yaml')
  config_file = LaunchConfiguration('config_file')
  use_rviz = LaunchConfiguration('use_rviz')
  rviz_config = os.path.join(get_package_share_directory('inland_loam'), 'rviz', 'test.rviz')

  # Tf transformations
  transform_map = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_init_to_map',
    arguments=['0', '0', '0', '1.570795', '0', '1.570795', 'map', 'camera_init'],
  )
#        arguments=['0', '0', '0', '1.569195', '0.0004', '1.515995', 'map', 'camera_init'],
#  )

  transform_camera = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera',
    arguments=['0', '0', '0', '-1.570795', '-1.570795', '0', 'camera', 'base_link'],
  )

  # OctoMap stores (camera_init.x, -camera_init.z, camera_init.y).
  # This transform describes that existing projection; it does not rotate data.
  transform_ssm = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_init_to_ssm',
    arguments=['--x', '0', '--y', '0', '--z', '0',
               '--roll', '-1.5707963267948966', '--pitch', '0', '--yaw', '0',
               '--frame-id', 'camera_init', '--child-frame-id', 'ssm'],
  )

  transform_velodyne = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='velodyne_to_base_link',
    arguments=['0', '0', '0', '0', '0', '0','base_link','velodyne'],
  )

  # Inland-LOAM
  lego_loam_node = Node(
    package='inland_loam',
    executable='inland_loam',
    output='screen',
    parameters=[config_file],
    remappings=[('/lidar_points', '/velodyne_points')],
    # prefix=['gdbserver localhost:3000']
  )
  

  # Rviz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    output='screen',
    condition=IfCondition(use_rviz)
  )

  ld = LaunchDescription()
  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(stdout_colorized_envvar)
  ld.add_action(DeclareLaunchArgument(
    'config_file',
    default_value=default_config_file,
    description='Absolute path to the Inland-LOAM ROS 2 parameter YAML file'))
  ld.add_action(DeclareLaunchArgument(
    'use_rviz',
    default_value='true',
    description='Start RViz together with Inland-LOAM'))
  # Add nodes
  ld.add_action(RegisterEventHandler(OnProcessExit(
    target_action=lego_loam_node, on_exit=shutdown_on_odometry_exit)))
  ld.add_action(lego_loam_node)
  ld.add_action(transform_map)
  ld.add_action(transform_camera)
  ld.add_action(transform_ssm)
  ld.add_action(transform_velodyne)
  ld.add_action(rviz_node)

  return ld
