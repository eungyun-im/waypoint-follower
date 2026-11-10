"""waypoint_follower on its own, publishing straight to /erp42_racing/control_command."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('waypoint_follower'), 'config', 'tracking_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='waypoint_follower',
            executable='waypoint_follower_node',
            # Must match the top-level key in tracking_params.yaml.
            name='waypoint_follower',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
