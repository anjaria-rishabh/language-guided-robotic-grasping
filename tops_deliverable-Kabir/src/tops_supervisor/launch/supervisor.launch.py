"""Start the supervisor alone, against an already-running controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tops_supervisor'), 'config', 'supervisor.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name', default_value='cartesian_impedance_controller',
            description='Which controller to drive. Switch this to '
                        'cartesian_admittance_controller for the comparison arm.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='tops_supervisor',
            executable='supervisor_node',
            name='tops_supervisor',
            output='screen',
            parameters=[
                config,
                {'controller_name': LaunchConfiguration('controller_name'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
