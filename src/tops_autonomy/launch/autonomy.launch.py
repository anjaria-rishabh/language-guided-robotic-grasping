"""Perception and passive PBVS; no robot actuator adapter launched."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('tops_autonomy'),
                          'config','autonomy.yaml')
    return LaunchDescription([
        Node(package='tops_autonomy', executable='perception_node',
             name='tops_target_localizer', parameters=[params], output='screen'),
        Node(package='tops_autonomy', executable='ee_tf_node',
             name='tops_ee_tf_bridge', parameters=[params], output='screen'),
        Node(package='tops_autonomy', executable='pbvs_node',
             name='tops_pbvs_reference', parameters=[params], output='screen'),
        # Bridge is disabled by default until Kabir's actual message schema is checked.
    ])
