"""
Kinematics-only view: RViz plus slider GUI, no physics, no controllers.

This is the step you never skip. A wrong joint axis takes ten seconds to
spot here. The same mistake in Gazebo presents as an arm that thrashes and
flies apart, and at that point you cannot tell a kinematics error from an
inertia error from a gain error.

    ros2 launch tops_description view_robot.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('tops_description')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('tops_description'), 'urdf', 'tops_robot.urdf.xacro'])

    # controllers_config is irrelevant without Gazebo, but the xacro arg is
    # required, so it is given a harmless value.
    # ParameterValue(..., value_type=str) is mandatory, not stylistic.
    #
    # Command() yields the xacro output as a lazily-evaluated substitution.
    # Without an explicit type, launch tries to parse that output as YAML to
    # work out what kind of parameter it is, the URDF is XML rather than
    # YAML, and the whole launch aborts before a single node starts with
    # "Unable to parse the value of parameter robot_description as yaml".
    # The error names the parameter but not the cause, so it is worth a
    # comment: the robot description is a STRING and must be declared one.
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' controllers_config:=/dev/null',
                ' use_sim:=false',
            ]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(share, 'rviz', 'view_robot.rviz'),
            description='RViz configuration to load.'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
