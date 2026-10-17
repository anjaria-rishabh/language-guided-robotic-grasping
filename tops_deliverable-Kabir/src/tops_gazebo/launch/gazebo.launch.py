"""
Start Gazebo with the tabletop world, spawn the robot and the objects, and
bridge the few topics that genuinely have to cross.

Note what this launch file does NOT do: it does not start a controller
manager. Gazebo does that itself, from inside its own process, because the
gz_ros2_control plugin in the URDF reads controllers.yaml directly. That is
why tops_bringup only ever SPAWNS controllers -- the manager they attach to
already exists -- and why this has to be running first.

    ros2 launch tops_gazebo gazebo.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_share = get_package_share_directory('tops_gazebo')
    controllers_share = get_package_share_directory('tops_controllers')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(gazebo_share, 'worlds', 'tabletop.sdf')
    bridge_config = os.path.join(gazebo_share, 'config', 'bridge.yaml')
    controllers_yaml = os.path.join(controllers_share, 'config', 'controllers.yaml')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('tops_description'), 'urdf', 'tops_robot.urdf.xacro'])

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
                ' controllers_config:=', controllers_yaml,
                ' use_sim:=true',
            ]),
            value_type=str,
        )
    }

    gui = LaunchConfiguration('gui')

    # -r runs immediately instead of starting paused.
    #
    # -s is server-only (no rendered window). The batch runner depends on
    # this: 120 trials do not need a window, and not rendering roughly
    # doubles the real-time factor, which is the difference between a
    # one-hour batch and a two-hour one. It is appended only when
    # gui:=false, so the flag has to be computed rather than hard-coded.
    headless_flag = PythonExpression([
        "'' if '", gui, "'.lower() in ('true', '1', 'yes') else '-s '",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', headless_flag, world],
            # Without this, killing the launch leaves an orphaned Gazebo
            # server holding the DDS ports, and the NEXT trial in a batch
            # silently discovers it and uses the wrong simulation clock.
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'tops_panda',
            '-x', '0', '-y', '0', '-z', '0',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Render the Gazebo window. Set false for batch '
                        'trials; not rendering roughly doubles the '
                        'real-time factor.'),

        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,

    ])
