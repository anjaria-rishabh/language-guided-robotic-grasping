"""
The whole system, one command.

    ros2 launch tops_bringup tops_system.launch.py
    ros2 launch tops_bringup tops_system.launch.py controller:=admittance
    ros2 launch tops_bringup tops_system.launch.py controller:=baseline regime:=calibration

---------------------------------------------------------------------------
The ordering, and why it is not negotiable
---------------------------------------------------------------------------
1. Gazebo starts. Its gz_ros2_control plugin reads controllers.yaml from
   the URDF's <parameters> tag and starts a controller_manager INSIDE the
   Gazebo process. Nothing here starts one.
2. The robot spawns. Only now do the hardware interfaces exist.
3. joint_state_broadcaster is spawned first, so the rest of ROS 2 can see
   the arm's state before anything tries to command it.
4. The chosen controller is spawned. It claims the effort interfaces.
5. The supervisor and the perception stub start last, because they talk to
   the controller's services and would spend their first seconds warning
   about a controller that does not exist yet.

Each step's event handler waits on the PREVIOUS spawner exiting, rather
than on a fixed sleep. A sleep that is long enough on your machine is not
long enough on a loaded one, and the failure mode -- a spawner timing out
against a controller manager that was three seconds from ready -- looks
exactly like a configuration error.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#: Maps the friendly --controller value to the controller_manager name.
CONTROLLER_NAMES = {
    'impedance': 'cartesian_impedance_controller',
    'admittance': 'cartesian_admittance_controller',
    'baseline': 'joint_trajectory_controller',
}


def generate_launch_description():
    gazebo_share = get_package_share_directory('tops_gazebo')
    bringup_share = get_package_share_directory('tops_bringup')
    controllers_yaml = os.path.join(
        get_package_share_directory('tops_controllers'), 'config', 'controllers.yaml')

    controller = LaunchConfiguration('controller')
    regime = LaunchConfiguration('regime')
    seed = LaunchConfiguration('seed')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    # The selected controller's real name, resolved at launch time. Written
    # as one expression rather than three conditioned Nodes so there is a
    # single place the mapping lives.
    controller_name = PythonExpression([
        "{", ", ".join(f"'{k}': '{v}'" for k, v in CONTROLLER_NAMES.items()), "}[",
        "'", controller, "'", "]",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            # Generous, because on a cold start Gazebo is still loading
            # meshes and physics while this waits.
            '--controller-manager-timeout', '60',
            '--param-file', controllers_yaml,
        ],
    )

    ft_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['force_torque_sensor_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[controller_name, '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml],
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    supervisor = Node(
        package='tops_supervisor',
        executable='supervisor_node',
        name='tops_supervisor',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('tops_supervisor'),
                'config', 'supervisor.yaml'),
            {'controller_name': controller_name, 'use_sim_time': True},
        ],
    )

    perception = Node(
        package='tops_perception_stub',
        executable='grasp_publisher',
        name='grasp_publisher',
        output='screen',
        parameters=[{
            'regime': regime,
            'seed': seed,
            'use_sim_time': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'tops_system.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='impedance',
            choices=list(CONTROLLER_NAMES.keys()),
            description='impedance (the deliverable), admittance (the '
                        'comparison), or baseline (rigid joint trajectory '
                        'control -- the condition this project claims to beat).'),
        DeclareLaunchArgument(
            'regime', default_value='clean',
            choices=['clean', 'sensor_noise', 'calibration', 'displacement'],
            description='Which disturbance the perception stub injects.'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),

        gazebo,

        # Strict ordering: state before command, always.
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[ft_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=ft_broadcaster,
            on_exit=[arm_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper_controller,
            on_exit=[GroupAction([supervisor, perception, rviz_node])])),
    ])
