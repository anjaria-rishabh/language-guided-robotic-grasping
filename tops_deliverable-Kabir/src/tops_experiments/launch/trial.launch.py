"""
ONE trial: bring the stack up, run the grasp, write a CSV row, shut down.

Not usually run by hand -- `run_trials` launches this once per trial. It is
runnable on its own for debugging a single condition:

    ros2 launch tops_experiments trial.launch.py \
        controller:=impedance regime:=calibration seed:=17 gui:=true

The sequencing is entirely time-based here, unlike tops_system.launch.py's
event handlers, because this file has to guarantee TERMINATION. An event
chain that never fires leaves the process alive forever, and one hung trial
stalls a 120-trial batch overnight. Every delay below is generous, and the
final shutdown is unconditional.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

CONTROLLER_NAMES = {
    'impedance': 'cartesian_impedance_controller',
    'admittance': 'cartesian_admittance_controller',
    'baseline': 'joint_trajectory_controller',
}


def generate_launch_description():
    gazebo_share = get_package_share_directory('tops_gazebo')
    supervisor_share = get_package_share_directory('tops_supervisor')
    controllers_yaml = os.path.join(
        get_package_share_directory('tops_controllers'), 'config', 'controllers.yaml')

    controller = LaunchConfiguration('controller')
    regime = LaunchConfiguration('regime')
    seed = LaunchConfiguration('seed')
    trial_id = LaunchConfiguration('trial_id')
    object_label = LaunchConfiguration('object_label')
    object_xyz = LaunchConfiguration('object_xyz')
    output_csv = LaunchConfiguration('output_csv')
    timeout = LaunchConfiguration('timeout')
    gui = LaunchConfiguration('gui')

    controller_name = PythonExpression([
        '{', ', '.join(f"'{k}': '{v}'" for k, v in CONTROLLER_NAMES.items()), '}[',
        "'", controller, "'", ']',
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': gui}.items(),
    )

        # --param-file is required, and its absence is a trap.
        #
        # The Gazebo plugin hands controllers.yaml to the controller MANAGER,
        # which is enough for the manager to look up each controller's `type`
        # and load the plugin. It is NOT enough for the controller's own
        # parameters: in current ros2_control a controller node does not
        # inherit the manager's parameter file, so every per-controller block
        # in that YAML is simply never seen.
        #
        # The failure is quiet and misleading. The controller loads fine and
        # then fails on_configure complaining that `joints` is empty -- which
        # reads like a malformed YAML file rather than a file that was never
        # delivered. Passing it here is what actually applies it.
    def spawner(name, delay):
        return TimerAction(period=delay, actions=[Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60',
                       '--param-file', controllers_yaml])])

    supervisor = Node(
        package='tops_supervisor', executable='supervisor_node',
        name='tops_supervisor', output='screen',
        parameters=[
            os.path.join(supervisor_share, 'config', 'supervisor.yaml'),
            {'controller_name': controller_name, 'use_sim_time': True},
        ],
    )

    perception = Node(
        package='tops_perception_stub', executable='grasp_publisher',
        name='grasp_publisher', output='screen',
        parameters=[{
            'regime': regime,
            'seed': seed,
            'object_label': object_label,
            'object_xyz': object_xyz,
            'use_sim_time': True,
        }],
    )

    logger = Node(
        package='tops_experiments', executable='trial_logger',
        name='trial_logger', output='screen',
        parameters=[{
            'output_csv': output_csv,
            'mode': controller,
            'regime': regime,
            'seed': seed,
            'trial_id': trial_id,
            'object_label': object_label,
            'use_sim_time': True,
        }],
    )

    # Start and stop the trial through the client library rather than the
    # `ros2` CLI. See tops_experiments/trial_control.py for why: the CLI goes
    # through a graph-caching daemon that intermittently fails to find a
    # service that is demonstrably advertised, and the resulting trial looks
    # like a control failure instead of a tooling failure.
    begin = Node(
        package='tops_experiments', executable='trial_control', output='screen',
        arguments=['--action', 'begin'],
        parameters=[{'use_sim_time': True}],
    )

    # Always runs, even for a trial that failed or hung: a missing CSV row is
    # indistinguishable from a trial that was never run.
    finish = Node(
        package='tops_experiments', executable='trial_control', output='screen',
        arguments=['--action', 'finish'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='impedance',
                              choices=list(CONTROLLER_NAMES.keys())),
        DeclareLaunchArgument('regime', default_value='clean'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('trial_id', default_value='0'),
        DeclareLaunchArgument('object_label', default_value='red block'),
        # In the ARM BASE frame (panda_link0), matching the stub and the
        # trial plan. This default was left in WORLD coordinates after the
        # other two were corrected, which put the block 50 mm away in x --
        # more than the 14 mm of clearance the open jaw has around a 50 mm
        # block, so the gripper closed on empty air every time while every
        # other number in the log looked healthy.
        DeclareLaunchArgument('object_xyz', default_value='[0.50, -0.15, 0.026]'),
        DeclareLaunchArgument('output_csv', default_value='results/trials.csv'),
        DeclareLaunchArgument(
            'timeout', default_value='45.0',
            description='Seconds the grasp is allowed once everything is up.'),
        DeclareLaunchArgument('gui', default_value='false'),

        gazebo,

        # Controllers, in dependency order. State before command, always.
        spawner('joint_state_broadcaster', 8.0),
        spawner('force_torque_sensor_broadcaster', 12.0),
        TimerAction(period=16.0, actions=[Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=[controller_name, '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '60',
                       '--param-file', controllers_yaml])]),
        spawner('gripper_controller', 20.0),

        TimerAction(period=23.0, actions=[logger, supervisor, perception]),

        # 2 s of settling after the nodes appear lets the controller finish
        # capturing its F/T bias, which needs the arm still and untouched.
        TimerAction(period=26.0, actions=[begin]),

        # Record and quit, unconditionally.
        TimerAction(
            period=PythonExpression(['26.0 + ', timeout]),
            actions=[finish]),
        TimerAction(
            period=PythonExpression(['32.0 + ', timeout]),
            actions=[Shutdown(reason='trial complete')]),
    ])
