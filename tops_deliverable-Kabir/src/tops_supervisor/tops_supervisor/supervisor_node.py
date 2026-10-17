#!/usr/bin/env python3
# Copyright 2026 Kabir Dhingra. MIT licence.
"""
The supervisor: the thin ROS layer around the grasp state machine.

Theory/09_Integration_FSM_and_Safety.md.

This node owns the sequencing and the interlocks. It does NOT own the
control law -- that is entirely inside the C++ controller, at 1 kHz, where
it belongs. The supervisor runs at 50 Hz and its job is to decide WHICH
target and WHICH gains, not to compute torque.

That division is the reason the safety argument holds. Anything the
supervisor does slowly (deciding, sequencing, logging) can afford to be in
Python. Anything that must never be late (the torque clamp, the wrench
cutoff inside the control law) is not here at all.

    ros2 run tops_supervisor supervisor_node
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, WrenchStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from tops_msgs.msg import ControllerDiagnostics, GraspTarget, SupervisorState
from tops_msgs.srv import SetControlMode, TriggerRetreat

from tops_supervisor.state_machine import (
    GraspStateMachine,
    Observation,
    State,
    Thresholds,
)

_STATE_NAMES = {s.value: s.name for s in State}


class SupervisorNode(Node):

    def __init__(self):
        super().__init__('tops_supervisor')

        self._declare_parameters()
        self._read_parameters()

        self.fsm = GraspStateMachine(self.thresholds)

        # Guards the observation fields written by subscription callbacks
        # and read by the timer. The GIL makes individual float assignments
        # atomic, but a consistent SET of them across one tick is not
        # guaranteed without this, and an observation assembled from two
        # different instants is exactly the kind of thing that produces a
        # transition nobody can reproduce.
        self._lock = threading.Lock()

        self._joint_state = None
        self._diagnostics = None
        self._latest_target = None
        self._target_stamp = None
        self._start_pose = None
        self._contact_force = 0.0
        self._start_requested = False
        self._abort_requested = False
        self._gripper_command = self.gripper_open
        self._last_state = None
        self._slow_loop_warned = False
        self._last_tick = None
        self._grasp_height = None
        self._last_width = None
        self._grasp_xy = None
        self._commanded = None
        self._goal = None
        self._tcp_xyz = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        cb_sub = ReentrantCallbackGroup()
        cb_timer = MutuallyExclusiveCallbackGroup()

        # --- inputs --------------------------------------------------------
        self.create_subscription(
            GraspTarget, '/tops/grasp_target', self._on_target, 10,
            callback_group=cb_sub)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, sensor_qos,
            callback_group=cb_sub)
        self.create_subscription(
            ControllerDiagnostics,
            f'/{self.controller_name}/diagnostics', self._on_diagnostics, 10,
            callback_group=cb_sub)

        # --- outputs -------------------------------------------------------
        self.state_pub = self.create_publisher(SupervisorState, '/tops/supervisor_state', 10)
        self.command_pub = self.create_publisher(
            GraspTarget, f'/{self.controller_name}/grasp_target', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        # --- controller services -------------------------------------------
        self.mode_client = self.create_client(
            SetControlMode, f'/{self.controller_name}/set_control_mode',
            callback_group=cb_sub)
        self.retreat_client = self.create_client(
            TriggerRetreat, f'/{self.controller_name}/trigger_retreat',
            callback_group=cb_sub)

        # --- trial control --------------------------------------------------
        self.create_service(Trigger, '~/start_trial', self._on_start, callback_group=cb_sub)
        self.create_service(Trigger, '~/abort', self._on_abort, callback_group=cb_sub)
        self.create_service(Trigger, '~/reset', self._on_reset, callback_group=cb_sub)

        self.timer = self.create_timer(
            1.0 / self.rate_hz, self._tick, callback_group=cb_timer)

        self.get_logger().info(
            f'supervisor up at {self.rate_hz:.0f} Hz, driving '
            f'/{self.controller_name}. Waiting for ~/start_trial.')

    # ------------------------------------------------------------------
    #  parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter('controller_name', 'cartesian_impedance_controller')
        self.declare_parameter('base_frame', 'panda_link0')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('arm_joints', [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'])
        self.declare_parameter('finger_joints',
                               ['panda_finger_joint1', 'panda_finger_joint2'])
        self.declare_parameter('gripper_open', 0.039)
        # Per-finger commanded position when closing, in metres (so the jaw
        # opening is twice this). NOT zero: see _observe() for why.
        self.declare_parameter('gripper_closed', 0.018)
        # How much wider than the command the jaw must end up for the object
        # to count as held.
        self.declare_parameter('gripper_grasp_margin', 0.004)
        # Finger speed below which the gripper counts as having stopped.
        self.declare_parameter('gripper_still_speed', 0.002)

        self.declare_parameter('compliant_trigger_force', 1.5)
        self.declare_parameter('compliant_release_force', 0.8)
        self.declare_parameter('compliant_trigger_distance', 0.015)
        self.declare_parameter('compliant_release_distance', 0.025)
        self.declare_parameter('hard_force_limit', 15.0)
        self.declare_parameter('retreat_distance', 0.050)
        self.declare_parameter('command_timeout', 0.100)
        self.declare_parameter('grasp_position_tolerance', 0.003)
        self.declare_parameter('min_confidence', 0.40)
        self.declare_parameter('lift_height', 0.10)
        # Speed at which the COMMANDED equilibrium is allowed to travel.
        # Not the arm's speed -- the arm follows this through its spring.
        self.declare_parameter('approach_speed', 0.20)   # m/s, free space
        self.declare_parameter('descent_speed', 0.05)    # m/s, final approach

    def _read_parameters(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.controller_name = g('controller_name')
        self.base_frame = g('base_frame')
        self.rate_hz = float(g('rate_hz'))
        self.arm_joints = list(g('arm_joints'))
        self.finger_joints = list(g('finger_joints'))
        self.gripper_open = float(g('gripper_open'))
        self.gripper_closed = float(g('gripper_closed'))
        self.gripper_grasp_margin = float(g('gripper_grasp_margin'))
        self.gripper_still_speed = float(g('gripper_still_speed'))

        self.thresholds = Thresholds(
            compliant_trigger_force=float(g('compliant_trigger_force')),
            compliant_release_force=float(g('compliant_release_force')),
            compliant_trigger_distance=float(g('compliant_trigger_distance')),
            compliant_release_distance=float(g('compliant_release_distance')),
            hard_force_limit=float(g('hard_force_limit')),
            retreat_distance=float(g('retreat_distance')),
            command_timeout=float(g('command_timeout')),
            grasp_position_tolerance=float(g('grasp_position_tolerance')),
            min_confidence=float(g('min_confidence')),
            lift_height=float(g('lift_height')),
        )
        self.approach_speed = float(g('approach_speed'))
        self.descent_speed = float(g('descent_speed'))

    # ------------------------------------------------------------------
    #  subscriptions
    # ------------------------------------------------------------------
    def _on_target(self, msg: GraspTarget):
        with self._lock:
            self._latest_target = msg
            self._target_stamp = self.get_clock().now()

    def _on_joint_state(self, msg: JointState):
        with self._lock:
            self._joint_state = msg

    def _on_diagnostics(self, msg: ControllerDiagnostics):
        with self._lock:
            self._diagnostics = msg
            self._contact_force = msg.external_force_norm

    # ------------------------------------------------------------------
    #  services
    # ------------------------------------------------------------------
    def _on_start(self, request, response):
        with self._lock:
            self.fsm.reset()
            self._start_requested = True
            self._abort_requested = False
            self._start_pose = None
            self._commanded = None
            self._goal = None
        response.success = True
        response.message = 'trial started'
        self.get_logger().info('--- trial start ---')
        return response

    def _on_abort(self, request, response):
        with self._lock:
            self._abort_requested = True
        response.success = True
        response.message = 'abort latched'
        self.get_logger().warning('abort requested')
        return response

    def _on_reset(self, request, response):
        with self._lock:
            self.fsm.reset()
            self._start_requested = False
            self._abort_requested = False
            self._start_pose = None
        response.success = True
        response.message = 'supervisor reset'
        return response

    # ------------------------------------------------------------------
    #  the tick
    # ------------------------------------------------------------------
    def _tick(self):
        with self._lock:
            obs = self._observe()
            target = self._latest_target

        action = self.fsm.step(obs)

        # Edge-triggered side effects. `entered_state` is true on exactly
        # the tick a transition happened, so each of these fires once.
        if action.entered_state:
            self.get_logger().info(
                f'{_STATE_NAMES.get(int(action.state), "?")}: '
                f'{action.transition_reason}')
            self._on_state_entry(action)

        self._drive_controller(action, target, obs)
        self._drive_gripper(action)
        self._publish_state(action, obs)
        self._trace(action, obs)

    def _trace(self, action, obs):
        """One line a second while the arm is moving.

        Without this, a trial that ends badly leaves only its final CSV row,
        and the row cannot distinguish "never converged" from "converged then
        drifted" from "never moved".
        """
        if action.state in (State.IDLE, State.ABORT, State.HOLD):
            return
        self._trace_accum = getattr(self, '_trace_accum', 0.0) + obs.dt
        if self._trace_accum < 1.0:
            return
        self._trace_accum = 0.0
        tcp = getattr(self, '_tcp_z', float('nan'))
        self.get_logger().info(
            f'  [{_STATE_NAMES.get(int(action.state), "?"):9s} '
            f't={self.fsm.time_in_state:5.2f}s] '
            f'err={obs.distance_to_target * 1000:7.1f}mm  '
            f'tcp_z={tcp:6.3f}  '
            f'F={obs.contact_force:5.2f}N  '
            f'lift={obs.tcp_height_above_start * 1000:+6.1f}mm  '
            f'jaw={self._last_width * 1000 if self._last_width is not None else -1:5.1f}mm '
            f'cmd={self._gripper_command * 2000:5.1f}mm '
            f'held={int(obs.object_grasped)}')

    def _observe(self) -> Observation:
        # Measure the tick interval; do not assume it.
        #
        # Everything here runs on SIMULATION time, and Gazebo does not
        # advance that clock smoothly -- it steps physics as fast as the host
        # allows and republishes /clock in bursts. A 50 Hz timer therefore
        # does NOT deliver 20 ms of simulated time per tick.
        #
        # Assuming 1/rate_hz made every duration in the state machine wrong by
        # whatever factor the host happened to be running at. The visible
        # symptom was the descent ramp: it is driven by time_in_state, so the
        # commanded target shot to the grasp pose almost immediately, the arm
        # could not follow, and the gripper closed with ~117 mm of error
        # still outstanding. That read as a controller failure and was a
        # clock-accounting failure.
        now = self.get_clock().now()
        if self._last_tick is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = (now - self._last_tick).nanoseconds * 1e-9
            # Guard both ends: a zero dt stalls every timeout, and a huge one
            # (a clock jump at start-up) would skip whole phases in one tick.
            dt = min(max(dt, 1e-4), 0.5)
        self._last_tick = now

        obs = Observation(dt=dt)
        obs.start_requested = self._start_requested
        obs.abort_requested = self._abort_requested
        obs.contact_force = self._contact_force

        now = self.get_clock().now()
        if self._target_stamp is not None:
            obs.seconds_since_target = (now - self._target_stamp).nanoseconds * 1e-9
        if self._latest_target is not None:
            obs.target_confidence = self._latest_target.confidence
            obs.target_is_final = self._latest_target.final

        if self._diagnostics is not None:
            d = self._diagnostics
            # Distance to the REQUESTED goal, not the controller's reported
            # tracking error.
            #
            # position_error_norm is the error against the equilibrium the
            # controller has ACCEPTED. When a target is refused as
            # implausible the equilibrium stops moving, the arm sits on it,
            # and that number falls to nearly zero -- it reads "arrived"
            # exactly when the arm is going nowhere. The supervisor then
            # walked the entire grasp sequence with the arm parked at home.
            #
            # Measuring TCP-to-goal directly cannot be fooled that way.
            goal = self._compute_goal()
            if goal is not None and self._tcp_xyz is not None:
                obs.distance_to_target = math.dist(self._tcp_xyz, goal)
            else:
                obs.distance_to_target = d.position_error_norm
            # The controller reports the stiffness actually in effect. The
            # ramp is settled when it has reached the compliant set, which
            # is the only definition that means "the gains you asked for
            # are the gains the arm has".
            obs.gains_settled = (
                abs(d.stiffness_scale - self._expected_stiffness_scale()) < 0.05)
            # Deliberately NOT a fault.
            #
            # Missed 1 kHz deadlines measure how well the host can run the
            # simulation, not whether the robot is behaving. On a laptop that
            # is also rendering Gazebo, thousands of them are normal and the
            # arm is still perfectly controlled -- the controller just runs
            # its loop at whatever rate it is given, and every gain it uses
            # is derived from the measured dt rather than an assumed one.
            #
            # Treating this as a fault aborted every trial before the arm
            # moved. The number still matters for the report, so it is
            # recorded in the trial CSV (max_update_period_ms,
            # missed_deadlines) and surfaced in the analysis under
            # "controller health" -- which is where a claim about 1 kHz
            # belongs, as evidence to qualify rather than a trigger to stop.
            obs.controller_faulted = False
            self._warn_if_loop_is_slow(d)

            tcp_z = d.pose_current.position.z
            if self._start_pose is None:
                self._start_pose = tcp_z
            self._tcp_z = tcp_z
            self._tcp_xyz = (d.pose_current.position.x,
                             d.pose_current.position.y,
                             tcp_z)

            # Height is measured from where the GRASP happened, not from
            # where the arm started.
            #
            # _start_pose is the TCP height at the first diagnostics message,
            # i.e. the home pose, roughly 0.48 m up. Lifting "0.10 m above
            # start" therefore commanded 0.58 m while the arm was down at
            # 0.03 m -- a 0.55 m jump, which the controller rightly refused
            # as an implausible target, so the arm never lifted at all.
            ref = self._grasp_height if self._grasp_height is not None else self._start_pose
            obs.tcp_height_above_start = tcp_z - ref

        if self._joint_state is not None:
            obs.at_home = self._at_home(self._joint_state)
            width = self._finger_width(self._joint_state)
            speed = self._finger_speed(self._joint_state)
            self._last_width = width

            if width is not None and speed is not None:
                # "Closed" means the fingers have STOPPED, not that the jaw
                # reached zero. Those are different things, and conflating
                # them was a real bug: a gripper holding a 50 mm cube can
                # never reach zero width, so the old test
                #
                #     gripper_closed = width <= small_number
                #
                # could only ever be satisfied by closing on thin air. Every
                # trial that actually caught the object therefore sat in
                # GRASP until it timed out, while every trial that MISSED it
                # closed fully and advanced happily to LIFT -- the condition
                # was, precisely, a detector for failed grasps.
                #
                # Stopping is the right signal, because it covers both
                # outcomes: the fingers either reach the commanded width
                # (nothing there) or stall against the object (something
                # there). Which of the two it was is then a question about
                # the final width, below.
                obs.gripper_closed = (
                    speed < self.gripper_still_speed
                    and self.fsm.time_in_state > 0.4)

                # Held if the jaw was blocked measurably short of its
                # command. Commanded 2 x 18 mm = 36 mm; a 50 mm cube stalls
                # the fingers at about 50 mm, which clears the margin easily,
                # while closing on nothing lands at 36 mm and does not.
                commanded = 2.0 * self.gripper_closed
                obs.object_grasped = width > commanded + self.gripper_grasp_margin

        return obs

    def _compute_goal(self):
        """Where the TCP is being asked to end up, right now."""
        state = self.fsm.state
        if state in (State.SERVO, State.COMPLIANT):
            t = self._latest_target
            if t is None:
                return None
            self._goal = (t.pose.position.x, t.pose.position.y, t.pose.position.z)
            self._goal_orientation = t.pose.orientation
        elif state in (State.LIFT, State.HOLD) and self._grasp_xy is not None:
            self._goal = (self._grasp_xy[0], self._grasp_xy[1],
                          self._grasp_height + self.thresholds.lift_height)
        return self._goal

    def _expected_stiffness_scale(self) -> float:
        """What stiffness_scale should read for the state we are in."""
        if self.fsm.state in (State.COMPLIANT, State.GRASP, State.LIFT):
            return 220.0 / 800.0
        return 1.0

    def _warn_if_loop_is_slow(self, diagnostics) -> None:
        """Say once, loudly, if the host cannot keep up with 1 kHz.

        This is a note about the MACHINE, not the robot. It is worth saying
        because a peak-force or timing number measured on a host running at a
        0.1 real-time factor deserves that caveat in the write-up, and the
        person reading the terminal is the one who can add it.
        """
        if self._slow_loop_warned:
            return
        if diagnostics.update_period_max > 0.005:      # 5x the 1 ms nominal
            self._slow_loop_warned = True
            self.get_logger().warning(
                'the control loop is not keeping up with 1 kHz on this host: '
                f'worst cycle {diagnostics.update_period_max * 1000:.1f} ms, '
                f'{diagnostics.missed_deadlines} missed deadlines. The trial '
                'still runs and the control law is unaffected, but quote '
                'timing results with this caveat. Closing the Gazebo window '
                '(gui:=false) typically doubles the real-time factor.')

    def _at_home(self, js: JointState) -> bool:
        try:
            q = [js.position[js.name.index(j)] for j in self.arm_joints]
        except (ValueError, IndexError):
            return False
        home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        return all(abs(a - b) < 0.05 for a, b in zip(q, home))

    def _finger_width(self, js: JointState):
        """Total jaw opening: the two finger positions summed."""
        try:
            return sum(js.position[js.name.index(j)] for j in self.finger_joints)
        except (ValueError, IndexError):
            return None

    def _finger_speed(self, js: JointState):
        """How fast the jaw is moving, regardless of direction."""
        try:
            return sum(abs(js.velocity[js.name.index(j)]) for j in self.finger_joints)
        except (ValueError, IndexError):
            return None

    # ------------------------------------------------------------------
    #  outputs
    # ------------------------------------------------------------------
    def _on_state_entry(self, action):
        if action.state is State.GRASP:
            # Freeze the reference the lift is measured and commanded from.
            self._grasp_height = getattr(self, '_tcp_z', None)
            self._grasp_xy = (self._tcp_xyz[0], self._tcp_xyz[1]) \
                if self._tcp_xyz else None
        if action.state in (State.IDLE, State.HOMING):
            self._grasp_height = None
            self._grasp_xy = None

        if action.request_compliant_gains:
            # Approach regulates force; everything after the gripper closes
            # keeps the soft gains but stops pressing.
            self._request_mode(
                SetControlMode.Request.MODE_COMPLIANT
                if action.state is State.COMPLIANT
                else SetControlMode.Request.MODE_GRASPED)
        elif action.request_stiff_gains:
            self._request_mode(SetControlMode.Request.MODE_STIFF)

        if action.state is State.RETREAT:
            self._request_retreat()

    def _request_mode(self, mode: int):
        if not self.mode_client.service_is_ready():
            self.get_logger().warning(
                f'/{self.controller_name}/set_control_mode is not available; '
                'the arm will keep whatever gains it has')
            return
        request = SetControlMode.Request()
        request.mode = mode
        # Fire and forget. Blocking the 50 Hz tick on a service round trip
        # would stall the interlocks, which is a considerably worse outcome
        # than not knowing whether the gain change was accepted -- and the
        # controller reports the gains it is actually running, so the
        # supervisor finds out either way.
        self.mode_client.call_async(request)

    def _request_retreat(self):
        if not self.retreat_client.service_is_ready():
            self.get_logger().error(
                'retreat service unavailable; the controller will stay '
                'latched at zero stiffness until it is deactivated')
            return
        request = TriggerRetreat.Request()
        request.distance = self.thresholds.retreat_distance
        request.then_hold = True
        self.retreat_client.call_async(request)

    def _drive_controller(self, action, target, obs):
        """Send the controller a target, or hold."""
        if action.state in (State.IDLE, State.ABORT):
            return

        msg = GraspTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        goal = self._goal
        if goal is not None and self._tcp_xyz is not None:
            # Slew the commanded equilibrium toward the goal at a bounded
            # speed instead of stepping it there.
            #
            # Two things go wrong with a step. The controller refuses it as
            # implausible once it is further than safety.max_command_step_m,
            # which is what silently broke every grasp. And even when
            # accepted, a 0.39 m displacement against the stiff-mode spring
            # commands 0.39 x 800 = 312 N, which the torque clamp then has to
            # absorb -- a lunge, not a servo.
            #
            # Slewing keeps the spring displacement equal to the genuine
            # tracking error at all times, so the commanded force stays in
            # the range the gains were designed for.
            if self._commanded is None:
                self._commanded = list(self._tcp_xyz)
            speed = (self.descent_speed
                     if action.state in (State.COMPLIANT, State.GRASP)
                     else self.approach_speed)
            step = max(1e-4, speed * max(obs.dt, 1e-4))
            dx = [g - c for g, c in zip(goal, self._commanded)]
            dist = math.sqrt(sum(v * v for v in dx))
            if dist > step:
                self._commanded = [c + v * step / dist
                                   for c, v in zip(self._commanded, dx)]
            else:
                self._commanded = list(goal)

            msg.pose.position.x = self._commanded[0]
            msg.pose.position.y = self._commanded[1]
            msg.pose.position.z = self._commanded[2]
            if getattr(self, '_goal_orientation', None) is not None:
                msg.pose.orientation = self._goal_orientation
            msg.confidence = 1.0
        elif self._diagnostics is not None:
            # Not tracking: command the pose the controller is already
            # holding, plus any lift offset. Republishing rather than going
            # silent keeps the controller's watchdog fed, which matters --
            # a silent supervisor and a dead supervisor look identical from
            # the controller's side, and only one of them should zero the
            # feed-forward.
            msg.pose = self._diagnostics.pose_desired
            msg.confidence = 1.0
        else:
            return

        self.command_pub.publish(msg)

    def _drive_gripper(self, action):
        if action.close_gripper:
            width = self.gripper_closed
        elif action.open_gripper:
            width = self.gripper_open
        else:
            width = self._gripper_command
        self._gripper_command = width

        msg = Float64MultiArray()
        msg.data = [width, width]
        self.gripper_pub.publish(msg)

    def _publish_state(self, action, obs):
        msg = SupervisorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.state = int(action.state)
        msg.state_name = _STATE_NAMES.get(int(action.state), 'UNKNOWN')
        msg.time_in_state = self.fsm.time_in_state
        msg.transition_reason = action.transition_reason
        msg.distance_to_target = (
            obs.distance_to_target if math.isfinite(obs.distance_to_target) else -1.0)
        msg.contact_force = obs.contact_force
        msg.contact_detected = obs.contact_force > self.thresholds.compliant_trigger_force
        msg.within_switch_radius = (
            obs.distance_to_target < self.thresholds.compliant_trigger_distance)
        msg.wrench_cutoff_tripped = 'wrench_cutoff' in action.faults
        msg.watchdog_tripped = 'watchdog' in action.faults
        msg.joint_limit_warning = False
        msg.active_faults = list(action.faults)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
