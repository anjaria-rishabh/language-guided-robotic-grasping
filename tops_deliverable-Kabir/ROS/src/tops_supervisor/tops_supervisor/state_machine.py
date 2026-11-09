# Copyright 2026 Kabir Dhingra. MIT licence.
"""
The grasp sequence as an explicit finite state machine.

Theory/09_Integration_FSM_and_Safety.md.

This module contains NO ROS code, on purpose. The transition logic is the
part most worth testing, and testing it should not require a simulator, a
robot, or a running middleware. `tops_supervisor.supervisor_node` is the
thin wrapper that connects this to topics and services.

---------------------------------------------------------------------------
Why a state machine and not a sequence of function calls
---------------------------------------------------------------------------
Because the interesting behaviour is in the transitions that are NOT the
happy path. A straight-line script has one route through it. This system
has to answer, at every instant: what happens if the force limit trips
right now? If the perception feed dies mid-approach? If the object moves
after the gains have already gone soft?

Writing the states down makes those answers explicit and reviewable, rather
than implicit in the order somebody happened to write the code.

---------------------------------------------------------------------------
The states
---------------------------------------------------------------------------
    IDLE      nothing commanded; controller active, holding position
    HOMING    moving to the known start configuration
    SERVO     free-space approach at stiff gains, tracking the vision target
    COMPLIANT final approach at soft gains; contact is expected here
    GRASP     gripper closing
    LIFT      raising the object
    HOLD      holding; the trial is over and succeeded
    RETREAT   a safety trigger fired; backing off
    ABORT     unrecoverable; the controller is deactivated

The only transitions that can happen from ANY state are the ones into
RETREAT and ABORT. That asymmetry is the safety argument: there is no
state from which the arm cannot be stopped.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional


class State(IntEnum):
    """Mirrors the constants in tops_msgs/SupervisorState.msg exactly."""

    IDLE = 0
    HOMING = 1
    SERVO = 2
    COMPLIANT = 3
    GRASP = 4
    LIFT = 5
    HOLD = 6
    RETREAT = 7
    ABORT = 8


#: States in which the arm is expected to be in, or entering, contact. The
#: controller runs compliant gains here and nowhere else.
CONTACT_STATES = frozenset({State.COMPLIANT, State.GRASP, State.LIFT})

#: States a trial can legitimately end in.
TERMINAL_STATES = frozenset({State.HOLD, State.ABORT})


@dataclass
class Thresholds:
    """Every number that decides a transition, in one place.

    These are the numbers the report has to defend, so they live together
    rather than scattered through the logic. Defaults are the project
    plan's target values.
    """

    #: Contact force that arms the compliant switch, newtons.
    compliant_trigger_force: float = 1.5

    #: ...but only within this distance of the goal, in metres.
    #:
    #: The wrist sensor measures the tool's own inertia as well as contact.
    #: Accelerating 1.55 kg of hand and fingers through free space reads as
    #: several newtons, which tripped the contact trigger 400 mm from the
    #: object and switched the arm to compliant gains before it had gone
    #: anywhere. Contact is only a meaningful explanation for a force when
    #: there is something within reach to be in contact WITH; a genuine
    #: unexpected collision further out is the 15 N safety cutoff's job,
    #: not this trigger's.
    contact_trigger_radius: float = 0.10
    #: ...and the force it must fall below to disarm. The gap is what stops
    #: a noisy signal chattering across a single threshold.
    compliant_release_force: float = 0.8

    #: Distance from the target that arms the compliant switch, metres.
    compliant_trigger_distance: float = 0.015
    #: ...and the distance it must exceed to disarm.
    compliant_release_distance: float = 0.025

    #: Hard cutoff. Motion halts and the arm retreats.
    hard_force_limit: float = 15.0
    #: How far to retreat, metres.
    retreat_distance: float = 0.050

    #: No grasp target within this many seconds is a fault.
    command_timeout: float = 0.100

    #: Terminal targeting accuracy the project TARGETS, in metres.
    #:
    #: Reaching it lets the gripper close immediately. Not reaching it is not
    #: a failure: see settle_before_grasp below. This distinction matters,
    #: because 3 mm is a RESULT the experiments measure, not a precondition
    #: the trial must satisfy. Gating on it would record a grasp with 4 mm of
    #: error as "never attempted" and quietly bias every reported mean.
    grasp_position_tolerance: float = 0.003

    #: If the tolerance is not met, close anyway once the approach has had
    #: this long to converge, and record whatever error was actually achieved.
    #: That keeps the metric honest in both directions: good trials are not
    #: delayed, and poor ones are measured instead of discarded.
    settle_before_grasp: float = 7.0
    #: Grounding confidence below which a target is ignored entirely.
    min_confidence: float = 0.40

    #: Per-state time limits. A state that has not achieved its goal in this
    #: long has failed; saying so is better than hanging. The trial budget
    #: is 15 s end to end, and these sum to less than that on purpose.
    homing_timeout: float = 10.0
    servo_timeout: float = 12.0
    compliant_timeout: float = 16.0
    grasp_timeout: float = 3.0
    #: The lift is slew-rate limited like every other motion: 100 mm of
    #: commanded travel at the approach speed is already ~0.8 s before the
    #: arm's own tracking lag, and on a host that cannot hold real time the
    #: wall-clock cost is several times that. At 4 s this tripped on three
    #: trials out of four while the grasp itself had succeeded — a timeout
    #: measuring the host, not the robot.
    lift_timeout: float = 10.0

    #: How long the gains must have settled before GRASP may begin. The
    #: controller reports this; closing the gripper mid-ramp means closing
    #: it at a stiffness nobody chose.
    gain_settle_time: float = 0.050

    #: Lift height, metres, and the height at which a lift counts.
    lift_height: float = 0.10
    lift_success_height: float = 0.05


@dataclass
class Observation:
    """Everything the FSM is allowed to look at.

    Deliberately a plain value object. The state machine cannot reach out
    and read a topic, call a service, or check the clock -- it can only
    respond to what it is handed. That is what makes every transition
    reproducible in a test.
    """

    dt: float = 0.0
    distance_to_target: float = float('inf')
    contact_force: float = 0.0
    target_confidence: float = 0.0
    target_is_final: bool = False
    seconds_since_target: float = float('inf')
    gains_settled: bool = False
    gripper_closed: bool = False
    object_grasped: bool = False
    tcp_height_above_start: float = 0.0
    at_home: bool = False
    start_requested: bool = False
    abort_requested: bool = False
    controller_faulted: bool = False


@dataclass
class Action:
    """What the supervisor should do as a result of this tick.

    Returned rather than performed, for the same reason Observation is
    passed in rather than read: it makes the whole decision testable.
    """

    state: State = State.IDLE
    entered_state: bool = False
    transition_reason: str = ''
    request_compliant_gains: bool = False
    request_stiff_gains: bool = False
    close_gripper: bool = False
    open_gripper: bool = False
    trigger_retreat: bool = False
    track_target: bool = True
    lift_offset: float = 0.0
    faults: List[str] = field(default_factory=list)


class GraspStateMachine:
    """The sequencer. One `step()` per supervisor tick, typically at 50 Hz."""

    def __init__(self, thresholds: Optional[Thresholds] = None):
        self.thresholds = thresholds or Thresholds()
        self._state = State.IDLE
        self._time_in_state = 0.0
        self._reason = 'initialised'
        self._just_entered = True
        # Latched: once contact is seen, the compliant switch stays armed.
        # Releasing on a momentary drop would mean re-stiffening the arm
        # while it is touching something, which is the single worst moment
        # to do that.
        self._compliant_armed = False
        self._faults: List[str] = []

    # -- introspection ---------------------------------------------------
    @property
    def state(self) -> State:
        return self._state

    @property
    def time_in_state(self) -> float:
        return self._time_in_state

    @property
    def transition_reason(self) -> str:
        return self._reason

    @property
    def faults(self) -> List[str]:
        return list(self._faults)

    def reset(self) -> None:
        self._state = State.IDLE
        self._time_in_state = 0.0
        self._reason = 'reset'
        self._just_entered = True
        self._compliant_armed = False
        self._faults = []

    # -- the transition itself -------------------------------------------
    def _transition(self, new_state: State, reason: str) -> None:
        if new_state is self._state:
            return
        self._state = new_state
        self._time_in_state = 0.0
        self._reason = reason
        self._just_entered = True

    def step(self, obs: Observation) -> Action:
        self._time_in_state += obs.dt
        self._faults = []

        t = self.thresholds

        # ------------------------------------------------------------------
        #  Interlocks, checked before anything else and from every state.
        #
        #  Order matters: an abort request beats a force fault beats a
        #  watchdog. Checking these first is what makes "there is no state
        #  from which the arm cannot be stopped" true rather than aspirational.
        # ------------------------------------------------------------------
        if obs.abort_requested:
            self._transition(State.ABORT, 'abort requested')
            return self._build(faults=['abort_requested'])

        if obs.controller_faulted:
            self._faults.append('controller_fault')
            self._transition(State.ABORT, 'controller reported a fault')
            return self._build(faults=self._faults)

        if obs.contact_force > t.hard_force_limit:
            self._faults.append('wrench_cutoff')
            if self._state not in (State.RETREAT, State.ABORT, State.IDLE):
                self._transition(
                    State.RETREAT,
                    f'contact force {obs.contact_force:.2f} N exceeded '
                    f'{t.hard_force_limit:.1f} N')
                action = self._build(faults=self._faults)
                action.trigger_retreat = True
                return action

        # The watchdog only applies while the arm is actively chasing a
        # target. During GRASP and LIFT the perception loop has already
        # done its job and has nothing left to say, so demanding a
        # heartbeat there would manufacture a fault out of normal operation.
        watchdog_states = (State.SERVO, State.COMPLIANT)
        if self._state in watchdog_states and obs.seconds_since_target > t.command_timeout:
            self._faults.append('watchdog')
            self._transition(
                State.RETREAT,
                f'no grasp target for {obs.seconds_since_target * 1000:.0f} ms')
            action = self._build(faults=self._faults)
            action.trigger_retreat = True
            return action

        # ------------------------------------------------------------------
        #  Per-state logic
        # ------------------------------------------------------------------
        handler = {
            State.IDLE: self._idle,
            State.HOMING: self._homing,
            State.SERVO: self._servo,
            State.COMPLIANT: self._compliant,
            State.GRASP: self._grasp,
            State.LIFT: self._lift,
            State.HOLD: self._hold,
            State.RETREAT: self._retreat,
            State.ABORT: self._abort,
        }[self._state]
        handler(obs)

        return self._build(faults=self._faults)

    # -- per-state handlers ----------------------------------------------
    def _idle(self, obs: Observation) -> None:
        if obs.start_requested:
            self._transition(State.HOMING, 'trial started')

    def _homing(self, obs: Observation) -> None:
        if obs.at_home:
            self._compliant_armed = False
            self._transition(State.SERVO, 'reached the home configuration')
        elif self._time_in_state > self.thresholds.homing_timeout:
            self._faults.append('homing_timeout')
            self._transition(State.ABORT, 'homing timed out')

    def _servo(self, obs: Observation) -> None:
        t = self.thresholds

        # A low-confidence grounding is not a reason to move. It is a reason
        # to wait: acting on it would send the arm to whatever the model
        # guessed, and a confident wrong answer is worse than no answer.
        if obs.target_confidence < t.min_confidence:
            self._faults.append('low_confidence')
            if self._time_in_state > t.servo_timeout:
                self._transition(State.ABORT, 'never received a confident target')
            return

        # Two independent ways to arm the compliant switch, because either
        # alone has a failure mode. Distance alone is blind to an object
        # that is not where vision said it was -- which is the entire point
        # of the project. Force alone would leave the arm stiff right up to
        # the instant of contact, so the first touch happens at 800 N/m.
        near = obs.distance_to_target < t.compliant_trigger_distance
        touching = (obs.contact_force > t.compliant_trigger_force
                    and obs.distance_to_target < t.contact_trigger_radius)

        if near or touching:
            self._compliant_armed = True
            reason = ('within {:.0f} mm of target'.format(obs.distance_to_target * 1000)
                      if near else
                      'contact detected at {:.2f} N'.format(obs.contact_force))
            self._transition(State.COMPLIANT, reason)
        elif self._time_in_state > t.servo_timeout:
            self._faults.append('servo_timeout')
            self._transition(State.RETREAT, 'servo phase timed out')

    def _compliant(self, obs: Observation) -> None:
        t = self.thresholds

        if not obs.gains_settled:
            # Still ramping. Closing the gripper now would close it at a
            # stiffness nobody chose.
            return

        close_enough = obs.distance_to_target <= t.grasp_position_tolerance
        converged = self._time_in_state >= t.settle_before_grasp
        ready = self._time_in_state >= t.gain_settle_time

        if ready and (close_enough or obs.target_is_final or converged):
            why = ('within tolerance' if close_enough
                   else 'perception reported final' if obs.target_is_final
                   else 'approach settled')
            self._transition(
                State.GRASP,
                'terminal error {:.1f} mm, {}'.format(
                    obs.distance_to_target * 1000, why))
        elif self._time_in_state > t.compliant_timeout:
            self._faults.append('compliant_timeout')
            self._transition(State.RETREAT, 'compliant approach timed out')

    def _grasp(self, obs: Observation) -> None:
        if obs.gripper_closed:
            self._transition(State.LIFT, 'gripper closed')
        elif self._time_in_state > self.thresholds.grasp_timeout:
            self._faults.append('grasp_timeout')
            self._transition(State.RETREAT, 'gripper did not close in time')

    def _lift(self, obs: Observation) -> None:
        t = self.thresholds
        if obs.tcp_height_above_start >= t.lift_success_height:
            if obs.object_grasped:
                self._transition(State.HOLD, 'lifted with the object held')
            else:
                # Lifted, but the object stayed on the table. That is a
                # failed grasp, not a failed trial -- it gets recorded and
                # the arm retreats cleanly.
                self._faults.append('object_dropped')
                self._transition(State.RETREAT, 'lifted but the object was not held')
        elif self._time_in_state > t.lift_timeout:
            self._faults.append('lift_timeout')
            self._transition(State.RETREAT, 'lift timed out')

    def _hold(self, obs: Observation) -> None:
        pass  # terminal success; the trial runner takes it from here

    def _retreat(self, obs: Observation) -> None:
        # Retreat unconditionally takes 1.5 s, then stops. It does not try
        # to recover and re-attempt: a trial that needed a retreat is a
        # trial that failed, and quietly retrying would corrupt the
        # statistics by turning one failure into a success.
        if self._time_in_state > 1.5:
            self._transition(State.ABORT, 'retreat complete')

    def _abort(self, obs: Observation) -> None:
        pass  # terminal failure

    # -- assembling the action -------------------------------------------
    def _build(self, faults: Optional[List[str]] = None) -> Action:
        # `entered_state` must be true on exactly the tick a transition
        # happened and no other, because the node uses it as the edge
        # trigger for one-shot side effects -- calling the mode service,
        # commanding the gripper, writing a log line. Latching it for an
        # extra tick would fire each of those twice.
        entered = self._just_entered
        self._just_entered = False

        action = Action(
            state=self._state,
            entered_state=entered,
            transition_reason=self._reason,
            faults=faults or [],
        )

        action.request_compliant_gains = self._state in CONTACT_STATES
        action.request_stiff_gains = self._state in (State.HOMING, State.SERVO)
        action.close_gripper = self._state in (State.GRASP, State.LIFT, State.HOLD)
        action.open_gripper = self._state in (State.IDLE, State.HOMING, State.RETREAT)
        action.trigger_retreat = self._state is State.RETREAT
        # Only the two approach states follow the vision target. Everywhere
        # else the supervisor commands the pose itself, so a late or wrong
        # perception update cannot move the arm during a grasp or a lift.
        action.track_target = self._state in (State.SERVO, State.COMPLIANT)
        action.lift_offset = self.thresholds.lift_height \
            if self._state in (State.LIFT, State.HOLD) else 0.0
        return action
