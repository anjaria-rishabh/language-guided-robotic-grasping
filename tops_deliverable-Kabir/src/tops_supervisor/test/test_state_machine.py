# Copyright 2026 Kabir Dhingra. MIT licence.
"""
Tests for the grasp FSM.

The interesting cases are all failure cases. A happy-path test proves the
sequence runs; the tests below prove it stops when it should, which is the
property that actually matters.
"""
import pytest

from tops_supervisor.state_machine import (
    GraspStateMachine,
    Observation,
    State,
    Thresholds,
)

DT = 0.02  # 50 Hz supervisor tick


def run(fsm, obs, seconds):
    """Tick the machine for `seconds` with a constant observation."""
    action = None
    for _ in range(int(seconds / DT)):
        action = fsm.step(obs)
    return action


def good_target(**kwargs):
    """An observation representing a healthy, confident target feed."""
    base = dict(
        dt=DT,
        target_confidence=0.95,
        seconds_since_target=0.0,
        distance_to_target=0.20,
        contact_force=0.0,
    )
    base.update(kwargs)
    return Observation(**base)


# ---------------------------------------------------------------------------
#  The happy path
# ---------------------------------------------------------------------------

def test_full_successful_sequence():
    fsm = GraspStateMachine()
    assert fsm.state is State.IDLE

    fsm.step(good_target(start_requested=True))
    assert fsm.state is State.HOMING

    fsm.step(good_target(at_home=True))
    assert fsm.state is State.SERVO

    # Still far away: stays in SERVO at stiff gains.
    action = fsm.step(good_target(distance_to_target=0.15))
    assert fsm.state is State.SERVO
    assert action.request_stiff_gains
    assert not action.request_compliant_gains

    # Inside the switch radius.
    action = fsm.step(good_target(distance_to_target=0.010))
    assert fsm.state is State.COMPLIANT
    assert action.request_compliant_gains
    assert not action.request_stiff_gains

    # Gains still ramping -- must not proceed.
    fsm.step(good_target(distance_to_target=0.002, gains_settled=False))
    assert fsm.state is State.COMPLIANT

    run(fsm, good_target(distance_to_target=0.002, gains_settled=True), 0.1)
    assert fsm.state is State.GRASP

    fsm.step(good_target(distance_to_target=0.002, gains_settled=True,
                         gripper_closed=True))
    assert fsm.state is State.LIFT

    action = fsm.step(good_target(gains_settled=True, gripper_closed=True,
                                  tcp_height_above_start=0.06,
                                  object_grasped=True))
    assert fsm.state is State.HOLD
    assert action.close_gripper


# ---------------------------------------------------------------------------
#  Mode switching
# ---------------------------------------------------------------------------

def test_contact_triggers_compliance_even_when_far_from_the_vision_target():
    """The case the whole project exists for.

    The camera says the object is 80 mm away. The arm touches it anyway,
    because the camera was wrong. Distance alone would have kept the arm at
    800 N/m through the impact.
    """
    fsm = GraspStateMachine()
    fsm.step(good_target(start_requested=True))
    fsm.step(good_target(at_home=True))
    assert fsm.state is State.SERVO

    action = fsm.step(good_target(distance_to_target=0.080, contact_force=2.0))
    assert fsm.state is State.COMPLIANT
    assert action.request_compliant_gains
    assert 'contact detected' in fsm.transition_reason


def test_compliant_gains_are_requested_in_every_contact_state():
    for state in (State.COMPLIANT, State.GRASP, State.LIFT):
        fsm = GraspStateMachine()
        fsm._state = state           # noqa: SLF001 - exercising one state directly
        action = fsm.step(good_target(gains_settled=True))
        assert action.request_compliant_gains, state


def test_target_is_only_tracked_during_the_two_approach_states():
    """A late perception update must not be able to move the arm mid-grasp."""
    tracked = set()
    for state in State:
        fsm = GraspStateMachine()
        fsm._state = state           # noqa: SLF001
        action = fsm.step(good_target(gains_settled=True))
        if action.track_target:
            tracked.add(fsm.state)
    assert tracked == {State.SERVO, State.COMPLIANT}


# ---------------------------------------------------------------------------
#  Interlocks
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('from_state', [
    State.HOMING, State.SERVO, State.COMPLIANT, State.GRASP, State.LIFT,
])
def test_hard_force_limit_retreats_from_any_active_state(from_state):
    """There must be no state from which the arm cannot be stopped."""
    fsm = GraspStateMachine()
    fsm._state = from_state          # noqa: SLF001
    action = fsm.step(good_target(contact_force=18.0))
    assert fsm.state is State.RETREAT
    assert action.trigger_retreat
    assert 'wrench_cutoff' in action.faults


def test_force_limit_is_checked_before_any_state_logic():
    """Even in a state that would otherwise transition, the fault wins."""
    fsm = GraspStateMachine()
    fsm._state = State.COMPLIANT     # noqa: SLF001
    fsm.step(good_target(distance_to_target=0.001, gains_settled=True,
                         contact_force=20.0))
    assert fsm.state is State.RETREAT


def test_watchdog_fires_when_the_perception_feed_stops():
    fsm = GraspStateMachine()
    fsm._state = State.SERVO         # noqa: SLF001
    action = fsm.step(good_target(seconds_since_target=0.15))
    assert fsm.state is State.RETREAT
    assert 'watchdog' in action.faults


def test_watchdog_does_not_fire_during_grasp_or_lift():
    """Perception has nothing left to say once the gripper is closing.

    Demanding a heartbeat there would manufacture a fault out of entirely
    normal operation.
    """
    for state in (State.GRASP, State.LIFT):
        fsm = GraspStateMachine()
        fsm._state = state           # noqa: SLF001
        fsm.step(good_target(seconds_since_target=5.0, gains_settled=True))
        assert fsm.state is not State.RETREAT, state


def test_abort_request_beats_everything():
    fsm = GraspStateMachine()
    fsm._state = State.LIFT          # noqa: SLF001
    fsm.step(good_target(abort_requested=True, contact_force=100.0))
    assert fsm.state is State.ABORT


def test_low_confidence_target_does_not_move_the_arm():
    """A confident wrong answer is worse than no answer."""
    fsm = GraspStateMachine()
    fsm._state = State.SERVO         # noqa: SLF001
    action = fsm.step(good_target(target_confidence=0.1, distance_to_target=0.005))
    assert fsm.state is State.SERVO
    assert 'low_confidence' in action.faults


def test_low_confidence_eventually_aborts_rather_than_hanging():
    fsm = GraspStateMachine(Thresholds(servo_timeout=1.0))
    fsm._state = State.SERVO         # noqa: SLF001
    run(fsm, good_target(target_confidence=0.1), 1.5)
    assert fsm.state is State.ABORT


# ---------------------------------------------------------------------------
#  Timeouts
# ---------------------------------------------------------------------------

def test_every_active_state_has_a_timeout():
    """A state that hangs is worse than a state that fails.

    This test is the guard against someone adding a state later and
    forgetting to give it a bound.
    """
    cases = [
        (State.HOMING, Thresholds(homing_timeout=0.5), {}),
        (State.SERVO, Thresholds(servo_timeout=0.5), {}),
        (State.COMPLIANT, Thresholds(compliant_timeout=0.5),
         {'gains_settled': True, 'distance_to_target': 0.5}),
        (State.GRASP, Thresholds(grasp_timeout=0.5), {}),
        (State.LIFT, Thresholds(lift_timeout=0.5), {}),
    ]
    for state, thresholds, extra in cases:
        fsm = GraspStateMachine(thresholds)
        fsm._state = state           # noqa: SLF001
        run(fsm, good_target(**extra), 1.0)
        assert fsm.state in (State.RETREAT, State.ABORT), \
            f'{state.name} did not time out'


def test_retreat_terminates_and_does_not_silently_retry():
    """A trial that needed a retreat failed.

    Quietly re-attempting would turn one failure into a success and corrupt
    the statistics.
    """
    fsm = GraspStateMachine()
    fsm._state = State.RETREAT       # noqa: SLF001
    run(fsm, good_target(), 2.0)
    assert fsm.state is State.ABORT


def test_lifting_without_the_object_is_recorded_as_a_failure():
    fsm = GraspStateMachine()
    fsm._state = State.LIFT          # noqa: SLF001
    action = fsm.step(good_target(tcp_height_above_start=0.08, object_grasped=False))
    assert fsm.state is State.RETREAT
    assert 'object_dropped' in action.faults


# ---------------------------------------------------------------------------
#  Bookkeeping
# ---------------------------------------------------------------------------

def test_transition_reason_is_always_populated():
    """Every transition must explain itself, for the trial log."""
    fsm = GraspStateMachine()
    fsm.step(good_target(start_requested=True))
    assert fsm.transition_reason
    fsm.step(good_target(at_home=True))
    assert 'home' in fsm.transition_reason.lower()


def test_entered_state_is_true_exactly_once_per_transition():
    fsm = GraspStateMachine()
    a = fsm.step(good_target(start_requested=True))
    assert a.entered_state
    b = fsm.step(good_target())
    assert not b.entered_state


def test_reset_returns_to_idle():
    fsm = GraspStateMachine()
    fsm._state = State.HOLD          # noqa: SLF001
    fsm.reset()
    assert fsm.state is State.IDLE
    assert fsm.time_in_state == 0.0


def test_state_values_match_the_message_constants():
    """The FSM and tops_msgs/SupervisorState.msg must not drift apart."""
    assert (State.IDLE, State.HOMING, State.SERVO, State.COMPLIANT,
            State.GRASP, State.LIFT, State.HOLD, State.RETREAT,
            State.ABORT) == (0, 1, 2, 3, 4, 5, 6, 7, 8)
