function [F, faults, entered] = decideGraspPhase(F, obs, U)
%DECIDEGRASPPHASE  Which phase of the grasp comes next?
%
%   [F, faults, entered] = grasp.supervisor.decideGraspPhase(F, obs, U)
%
%   Pure: no clock, no I/O. It only responds to the observation it is
%   handed, which is what makes every transition checkable in a test.
%
%   F        .state, .timeInState, .justEntered
%   obs      observation struct, built in grasp.supervisor.superviseGrasp
%   U        thresholds (grasp.config.controllerSettings)
%   faults   bitmask raised THIS tick (grasp.supervisor.faultNames)
%   entered  true on exactly the tick a transition happened
%
%   Phases (grasp.supervisor.phaseNames):
%     0 WAIT     1 GO_HOME    2 MOVE_ABOVE_BOX  3 DESCEND  4 CLOSE_FINGERS
%     5 LIFT     6 HOLD       7 RETREAT         8 STOP
%
%   RETREAT and STOP can be reached from any phase, so there is no phase in
%   which the arm cannot be stopped.

WAIT = 0; GO_HOME = 1; MOVE_ABOVE_BOX = 2; DESCEND = 3; CLOSE_FINGERS = 4;
LIFT = 5; HOLD = 6; RETREAT = 7; STOP = 8;
% fault bits
SAFETY_STOP = 1; NO_TARGET = 2; HOME_TO = 4; MOVE_TO = 8; DESCEND_TO = 16;
CLOSE_TO = 32; LIFT_TO = 64; DROPPED = 128; LOW_CONF = 256;

F.timeInState = F.timeInState + obs.dt;
faults = 0;
s = F.state;

% ---- stops, checked first, from every phase --------------------------------
if obs.safety_stop
    faults = bitor(faults, SAFETY_STOP);
    if s ~= RETREAT && s ~= STOP && s ~= WAIT
        [F, entered] = go(F, RETREAT);
        return
    end
end
% Only while chasing a target: during the grip and lift there is nothing
% new for the target source to say.
if (s == MOVE_ABOVE_BOX || s == DESCEND) && obs.seconds_since_target > U.target_timeout
    faults = bitor(faults, NO_TARGET);
    [F, entered] = go(F, RETREAT);
    return
end

% ---- per-phase logic --------------------------------------------------------
switch s
    case WAIT
        if obs.start_requested
            F = moveTo(F, GO_HOME);
        end

    case GO_HOME
        if obs.at_home
            F = moveTo(F, MOVE_ABOVE_BOX);
        elseif F.timeInState > U.home_timeout
            faults = bitor(faults, HOME_TO);
            F = moveTo(F, STOP);
        end

    case MOVE_ABOVE_BOX
        if obs.target_confidence < U.min_confidence
            % A low-confidence target is a reason to wait, not to move.
            faults = bitor(faults, LOW_CONF);
            if F.timeInState > U.move_timeout
                F = moveTo(F, STOP);
            end
        else
            % Arrived, or touching something close to the box.
            near = obs.distance_to_target < U.descend_trigger_distance;
            touching = obs.force_at_wrist > U.descend_trigger_force && ...
                       obs.distance_to_target < U.touch_radius;
            if near || touching
                F = moveTo(F, DESCEND);
            elseif F.timeInState > U.move_timeout
                faults = bitor(faults, MOVE_TO);
                F = moveTo(F, RETREAT);
            end
        end

    case DESCEND
        % Close once within tolerance, or once the descent has had long
        % enough to converge. Never while stiffness is still changing.
        closeEnough = obs.distance_to_target <= U.close_tolerance;
        converged = F.timeInState >= U.settle_before_close;
        ready = F.timeInState >= U.gain_settle_time && obs.gains_settled;
        if ready && (closeEnough || converged)
            F = moveTo(F, CLOSE_FINGERS);
        elseif F.timeInState > U.descend_timeout
            faults = bitor(faults, DESCEND_TO);
            F = moveTo(F, RETREAT);
        end

    case CLOSE_FINGERS
        if obs.gripper_closed
            F = moveTo(F, LIFT);
        elseif F.timeInState > U.close_timeout
            faults = bitor(faults, CLOSE_TO);
            F = moveTo(F, RETREAT);
        end

    case LIFT
        if obs.tcp_height_above_start >= U.lift_success_height
            if obs.object_grasped
                F = moveTo(F, HOLD);
            else
                faults = bitor(faults, DROPPED);
                F = moveTo(F, RETREAT);
            end
        elseif F.timeInState > U.lift_timeout
            faults = bitor(faults, LIFT_TO);
            F = moveTo(F, RETREAT);
        end

    case RETREAT
        % Never a quiet retry: a run that needed a retreat has failed.
        if F.timeInState > U.retreat_time
            F = moveTo(F, STOP);
        end
end

entered = F.justEntered;
F.justEntered = false;
end

% =========================================================================
function F = moveTo(F, newState)
if newState ~= F.state
    F.state = newState;
    F.timeInState = 0;
    F.justEntered = true;
end
end

function [F, entered] = go(F, newState)
F = moveTo(F, newState);
entered = F.justEntered;
F.justEntered = false;
end
