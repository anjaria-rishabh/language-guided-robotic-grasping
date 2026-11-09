function [cmd, gripCmd, sv, S] = superviseGrasp(S, t, boxTarget, status, q, qf, qfd, truthGrasp, U)
%SUPERVISEGRASP  One 50 Hz tick: what phase are we in, and what do we send?
%
%   [cmd, gripCmd, sv, S] = grasp.supervisor.superviseGrasp( ...
%                                S, t, boxTarget, status, q, qf, qfd, truthGrasp, U)
%
%   S           state (grasp.supervisor.startingSupervisorState)
%   t           simulation time, s
%   boxTarget   9x1 [p(3); quat(4) w,x,y,z; confidence; stamp]
%   status      20x1 from grasp.control.controlArm
%   q           7x1 arm joint positions (for "at home")
%   qf, qfd     2x1 finger positions and velocities
%   truthGrasp  3x1 TRUE grasp point, base frame. Only for recording the
%               position error, never for steering.
%   U           grasp.config.controllerSettings
%
%   cmd      9x1 to the controller [p(3); quat(4); mode; retreatId]
%   gripCmd  commanded opening per finger, m
%   sv       14x1 for logging and stopping, layout at the bottom

tcp = status(1:3);
forceAtWrist = status(5);
safetyLevel = status(8);
controllerLive = status(12) > 0.5;

% ---- start once the force sensor has been zeroed ------------------------
startRequested = (S.fsm.state == 0) && controllerLive && ~S.started;
if startRequested
    S.started = true;
    S.tStart = t;
end

% ---- where to go ---------------------------------------------------------
phase = S.fsm.state;
S = grasp.planner.nextArmTarget(S, phase, tcp, boxTarget, controllerLive, safetyLevel == 1, U);

% ---- what the phase logic gets to see ------------------------------------
obs.dt = U.dt;
obs.distance_to_target = norm(tcp - S.goal);
if ~S.haveGoal
    obs.distance_to_target = inf;
end
obs.force_at_wrist = forceAtWrist;
obs.safety_stop = safetyLevel >= 2;
obs.target_confidence = boxTarget(8);
obs.seconds_since_target = t - boxTarget(9);
obs.gains_settled = status(7) > 0.5;
width = qf(1) + qf(2);
speed = abs(qfd(1)) + abs(qfd(2));
% "Closed" means the fingers have STOPPED: fingers on a box never reach zero.
obs.gripper_closed = speed < U.gripper_still_speed && S.fsm.timeInState > U.gripper_min_close_time;
obs.object_grasped = width > 2 * U.gripper_closed + U.gripper_grasp_margin;
if S.graspSet
    obs.tcp_height_above_start = tcp(3) - S.graspZ;
else
    obs.tcp_height_above_start = 0;
end
obs.at_home = all(abs(q(:) - U.q_home(:)) < U.home_tolerance);
obs.start_requested = startRequested;

% ---- the phase logic ------------------------------------------------------
previous = S.fsm.state;
[S.fsm, faults, entered] = grasp.supervisor.decideGraspPhase(S.fsm, obs, U);
phase = S.fsm.state;
S.faultsEver = bitor(S.faultsEver, faults);

% ---- one-shot actions on entering a phase ----------------------------------
if entered
    switch phase
        case 4                                       % CLOSE_FINGERS: fix the lift reference
            S.graspZ = tcp(3);
            S.graspXY = tcp(1:2);
            S.graspSet = true;
            S.tGrasp = t;
            S.targetErr = norm(tcp - truthGrasp);
        case 7                                       % RETREAT
            S.retreatId = S.retreatId + 1;
        case {6, 8}                                  % HOLD, STOP: the end of the run
            if S.tTerminal < 0
                S.tTerminal = t;
            end
    end
end

cmd = [S.cmdPos; S.goalQuat; grasp.control.chooseControlMode(phase); S.retreatId];
gripCmd = S.grip;

% ---- is the run over? -------------------------------------------------------
done = 0;
if phase == 8
    done = 1;                                        % STOP ends it at once
elseif phase == 6 && (t - S.tTerminal) >= U.hold_time
    done = 1;                                        % held for a second: success
elseif t >= U.timeout
    done = 1;
    S.faultsEver = bitor(S.faultsEver, 512);         % run timeout
end
if previous ~= phase
    S.transitions = S.transitions + 1;
end

% ---- status -------------------------------------------------------------
%  1 phase            6 tTerminal (-1 until reached)   11 distance TCP->goal
%  2 faults now       7 tGrasp (-1 if never)           12 transitions
%  3 faults ever      8 position error at grip, m      13 jaw width, m
%  4 done             9 run started                    14 control mode sent
%  5 tStart          10 force at wrist
sv = [phase; faults; S.faultsEver; done; S.tStart; S.tTerminal; S.tGrasp; ...
      S.targetErr; S.started; forceAtWrist; obs.distance_to_target; ...
      S.transitions; width; cmd(8)];
end
