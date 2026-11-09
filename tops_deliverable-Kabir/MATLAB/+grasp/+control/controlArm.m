function [tau, fFinger, status, S] = controlArm(S, q, qd, wRaw, cmd, gripCmd, qf, qfd, C, km)
%CONTROLARM  One 1 kHz control cycle: arm torques and finger forces.
%
%   [tau, fFinger, status, S] = grasp.control.controlArm( ...
%                                  S, q, qd, wRaw, cmd, gripCmd, qf, qfd, C, km)
%
%   S        controller state (grasp.control.startingControlState)
%   q, qd    7x1 arm joint positions / velocities
%   wRaw     6x1 raw wrist reading [force; torque], hand frame
%   cmd      9x1 [p_des(3); quat_des(4) (w,x,y,z); mode; retreatId]
%            mode comes from grasp.control.chooseControlMode
%   gripCmd  commanded opening per finger, metres
%   qf, qfd  2x1 finger positions / velocities
%   C, km    grasp.config.controllerSettings
%
%   tau      7x1 joint torques
%   fFinger  2x1 finger forces
%   status   20x1, layout at the bottom of this function
%
%   The order below is the whole controller: read the sensor, check it is
%   safe, take the target, set the stiffness, run the control law for this
%   mode, add gravity, limit the torque.

% ---- the arm, once per cycle ------------------------------------------
[T, J, M, g, cor, Th] = grasp.robot.armDynamics(q, qd, km);
p = T(1:3, 4);  Rtcp = T(1:3, 1:3);
twist = J * qd;                                        % [v; w] of the TCP

% ---- force sensor and safety ------------------------------------------
[S, wControl, wSafety, spike, sensorBad] = grasp.control.readForceSensor(S, wRaw, Th(1:3, 1:3), twist, C);
mode = cmd(8);
moving = mode == 1;
[level, reason] = grasp.control.checkWrenchSafety(wSafety, spike, sensorBad, moving, C);
if level == 2 && ~S.latched
    S.latched = true;                                  % limp until the retreat request
    S.stopReason = reason;
end

% ---- the target ---------------------------------------------------------
S = acceptTarget(S, cmd(1:3), cmd(4:7), p, Rtcp, C);
if cmd(9) ~= S.retreatId
    S = startRetreat(S, cmd(9), p, Rtcp, C);
end
if S.retreating
    d = S.retreatGoal - S.pDes;                        % never a jump: the spring is stiff
    S.pDes = S.pDes + min(1, C.retreat_speed * C.dt / max(norm(d), 1e-9)) * d;
end

% ---- stiffness and the control law --------------------------------------
[S, settled] = updateStiffness(S, moving, C);
e = grasp.control.poseError(S.pDes, S.qDes, p, Rtcp);
L = grasp.control.taskInertia(J, M, C.singularity_damping);
if moving && ~S.latched
    [F, S] = grasp.control.admittanceControl(S, e, twist, L, wControl, C);
else
    regulate = mode == 2 && ~S.latched;
    approach = Rtcp * C.force_axis_tool(:);
    [F, S] = grasp.control.impedanceControl(S, e, twist, L, wControl, approach, regulate, C);
end

% ---- torque -------------------------------------------------------------
tauPrev = S.tauPrev;
tau = J.' * F + grasp.control.postureTorque(J, M, q, qd, C) + g + cor;
[tau, S] = grasp.control.limitTorque(tau, S, C);

% ---- gripper ------------------------------------------------------------
[fFinger, S] = grasp.control.controlGripper(S, gripCmd, qf, qfd, C);

% ---- status -------------------------------------------------------------
%  1-3  TCP position (base)              12  sensor zeroed
%  4    force, control filter            13-15 accepted target position
%  5    force, safety filter             16  TCP speed
%  6    stiffness in effect, N/m (x)     17  wrist torque, N.m
%  7    stiffness has settled            18  admittance offset, m
%  8    safety level 0 ok 1 warn 2 stop  19  control mode in use
%  9    reason of the last stop          20  largest joint torque change, N.m
%  10   distance TCP -> accepted target
%  11   stopped (limp)
status = zeros(20, 1);
status(1:3) = p;
status(4) = norm(wControl(1:3));
status(5) = norm(wSafety(1:3));
status(6) = S.K(1);
status(7) = settled;
status(8) = max(level, 2 * double(S.latched));
status(9) = S.stopReason;
status(10) = norm(S.pDes - p);
status(11) = S.latched;
status(12) = S.zeroed;
status(13:15) = S.pDes;
status(16) = norm(twist(1:3));
status(17) = norm(wSafety(4:6));
status(18) = norm(S.xa(1:3));
status(19) = mode;
status(20) = max(abs(tau - tauPrev));
end

% =========================================================================
function S = acceptTarget(S, pNew, qNew, p, Rtcp, C)
% Take a new target unless it is implausibly far from the current one, so a
% wild pose from whatever is planning cannot throw the arm across the table.
if ~S.initialised
    S.pDes = p;  S.qDes = grasp.control.rotationToQuaternion(Rtcp);  S.initialised = true;
end
if S.retreating || norm(qNew) < 0.5                    % no orientation yet: not a target
    return
end
if norm(pNew - S.pDes) <= C.max_target_jump
    S.pDes = pNew;
    S.qDes = qNew / norm(qNew);
end
end

function S = startRetreat(S, id, p, Rtcp, C)
% Back away from the approach direction, from wherever the hand is now.
S.retreatId = id;
S.pDes = p;
S.qDes = grasp.control.rotationToQuaternion(Rtcp);
S.retreatGoal = p - Rtcp * C.force_axis_tool(:) * C.retreat_distance;
S.latched = false;
S.forceI = 0;
S.retreating = true;
end

function [S, settled] = updateStiffness(S, moving, C)
% Stiffness only ever ramps: changing it in a step would jerk the arm.
if S.latched
    Kt = zeros(6, 1);
elseif moving
    Kt = C.K_moving(:);
else
    Kt = C.K_gripping(:);
end
if S.latched || S.retreating
    step = C.K_stop_step(:);
else
    step = C.K_step(:);
end
S.K = S.K + min(max(Kt - S.K, -step), step);
settled = all(abs(Kt - S.K) <= 1e-3 * max(1, max(abs(Kt))));
end
