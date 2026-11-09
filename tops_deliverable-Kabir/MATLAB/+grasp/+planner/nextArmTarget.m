function S = nextArmTarget(S, phase, tcp, boxTarget, controllerLive, warned, U)
%NEXTARMTARGET  Where the arm should go this tick, and how wide the gripper.
%
%   S = grasp.planner.nextArmTarget(S, phase, tcp, boxTarget, controllerLive, warned, U)
%
%   phase           grasp phase code (grasp.supervisor.phaseNames)
%   tcp             3x1 current TCP position, base frame
%   boxTarget       9x1 [p(3); quat(4) w,x,y,z; confidence; stamp]
%   controllerLive  true once the force sensor has been zeroed
%   warned          true if the safety monitor wants the arm slowed
%
%   Writes S.goal, S.goalQuat, S.haveGoal (where the arm is headed) and
%   S.cmdPos, S.grip (what is sent this tick).
%
%   THIS IS THE PLACE A VLA PLUGS IN. Today it is a straight line to whatever
%   boxTarget says. A learned planner replaces this body and sets the same
%   fields at its own rate: the controller, safety monitor and phases do not
%   change.

% ---- goal -------------------------------------------------------------
if phase == 2 || phase == 3                         % MOVE_ABOVE_BOX, DESCEND: go to the box
    if boxTarget(8) > 0
        S.goal = boxTarget(1:3);
        S.goalQuat = boxTarget(4:7);
        S.haveGoal = true;
    end
elseif (phase == 5 || phase == 6) && S.graspSet     % LIFT, HOLD: straight up from the grip
    S.goal = [S.graspXY; S.graspZ + U.lift_height];
end

% ---- commanded pose: slewed toward the goal ------------------------------
% Stepping it would ask the spring for (distance x stiffness) of force at
% once. It starts where the TCP is, but only once the controller is live:
% before that, the TCP position it reports is still zero.
if ~S.cmdInit && controllerLive
    S.cmdPos = tcp;  S.cmdInit = true;
end
if phase ~= 0 && phase ~= 8 && S.haveGoal && S.cmdInit
    if phase == 3 || phase == 4
        v = U.descend_speed;
    else
        v = U.move_speed;
    end
    if warned
        v = v * U.warn_speed_scale;
    end
    d = S.goal - S.cmdPos;
    n = norm(d);
    step = max(1e-4, v * U.dt);
    if n > step
        S.cmdPos = S.cmdPos + d * (step / n);
    else
        S.cmdPos = S.goal;
    end
end

% ---- gripper ------------------------------------------------------------
if phase == 4 || phase == 5 || phase == 6           % CLOSE_FINGERS, LIFT, HOLD
    S.grip = U.gripper_closed;
elseif phase == 0 || phase == 1 || phase == 7       % WAIT, GO_HOME, RETREAT
    S.grip = U.gripper_open;
end
end
