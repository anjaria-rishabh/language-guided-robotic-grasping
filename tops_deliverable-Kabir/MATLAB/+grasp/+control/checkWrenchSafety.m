function [level, reason] = checkWrenchSafety(wSafety, spike, sensorBad, moving, C)
%CHECKWRENCHSAFETY  Is the force and torque at the wrist acceptable right now?
%
%   [level, reason] = grasp.control.checkWrenchSafety(wSafety, spike, sensorBad, moving, C)
%
%   wSafety    6x1 external wrench [force; torque], 120 Hz filter
%   spike      N, how sudden the force is (grasp.control.readForceSensor)
%   sensorBad  true if the sensor reading cannot be trusted
%   moving     true while travelling, when nothing should be touching the arm
%
%   level      0 fine    1 warn: slow down    2 stop: go limp and retreat
%   reason     0 none    1 force    2 torque    3 sudden hit    4 bad sensor
%
%   The limits are lower while moving: a few newtons at the wrist during
%   free travel means something is in the way. Pure function, no state.

L = C.safety;
if moving
    forceWarn = L.force_warn_moving;   forceStop = L.force_stop_moving;
else
    forceWarn = L.force_warn_contact;  forceStop = L.force_stop_contact;
end
force = norm(wSafety(1:3));
torque = norm(wSafety(4:6));

level = 0;  reason = 0;
if sensorBad
    level = 2;  reason = 4;
elseif force > forceStop
    level = 2;  reason = 1;
elseif torque > L.torque_stop
    level = 2;  reason = 2;
elseif moving && spike > L.spike_limit
    level = 2;  reason = 3;
elseif force > forceWarn
    level = 1;  reason = 1;
elseif torque > L.torque_warn
    level = 1;  reason = 2;
end
end
