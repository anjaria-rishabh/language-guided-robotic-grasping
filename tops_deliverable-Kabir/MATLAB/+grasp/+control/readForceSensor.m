function [S, wControl, wSafety, spike, sensorBad] = readForceSensor(S, wRaw, Rh, twist, C)
%READFORCESENSOR  Raw wrist reading -> what the world is pushing on the hand with.
%
%   [S, wControl, wSafety, spike, sensorBad] = ...
%       grasp.control.readForceSensor(S, wRaw, Rh, twist, C)
%
%   wRaw       6x1 [force; torque] from the wrist sensor, hand frame
%   Rh         hand orientation in the base frame
%   twist      6x1 TCP twist
%
%   wControl   6x1 external wrench, base frame, 30 Hz, deadbanded and clamped
%   wSafety    6x1 external wrench, base frame, 120 Hz
%   spike      N, fast filter minus slow filter: large only for a sudden hit
%   sensorBad  true if the reading is invalid or has stopped changing
%
%   The sensor reports the force the hand exerts on the flange. For a hand
%   of mass m, acceleration a, gravity gw:  f_sensor = m (gw - a) + F_env.
%   So the tool's weight AND inertia are predicted and removed. What
%   remains after the at-rest zeroing is F_env.

% ---- is the sensor alive? ---------------------------------------------
invalid = any(~isfinite(wRaw));
if invalid
    wRaw = S.wRawPrev;                                 % never let a NaN reach the filters
end
if S.zeroed && all(wRaw == S.wRawPrev)
    S.frozenCount = S.frozenCount + 1;
else
    S.frozenCount = 0;
end
S.wRawPrev = wRaw;
sensorBad = invalid || S.frozenCount >= C.safety.frozen_samples;

% ---- remove the tool --------------------------------------------------
gw = [0; 0; -9.80665];
aRaw = (twist(1:3) - S.vPrev) / C.dt;
S.vPrev = twist(1:3);
S.acc = S.acc + C.alpha_accel * (aRaw - S.acc);        % filtered TCP acceleration
fTool = Rh.' * (C.tool_mass * (gw - S.acc));           % hand frame
tTool = cross(C.tool_com(:), fTool);
reading = wRaw - [fTool; tTool];

% ---- zero it: the arm is still and untouched for the first half second -
if S.zeroCount < C.zeroing_samples
    S.zeroSum = S.zeroSum + reading;
    S.zeroCount = S.zeroCount + 1;
    if S.zeroCount == C.zeroing_samples
        S.zero = S.zeroSum / C.zeroing_samples;
        S.zeroed = true;
    end
    wControl = zeros(6, 1);  wSafety = zeros(6, 1);  spike = 0;
    return
end
reading = reading - S.zero;

% ---- into the base frame, filtered twice --------------------------------
wBase = [Rh * reading(1:3); Rh * reading(4:6)];
S.wControl = S.wControl + C.alpha_control * (wBase - S.wControl);
S.wSafety  = S.wSafety  + C.alpha_safety  * (wBase - S.wSafety);
spike = norm(S.wSafety(1:3) - S.wControl(1:3));

wControl = S.wControl;
wControl(abs(wControl) < C.sensor_deadband) = 0;
wControl = min(max(wControl, -C.sensor_clamp), C.sensor_clamp);
wSafety = S.wSafety;
end
