function [fRegulate, K, S] = regulateContactForce(S, wControl, approach, regulate, C)
%REGULATECONTACTFORCE  Hold a set force along the approach axis during contact.
%
%   [fRegulate, K, S] = grasp.control.regulateContactForce(S, wControl, approach, regulate, C)
%
%   wControl   6x1 external wrench, base frame
%   approach   3x1 approach axis, base frame
%   regulate   true while descending and not stopped
%
%   fRegulate  6x1 task-space force from the PI loop
%   K          6x1 stiffness to use (softened along the approach axis)
%
%   The loop runs only when something pushes back along the approach axis.
%   A sideways force is not pressing, so it does not switch the loop on.

fRegulate = zeros(6, 1);
K = S.K;
pressing = -dot(wControl(1:3), approach);              % pushing back at the hand = +
if ~regulate || pressing < C.force_contact_threshold
    S.forceI = 0;
    return
end

err = C.force_target - pressing;
next = S.forceI + C.force_ki * err * C.dt;
S.forceI = min(max(next, -C.force_integral_limit), C.force_integral_limit);
fRegulate(1:3) = approach * (C.force_kp * err + S.forceI);

% Position and force along one axis are one degree of freedom: soften the
% spring along that axis instead of letting the two fight.
K(1:3) = S.K(1:3) .* (1 - abs(approach) * (1 - C.force_axis_stiffness_scale));
end
