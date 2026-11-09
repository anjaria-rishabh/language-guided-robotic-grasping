function [F, S] = impedanceControl(S, e, twist, L, wControl, approach, regulate, C)
%IMPEDANCECONTROL  A soft spring and damper between the hand and its target.
%
%   [F, S] = grasp.control.impedanceControl(S, e, twist, L, wControl, approach, regulate, C)
%
%   e         6x1 pose error to the commanded target
%   twist     6x1 TCP twist
%   L         6x6 task-space inertia
%   wControl  6x1 external wrench, base frame
%   approach  3x1 approach axis, base frame
%   regulate  true to run contact force regulation
%   F         6x1 task-space force to apply
%
%   Used near and during contact. Any admittance offset left over from
%   travel fades out here instead of jumping to zero at the switch.

S = grasp.control.updateVirtualOffset(S, zeros(6, 1), C);
[fRegulate, K, S] = grasp.control.regulateContactForce(S, wControl, approach, regulate, C);
D = grasp.control.taskDamping(S.K, L, C);
F = K .* (e + S.xa) + fRegulate - D .* twist;
end
