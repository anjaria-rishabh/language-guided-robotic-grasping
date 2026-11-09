function [F, S] = admittanceControl(S, e, twist, L, wControl, C)
%ADMITTANCECONTROL  Stiff tracking of a reference that measured force can move.
%
%   [F, S] = grasp.control.admittanceControl(S, e, twist, L, wControl, C)
%
%   e         6x1 pose error to the commanded target
%   twist     6x1 TCP twist
%   L         6x6 task-space inertia
%   wControl  6x1 external wrench, base frame
%   F         6x1 task-space force to apply
%
%   Used while the arm travels. With nothing touching it the arm follows the
%   target accurately. If something pushes it, the reference gives way.

S = grasp.control.updateVirtualOffset(S, wControl, C);
K = S.K;
D = grasp.control.taskDamping(K, L, C);
F = K .* (e + S.xa) + D .* (S.va - twist);
end
