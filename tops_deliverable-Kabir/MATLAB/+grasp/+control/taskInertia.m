function L = taskInertia(J, M, lambda)
%TASKINERTIA  Damped task-space inertia: (J M^-1 J' + lambda^2 I)^-1.
%
%   L = grasp.control.taskInertia(J, M, lambda)
%
%   The damping keeps it finite at a singularity, at a small cost in
%   accuracy there.

L = inv(J * (M \ J.') + lambda^2 * eye(6));
end
