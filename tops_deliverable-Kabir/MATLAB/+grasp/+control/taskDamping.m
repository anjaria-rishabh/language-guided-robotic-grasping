function D = taskDamping(K, L, C)
%TASKDAMPING  Damping for a task-space stiffness, from the arm's inertia.
%
%   D = grasp.control.taskDamping(K, L, C)
%
%   K  6x1 stiffness in effect      L  6x6 task-space inertia
%
%   Never computed from less than the gripping stiffness, so the arm stays
%   damped when a safety stop takes its stiffness to zero.

D = 2 * C.damping_ratio * sqrt(max(K, C.K_gripping(:)) .* max(0.05, abs(diag(L))));
end
