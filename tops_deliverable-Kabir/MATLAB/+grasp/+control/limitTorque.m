function [tau, S] = limitTorque(tau, S, C)
%LIMITTORQUE  The last step, always: torque rate limit, then per-joint limit.
%
%   [tau, S] = grasp.control.limitTorque(tau, S, C)

step = C.torque_rate_limit * C.dt;
tau(~isfinite(tau)) = 0;
tau = S.tauPrev + min(max(tau - S.tauPrev, -step), step);
tau = min(max(tau, -C.torque_limits(:)), C.torque_limits(:));
S.tauPrev = tau;
end
