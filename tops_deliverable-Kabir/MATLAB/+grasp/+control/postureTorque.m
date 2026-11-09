function tauN = postureTorque(J, M, q, qd, C)
%POSTURETORQUE  Use the 7th joint to stay near the home posture.
%
%   tauN = grasp.control.postureTorque(J, M, q, qd, C)
%
%   The torque goes through a projection that is invisible to the hand, and
%   adds a one-sided spring near each joint limit.

L = grasp.control.taskInertia(J, M, C.singularity_damping);
Jbar = M \ (J.' * L);
N = eye(7) - J.' * Jbar.';
tau0 = zeros(7, 1);
for i = 1:7
    d = 2 * C.posture_damping_ratio * sqrt(C.posture_stiffness * max(0.05, M(i, i)));
    tau0(i) = -C.posture_stiffness * (q(i) - C.q_home(i)) - d * qd(i);
    over = q(i) - (C.q_upper(i) - C.joint_limit_margin);
    under = (C.q_lower(i) + C.joint_limit_margin) - q(i);
    if over > 0
        tau0(i) = tau0(i) - C.joint_limit_stiffness * over;
    elseif under > 0
        tau0(i) = tau0(i) + C.joint_limit_stiffness * under;
    end
end
tauN = N * tau0;
end
