function [T, J, M, g, c, Thand] = armDynamics(q, qd, km)
%ARMDYNAMICS  Everything the controllers need about the arm, in one call.
%
%   [T, J, M, g, c, Thand] = grasp.robot.armDynamics(q, qd, km)
%
%   q, qd   7x1 joint positions (rad) and velocities (rad/s)
%   km      the numeric model from grasp.robot.armModelForController
%   T       4x4 pose of the TCP (panda_hand_tcp) in the base frame
%   J       6x7 geometric Jacobian at the TCP, base frame, rows
%           [vx vy vz wx wy wz] (linear first)
%   M       7x7 joint-space mass matrix M(q)
%   g       7x1 torque that holds the arm against gravity, g(q)
%   c       7x1 Coriolis and centrifugal torque C(q, qd) * qd
%   Thand   4x4 pose of the hand frame (where the F/T sensor reads)
%
%   From the manipulator equation (Theory 04):
%       M(q) qdd + C(q, qd) qd + g(q) = tau + J' F_ext
%
%   Written out by hand rather than called from a library, for two
%   reasons: it compiles into the 1 kHz Simulink controller with no
%   restrictions, and every line is the textbook algorithm, so it can be
%   read against Theory 03/04. tests/test_dynamics.m checks it against
%   Robotics System Toolbox (an independent implementation) and against
%   the URDF.
%
%   Algorithms
%     * forward kinematics: chain the fixed joint origins and Rz(q_i)
%     * Jacobian: column i = [z_i x (p_tcp - p_i); z_i], all in the base
%     * g, c, M: recursive Newton-Euler (RNEA). One pass with gravity and
%       zero velocity gives g; one pass with velocity and no gravity gives
%       c; one pass per joint with a unit acceleration and nothing else
%       gives that column of M.

gvec = [0; 0; -9.80665];

% ---- forward kinematics ------------------------------------------------
R0 = zeros(3, 3, 7);  p0 = zeros(3, 7);     % pose of each link frame in base
Rk = eye(3);  pk = zeros(3, 1);
for i = 1:7
    pk = pk + Rk * km.pj(:, i);
    Rk = Rk * km.Rj(:, :, i) * rz(q(i));
    R0(:, :, i) = Rk;  p0(:, i) = pk;
end
T7 = [Rk, pk; 0 0 0 1];
T = T7 * km.T7tcp;
Thand = T7 * km.T7hand;

% ---- Jacobian at the TCP -----------------------------------------------
J = zeros(6, 7);
ptcp = T(1:3, 4);
for i = 1:7
    z = R0(:, 3, i);                         % joint i's axis, in the base
    J(1:3, i) = cross(z, ptcp - p0(:, i));
    J(4:6, i) = z;
end

% ---- dynamics by recursive Newton-Euler --------------------------------
zero7 = zeros(7, 1);
g = rnea(q, zero7, zero7, gvec, km);
c = rnea(q, qd, zero7, [0; 0; 0], km);
M = zeros(7, 7);
for j = 1:7
    e = zero7;  e(j) = 1;
    M(:, j) = rnea(q, zero7, e, [0; 0; 0], km);
end
M = 0.5 * (M + M.');                         % exact in theory; tidy the rounding
end

% =========================================================================
function tau = rnea(q, qd, qdd, gvec, km)
% Joint torques for a given motion, by recursive Newton-Euler.
% Every vector of link i is expressed in link i's own frame.
z = [0; 0; 1];
w = zeros(3, 7);  wd = zeros(3, 7);  a = zeros(3, 7);
F = zeros(3, 7);  N = zeros(3, 7);
Rl = zeros(3, 3, 7);                          % rotation from frame i-1 to i

% Outward: velocities and accelerations. Gravity enters as an upward
% acceleration of the base, which is the standard trick: it then appears
% in every link's inertial force without a separate term.
wp = [0; 0; 0];  wdp = [0; 0; 0];  ap = -gvec;
for i = 1:7
    Ri = km.Rj(:, :, i) * rz(q(i));
    Rl(:, :, i) = Ri;
    Rt = Ri.';
    p = km.pj(:, i);                          % joint origin in frame i-1
    w(:, i)  = Rt * wp + qd(i) * z;
    wd(:, i) = Rt * wdp + qdd(i) * z + cross(Rt * wp, qd(i) * z);
    a(:, i)  = Rt * (ap + cross(wdp, p) + cross(wp, cross(wp, p)));
    ci = km.c(:, i);
    ac = a(:, i) + cross(wd(:, i), ci) + cross(w(:, i), cross(w(:, i), ci));
    F(:, i) = km.m(i) * ac;
    Ii = km.I(:, :, i);
    N(:, i) = Ii * wd(:, i) + cross(w(:, i), Ii * w(:, i));
    wp = w(:, i);  wdp = wd(:, i);  ap = a(:, i);
end

% Inward: forces and moments, projected onto each joint axis.
tau = zeros(7, 1);
f = [0; 0; 0];  n = [0; 0; 0];
for i = 7:-1:1
    if i < 7
        Rn = Rl(:, :, i + 1);  pn = km.pj(:, i + 1);
        fc = Rn * f;                           % child's force, in frame i
        n = N(:, i) + Rn * n + cross(km.c(:, i), F(:, i)) + cross(pn, fc);
        f = fc + F(:, i);
    else
        n = N(:, i) + cross(km.c(:, i), F(:, i));
        f = F(:, i);
    end
    tau(i) = n.' * z;
end
end

function R = rz(a)
R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end
