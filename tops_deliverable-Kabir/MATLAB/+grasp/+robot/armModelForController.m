function km = armModelForController(P)
%ARMMODELFORCONTROLLER  The arm as plain numbers, for grasp.robot.armDynamics.
%
%   km = grasp.robot.armModelForController(P)          % P = grasp.config.settings()
%
%   A numeric struct (no objects, no strings) so it can be passed into the
%   compiled Simulink controller as a parameter:
%
%     km.Rj(:,:,i), km.pj(:,i)  joint i's fixed origin in link i-1's frame
%                               (rotation, translation); the joint itself
%                               then rotates about the new frame's z axis
%     km.m(i), km.c(:,i)        mass and centre of mass of moving body i
%                               (link i's frame)
%     km.I(:,:,i)               its inertia tensor about that COM
%     km.T7tcp                  4x4 pose of the TCP in link7's frame
%     km.T7hand                 4x4 pose of the hand (= sensor) in link7's frame
%
%   Body 7 is link7 together with everything bolted to it: the hand and both
%   fingers (lumped half-open, as in grasp.robot.armRigidBodyTree). They move as one
%   rigid body, so for dynamics they ARE one body.

R = P.robot;
km.Rj = zeros(3, 3, 7);
km.pj = zeros(3, 7);
km.m = zeros(1, 7);
km.c = zeros(3, 7);
km.I = zeros(3, 3, 7);

for i = 1:7
    rpy = R.joint_rpy(i, :);
    km.Rj(:, :, i) = Rz(rpy(3)) * Ry(rpy(2)) * Rx(rpy(1));   % fixed-axis rpy
    km.pj(:, i) = R.joint_xyz(i, :).';
    km.m(i) = R.link(i + 1).mass;
    km.c(:, i) = R.link(i + 1).com(:);
    km.I(:, :, i) = R.link(i + 1).I;
end

% Hand pose in link7: up the flange, then the -45 deg mount.
km.T7hand = [Rz(P.robot.hand_yaw), [0; 0; R.flange_z]; 0 0 0 1];
km.T7tcp  = km.T7hand * [eye(3), [0; 0; P.robot.tcp_offset]; 0 0 0 1];

% Fold the hand (+ fingers) into body 7.
hb = lumpedHand(P);                               % in the hand frame
Rh = km.T7hand(1:3, 1:3);  ph = km.T7hand(1:3, 4);
ch = ph + Rh * hb.com(:);                         % hand COM in link7's frame
Ih = Rh * hb.I * Rh.';                            % about its COM, link7 axes
[km.m(7), km.c(:, 7), km.I(:, :, 7)] = combine(km.m(7), km.c(:, 7), km.I(:, :, 7), hb.mass, ch, Ih);
end

% =========================================================================
% Elementary rotations, in RADIANS. (MATLAB's own rotx/roty/rotz take
% degrees; using them here would be a silent factor of 57.)
function R = Rx(a), R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
function R = Ry(a), R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end
function R = Rz(a), R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end

function [m, c, I] = combine(m1, c1, I1, m2, c2, I2)
% Two rigid bodies in one frame -> one, about the combined COM.
m = m1 + m2;
c = (m1 * c1 + m2 * c2) / m;
I = I1 + m1 * pa(c1 - c) + I2 + m2 * pa(c2 - c);
end

function S = pa(d)
% Parallel-axis term for offset d: |d|^2 E - d d'.
S = (d.' * d) * eye(3) - d * d.';
end

function s = lumpedHand(P)
% Hand + both fingers (half open) about their combined COM, hand frame.
R = P.robot;
f = R.finger;  h = R.hand;
q = 0.5 * R.finger_max;
Rz = diag([-1 -1 1]);                       % the right finger is turned 180 deg
cL = [0; q; R.finger_z] + f.com(:);
cR = [0; -q; R.finger_z] + Rz * f.com(:);
[m, c, I] = combine(h.mass, h.com(:), h.I, f.mass, cL, f.I);
[m, c, I] = combine(m, c, I, f.mass, cR, Rz * f.I * Rz.');
s = struct('mass', m, 'com', c.', 'I', I);
end
