function robot = armRigidBodyTree(P)
%ARMRIGIDBODYTREE  The Panda as a rigidBodyTree, from Franka's data.
%
%   robot = grasp.robot.armRigidBodyTree(P)        % P = grasp.config.settings()
%
%   Forward kinematics, the Jacobian, the mass matrix, gravity and Coriolis
%   torques, from the Robotics System Toolbox. The physics plant is a
%   separate Simscape Multibody model built from the same numbers
%   (grasp.sim.buildArmModel), so controller and plant agree by construction.
%
%   Chain:  panda_link0 -j1..j7-> panda_link7 -fixed-> panda_link8 (flange)
%           -fixed, -45 deg-> panda_hand -fixed-> panda_hand_tcp
%
%   The fingers are lumped into the hand body at a nominal half-open
%   position (their COMs move by a few millimetres with the jaw, which is
%   negligible for gravity compensation).
%
%   Two conventions to know when reading the code that uses this:
%     * rigidBody.Inertia is about the BODY FRAME, not the centre of mass,
%       so each Franka tensor (given about its COM) is shifted with the
%       parallel-axis theorem below.
%     * geometricJacobian returns [angular; linear]. The controllers
%       reorder it to [linear; angular].

R = P.robot;
robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 12);
robot.BaseName = 'panda_link0';
robot.Gravity = [0 0 -9.80665];

parent = 'panda_link0';
for j = 1:7
    name = sprintf('panda_link%d', j);
    body = rigidBody(name);
    jnt = rigidBodyJoint(sprintf('panda_joint%d', j), 'revolute');
    jnt.JointAxis = [0 0 1];
    jnt.PositionLimits = [R.q_lower(j), R.q_upper(j)];
    jnt.HomePosition = P.robot.q_home(j);
    setFixedTransform(jnt, originTform(R.joint_xyz(j, :), R.joint_rpy(j, :)));
    body.Joint = jnt;
    setInertial(body, R.link(j + 1));
    addBody(robot, body, parent);
    parent = name;
end

% Flange: massless frame 107 mm along link7's z.
addFixed(robot, 'panda_link8', 'panda_link7', trvec2tform([0 0 R.flange_z]));

% Hand (+ both fingers lumped), mounted at -45 deg about the flange's z.
hand = rigidBody('panda_hand');
jh = rigidBodyJoint('panda_hand_joint', 'fixed');
setFixedTransform(jh, axang2tform([0 0 1 P.robot.hand_yaw]));
hand.Joint = jh;
setInertial(hand, lumpHand(P));
addBody(robot, hand, 'panda_link8');

% The TCP: between the fingertip pads, 103.4 mm out along the hand's z.
addFixed(robot, 'panda_hand_tcp', 'panda_hand', trvec2tform([0 0 P.robot.tcp_offset]));
end

% =========================================================================
function T = originTform(xyz, rpy)
% URDF/Franka origin: translate, then rotate by fixed-axis roll-pitch-yaw,
% i.e. R = Rz(yaw) * Ry(pitch) * Rx(roll).
T = trvec2tform(xyz) * eul2tform([rpy(3) rpy(2) rpy(1)], 'ZYX');
end

function addFixed(robot, name, parent, T)
b = rigidBody(name);
j = rigidBodyJoint([name '_joint'], 'fixed');
setFixedTransform(j, T);
b.Joint = j;
b.Mass = 0;                       % a frame, nothing more
b.Inertia = [0 0 0 0 0 0];
addBody(robot, b, parent);
end

function setInertial(body, s)
% Franka gives the tensor about the COM; rigidBody wants it about the body
% frame origin: I_o = I_c + m (|c|^2 E - c c').
c = s.com(:);
Io = s.I + s.mass * ((c.' * c) * eye(3) - (c * c.'));
body.Mass = s.mass;
body.CenterOfMass = c.';
body.Inertia = [Io(1,1) Io(2,2) Io(3,3) Io(2,3) Io(1,3) Io(1,2)];  % [xx yy zz yz xz xy]
end

function s = lumpHand(P)
% Hand + two fingers as one rigid body, in the hand frame, about its COM.
R = P.robot;
h = R.hand;  f = R.finger;
q = 0.5 * R.finger_max;                      % nominal half-open fingers
% Left finger frame: hand z + finger_z, displaced +q along y.
% Right finger frame: the same, rotated 180 deg about z (so its +y is -y).
Rz = [-1 0 0; 0 -1 0; 0 0 1];
cL = [0 q R.finger_z] + f.com;
cR = [0 -q R.finger_z] + (Rz * f.com(:)).';
IL = f.I;  IR = Rz * f.I * Rz.';

m = h.mass + 2 * f.mass;
c = (h.mass * h.com + f.mass * cL + f.mass * cR) / m;
% Parallel axis: a body's tensor about the combined COM c, where d is the
% offset (row vector) from c to the body's own COM.
shift = @(Ic, mi, ci) Ic + mi * (norm(ci - c)^2 * eye(3) - (ci - c).' * (ci - c));
I = shift(h.I, h.mass, h.com) + shift(IL, f.mass, cL) + shift(IR, f.mass, cR);
s = struct('mass', m, 'com', c, 'I', I);
end
