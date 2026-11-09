function R = pandaPublishedData(inertials)
%PANDAPUBLISHEDDATA  The Franka Emika Panda: published data for every parameter.
%
%   R = grasp.robot.pandaPublishedData()             % identified links (default)
%   R = grasp.robot.pandaPublishedData('franka')     % Franka's CAD inertials instead
%
%   Moving links and joint friction come from the identification on a real
%   arm by Gaz, Cognetti, Oliva, Robuffo Giordano and De Luca, IEEE RA-L
%   4(4), 2019, supplementary material (rev. 15 Oct 2019): Tables VIII, IX
%   (estimates p-hat) and Table I. The inertias are about each link's centre
%   of mass, in the link frame; tests/test_literature.m confirms this by
%   rebuilding the paper's Table III from them. c1z has no effect on the
%   dynamics and is not identified, so Franka's value is used. The paper
%   describes the bare 18 kg arm and mentions no tool, so the Hand is added
%   as a separate body.
%
%   Torque-rate limit: Franka's robot specifications, FER table.
%   Hand: Franka Hand product manual R50010 v1.2.
%
%   The kinematics, limits, base and hand inertials are copied unchanged
%   from Franka's own description
%   package, franka_description (github.com/frankarobotics/franka_description,
%   commit 7aeeddc, Apache-2.0, (c) Franka Robotics GmbH):
%       robots/fer/kinematics.yaml               -> R.joint_xyz, R.joint_rpy, R.flange_z
%       robots/fer/joint_limits.yaml             -> R.q_lower, R.q_upper, R.qd_max, R.tau_max
%       robots/fer/inertials.yaml                -> R.link(1..8)  (= link0..link7)
%       end_effectors/franka_hand/inertials.yaml -> R.hand, R.finger
%   ("FER" = Franka Emika Robot = the Panda.) The licence text is in
%   robot/LICENSE-franka_description-Apache-2.0.
%
%   With 'franka', ONE value is not Franka's: link4's inertia tensor. Franka's breaks the
%   triangle inequality (I1 + I2 >= I3, which every rigid body obeys) by
%   1.10e-3 kg m^2, so no physical body can have it, and a physics engine
%   rejects it. It is replaced by the nearest valid tensor: same mass, same
%   centre of mass, same principal axes, principal moments moved by the
%   minimum-norm amount (+d/3, +d/3, -d/3 with d = 1.10e-3 + 1e-6). That
%   changes the tensor by 4.1 % (Frobenius norm). Franka's original is kept
%   in the comment beside it.
%
%   Units: metres, radians, kilograms, kg m^2, N m, rad/s.
%   Inertia tensors are about each body's centre of mass, in its link frame.

if nargin < 1, inertials = 'gaz2019'; end

% ---- kinematics: origin of joint i in link i-1's frame ---------------------
%        x        y        z        roll                pitch  yaw
K = [    0        0        0.333    0                   0      0
         0        0        0       -1.570796326794897   0      0
         0       -0.316    0        1.570796326794897   0      0
         0.0825   0        0        1.570796326794897   0      0
        -0.0825   0.384    0       -1.570796326794897   0      0
         0        0        0        1.570796326794897   0      0
         0.088    0        0        1.570796326794897   0      0 ];
R.joint_xyz = K(:, 1:3);
R.joint_rpy = K(:, 4:6);
R.flange_z = 0.107;                  % link7 -> link8, the flange

% ---- joint limits -----------------------------------------------------------
R.q_lower = [-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973];   % rad
R.q_upper = [ 2.8973  1.7628  2.8973 -0.0698  2.8973  3.7525  2.8973];   % rad
R.qd_max  = [ 2.175   2.175   2.175   2.175   2.61    2.61    2.61  ];   % rad/s
R.tau_max = [87      87      87      87      12      12      12    ];     % N m

% ---- joint dynamics -----------------------------------------------------------
% Gaz et al. Table I, per joint:
%   tau_f = phi1 / (1 + exp(-phi2 (qd + phi3))) - phi1 / (1 + exp(-phi2 phi3))
% This is the joint's whole friction, viscous part included, so the revolute
% joints carry no damping of their own.
R.friction_phi1 = [0.54615  0.87224  0.64068  1.2794   0.83904  0.30301  0.56489  ];   % N m
R.friction_phi2 = [5.1181   9.0657   10.136   5.5903   8.3469   17.133   10.336   ];   % s/rad
R.friction_phi3 = [0.039533 0.025882 -0.04607 0.036194 0.026226 -0.021047 0.0035526];   % rad/s
R.damping  = zeros(1, 7);                                     % N m s/rad
R.tau_rate_max = 1000;                                        % N m/s, every joint

% ---- inertials: mass, centre of mass, tensor about the COM --------------------
%                 mass      centre of mass (link frame)
R.link(1) = inertial(2.3966, [-0.0172 0.0004 0.0745], ...                 % link0
    [0.0090 0.0000 0.0020; 0.0000 0.0115 0.0000; 0.0020 0.0000 0.0085]);
R.link(2) = inertial(2.7907, [0.00033 -0.02204 -0.04762], ...             % link1
    [0.01564782655 0.00000531236 -0.00003676721
     0.00000531236 0.01439883526 -0.00248480843
    -0.00003676721 -0.00248480843 0.00500443991]);
R.link(3) = inertial(2.5420, [0.00038 -0.09211 0.01908], ...              % link2
    [0.01662427728 0.00003086077 0.00000835940
     0.00003086077 0.00462501261 0.00355899830
     0.00000835940 0.00355899830 0.01545124526]);
R.link(4) = inertial(2.2513, [0.05152 0.01696 -0.02971], ...              % link3
    [0.00631741992 0.00097309955 0.00292525834
     0.00097309955 0.00866285493 0.00093469224
     0.00292525834 0.00093469224 0.00657804051]);
% link4: Franka's tensor is NOT physical; corrected as described above.
% Franka's:   [ 0.00784585566 -0.00339591957  0.00158704074
%              -0.00339591957  0.00649543136 -0.00181477884
%               0.00158704074 -0.00181477884  0.01003079917]
R.link(5) = inertial(2.2037, [-0.05113 0.05825 0.01698], ...              % link4
    [ 0.00798256086 -0.00319112589  0.00131438893
     -0.00319112589  0.00668099429 -0.00157271745
      0.00131438893 -0.00157271745  0.01007591063]);
R.link(6) = inertial(2.2855, [-0.00005 0.03730 -0.09280], ...             % link5
    [ 0.02297014781 -0.00000949345 -0.00002063156
     -0.00000949345  0.02095060919  0.00382345782
     -0.00002063156  0.00382345782  0.00430606551]);
R.link(7) = inertial(1.353, [0.06572 -0.00371 0.00153], ...               % link6
    [ 0.00087964522 -0.00021487814 -0.00011911662
     -0.00021487814  0.00277796968  0.00001274322
     -0.00011911662  0.00001274322  0.00286701969]);
R.link(8) = inertial(0.35973, [0.00089 -0.00044 0.05491], ...             % link7
    [ 0.00019541063  0.00000165231  0.00000148826
      0.00000165231  0.00019210361 -0.00000131132
      0.00000148826 -0.00000131132  0.00017936256]);

if strcmp(inertials, 'gaz2019')
    R.link(2:8) = gazLinks(R.link(2).com(3));
end
R.inertial_source = inertials;

% ---- the Franka Hand and one finger --------------------------------------------
R.hand = inertial(0.6544, [-0.0000376 0.0119128 0.0207260], ...
    [0.00186 0.00000 0.00000; 0.00000 0.00030 -0.00002; 0.00000 -0.00002 0.00174]);
R.finger = inertial(0.0291, [0 0.0152850 0.0219675], ...
    [0.00000849 0.00000000 0.00000000; 0.00000000 0.00000853 -0.00000106; 0.00000000 -0.00000106 0.00000177]);
end

function L = gazLinks(c1z)
% Gaz et al. Tables VIII and IX, links 1..7.
m = [4.970684 0.646926 3.228604 3.587895 1.225946 1.666555 7.35522e-01];
c = [ 3.875e-03   2.081e-03   c1z
     -3.141e-03  -2.872e-02   3.495e-03
      2.7518e-02  3.9252e-02 -6.6502e-02
     -5.317e-02   1.04419e-01 2.7454e-02
     -1.1953e-02  4.1065e-02 -3.8437e-02
      6.0149e-02 -1.4117e-02 -1.0517e-02
      1.0517e-02 -4.252e-03  6.1597e-02];
%        Ixx         Ixy         Ixz         Iyy         Iyz         Izz
J = [ 7.0337e-01 -1.3900e-04  6.7720e-03  7.0661e-01  1.9169e-02  9.1170e-03
      7.9620e-03 -3.9250e-03  1.0254e-02  2.8110e-02  7.0400e-04  2.5995e-02
      3.7242e-02 -4.7610e-03 -1.1396e-02  3.6155e-02 -1.2805e-02  1.0830e-02
      2.5853e-02  7.7960e-03 -1.3320e-03  1.9552e-02  8.6410e-03  2.8323e-02
      3.5549e-02 -2.1170e-03 -4.0370e-03  2.9474e-02  2.2900e-04  8.6270e-03
      1.9640e-03  1.0900e-04 -1.1580e-03  4.3540e-03  3.4100e-04  5.4330e-03
      1.2516e-02 -4.2800e-04 -1.1960e-03  1.0027e-02 -7.4100e-04  4.8150e-03];
for i = 7:-1:1
    r = J(i, :);
    L(i) = inertial(m(i), c(i, :), [r(1) r(2) r(3); r(2) r(4) r(5); r(3) r(5) r(6)]);
end
end

function s = inertial(mass, com, I)
s = struct('mass', mass, 'com', com, 'I', I);
end
