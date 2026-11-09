function tests = test_arm_dynamics
%TEST_ARM_DYNAMICS  The hand-written arm dynamics against an independent reference.
%
%   grasp.robot.armDynamics (forward kinematics, Jacobian, recursive
%   Newton-Euler) is checked against MathWorks' Robotics System Toolbox
%   running on the same Franka data (grasp.robot.armRigidBodyTree). Two separate
%   implementations agreeing to rounding error is the evidence that the
%   1 kHz controller's model is right.
%
%   Run:  runtests('tests')     or     tops test
tests = functiontests(localfunctions);
end

function setupOnce(tc)
tc.TestData.P = grasp.config.settings();
tc.TestData.km = grasp.robot.armModelForController(tc.TestData.P);
tc.TestData.tree = grasp.robot.armRigidBodyTree(tc.TestData.P);
end

function testHomeTcpIsFrankasReadyPose(tc)
% Franka's "ready" pose puts the TCP of the Panda + Hand at (0.307, 0, 0.487) m.
P = tc.TestData.P;
T = grasp.robot.armDynamics(P.robot.q_home(:), zeros(7, 1), tc.TestData.km);
tc.verifyEqual(T(1:3, 4), [0.307; 0; 0.487], 'AbsTol', 1e-3);
% ... with the gripper pointing straight down (TCP z = -world z).
tc.verifyEqual(T(1:3, 3), [0; 0; -1], 'AbsTol', 1e-3);
end

function testAgreesWithRoboticsSystemToolbox(tc)
P = tc.TestData.P;  km = tc.TestData.km;  t = tc.TestData.tree;
rng(1);
for k = 1:20
    q = P.robot.q_home(:) + 0.4 * randn(7, 1);
    qd = randn(7, 1);
    [T, J, M, g, c] = grasp.robot.armDynamics(q, qd, km);
    Jr = geometricJacobian(t, q, 'panda_hand_tcp');
    tc.verifyEqual(T, getTransform(t, q, 'panda_hand_tcp'), 'AbsTol', 1e-12);
    tc.verifyEqual(J, [Jr(4:6, :); Jr(1:3, :)], 'AbsTol', 1e-12);
    tc.verifyEqual(M, massMatrix(t, q), 'AbsTol', 1e-12);
    tc.verifyEqual(g, gravityTorque(t, q), 'AbsTol', 1e-10);
    tc.verifyEqual(c, velocityProduct(t, q, qd), 'AbsTol', 1e-10);
end
end

function testMassMatrixIsSymmetricPositiveDefinite(tc)
P = tc.TestData.P;
[~, ~, M] = grasp.robot.armDynamics(P.robot.q_home(:), zeros(7, 1), tc.TestData.km);
tc.verifyEqual(M, M.', 'AbsTol', 1e-14);
tc.verifyGreaterThan(min(eig(M)), 0);
end

function testEveryInertiaIsPhysical(tc)
% Triangle inequality on every body, including the corrected link4. A
% physics engine refuses a tensor that fails this.
R = tc.TestData.P.robot;
bodies = [R.link, R.hand, R.finger];
for b = bodies
    e = sort(eig(b.I));
    tc.verifyGreaterThan(e(1), 0);
    tc.verifyGreaterThanOrEqual(e(1) + e(2) - e(3), 0);
end
end
