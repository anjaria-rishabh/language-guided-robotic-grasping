function tests = test_contact_force
%TEST_CONTACT_FORCE  The force loop along the approach axis.
tests = functiontests(localfunctions);
end

function setup(tc)
C = grasp.config.controllerSettings(grasp.config.settings());
tc.TestData.C = C;
tc.TestData.S = grasp.control.startingControlState(C);
tc.TestData.approach = [0; 0; -1];                    % gripper pointing down
end

function testOffWithoutContact(tc)
d = tc.TestData;
[f, K] = grasp.control.regulateContactForce(d.S, zeros(6, 1), d.approach, true, d.C);
tc.verifyEqual(f, zeros(6, 1));
tc.verifyEqual(K, d.S.K);
end

function testSidewaysForceDoesNotCountAsPressing(tc)
d = tc.TestData;
f = grasp.control.regulateContactForce(d.S, [5; 0; 0; 0; 0; 0], d.approach, true, d.C);
tc.verifyEqual(f, zeros(6, 1));
end

function testTooLittleForcePressesHarder(tc)
d = tc.TestData;
w = [0; 0; 2; 0; 0; 0];                               % the box pushes up with 2 N
f = grasp.control.regulateContactForce(d.S, w, d.approach, true, d.C);
tc.verifyLessThan(f(3), 0, 'target is 5 N: must push further down');
end

function testTooMuchForceBacksOff(tc)
d = tc.TestData;
w = [0; 0; 8; 0; 0; 0];
f = grasp.control.regulateContactForce(d.S, w, d.approach, true, d.C);
tc.verifyGreaterThan(f(3), 0);
end

function testIntegralStaysLimited(tc)
d = tc.TestData;
S = d.S;
w = [0; 0; 1.5; 0; 0; 0];
for k = 1:20000
    [~, ~, S] = grasp.control.regulateContactForce(S, w, d.approach, true, d.C);
end
tc.verifyLessThanOrEqual(abs(S.forceI), d.C.force_integral_limit + 1e-9);
end

function testApproachAxisIsSoftened(tc)
d = tc.TestData;
[~, K] = grasp.control.regulateContactForce(d.S, [0; 0; 2; 0; 0; 0], d.approach, true, d.C);
tc.verifyEqual(K(3), d.S.K(3) * d.C.force_axis_stiffness_scale, 'AbsTol', 1e-9);
tc.verifyEqual(K(1), d.S.K(1));
end
