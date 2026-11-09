function tests = test_control_modes
%TEST_CONTROL_MODES  Which control runs when, and what admittance does.
tests = functiontests(localfunctions);
end

function setup(tc)
tc.TestData.C = grasp.config.controllerSettings(grasp.config.settings());
end

function testAdmittanceWhileTravelling(tc)
for phase = 0:2
    tc.verifyEqual(grasp.control.chooseControlMode(phase), 1);
end
end

function testDescentRegulatesForce(tc)
tc.verifyEqual(grasp.control.chooseControlMode(3), 2);
end

function testImpedanceFromGripOnwards(tc)
for phase = 4:8
    tc.verifyEqual(grasp.control.chooseControlMode(phase), 3);
end
end

function testGrippingIsSofterThanMoving(tc)
C = tc.TestData.C;
tc.verifyTrue(all(C.K_gripping < C.K_moving));
end

function testPushMovesTheReferenceTheWayOfThePush(tc)
C = tc.TestData.C;
S = grasp.control.startingControlState(C);
push = [10; 0; 0; 0; 0; 0];
for k = 1:1000
    S = grasp.control.updateVirtualOffset(S, push, C);
end
tc.verifyGreaterThan(S.xa(1), 0.02);
tc.verifyLessThan(S.xa(1), C.adm_max_offset + 1e-9);
end

function testOffsetFadesWhenThePushIsGone(tc)
C = tc.TestData.C;
S = grasp.control.startingControlState(C);
S.xa(1) = 0.05;
for k = 1:5000
    S = grasp.control.updateVirtualOffset(S, zeros(6, 1), C);
end
tc.verifyLessThan(abs(S.xa(1)), 1e-3);
end

function testOffsetNeverPassesItsLimit(tc)
C = tc.TestData.C;
S = grasp.control.startingControlState(C);
for k = 1:5000
    S = grasp.control.updateVirtualOffset(S, [500; 0; 0; 0; 0; 0], C);
end
tc.verifyLessThanOrEqual(norm(S.xa(1:3)), C.adm_max_offset + 1e-9);
end
