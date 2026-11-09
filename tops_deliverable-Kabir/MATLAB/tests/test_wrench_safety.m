function tests = test_wrench_safety
%TEST_WRENCH_SAFETY  Each safety check fires when it should and only then.
tests = functiontests(localfunctions);
end

function setup(tc)
tc.TestData.C = grasp.config.controllerSettings(grasp.config.settings());
end

function [level, reason] = check(tc, force, torque, spike, bad, moving)
w = [force; 0; 0; torque; 0; 0];
[level, reason] = grasp.control.checkWrenchSafety(w, spike, bad, moving, tc.TestData.C);
end

function testNothingHappeningIsFine(tc)
[level, reason] = check(tc, 0, 0, 0, false, true);
tc.verifyEqual([level reason], [0 0]);
end

function testForceWarnsThenStopsWhileMoving(tc)
L = tc.TestData.C.safety;
tc.verifyEqual(check(tc, L.force_warn_moving + 0.1, 0, 0, false, true), 1);
[level, reason] = check(tc, L.force_stop_moving + 0.1, 0, 0, false, true);
tc.verifyEqual([level reason], [2 1]);
end

function testContactAllowsMoreForceThanTravel(tc)
L = tc.TestData.C.safety;
f = L.force_stop_moving + 0.1;
tc.verifyEqual(check(tc, f, 0, 0, false, true), 2);
tc.verifyEqual(check(tc, f, 0, 0, false, false), 0);
[level, reason] = check(tc, L.force_stop_contact + 0.1, 0, 0, false, false);
tc.verifyEqual([level reason], [2 1]);
end

function testTorqueWarnsThenStops(tc)
L = tc.TestData.C.safety;
[level, reason] = check(tc, 0, L.torque_warn + 0.1, 0, false, false);
tc.verifyEqual([level reason], [1 2]);
[level, reason] = check(tc, 0, L.torque_stop + 0.1, 0, false, false);
tc.verifyEqual([level reason], [2 2]);
end

function testSuddenHitOnlyCountsWhileMoving(tc)
L = tc.TestData.C.safety;
[level, reason] = check(tc, 0, 0, L.spike_limit + 0.1, false, true);
tc.verifyEqual([level reason], [2 3]);
tc.verifyEqual(check(tc, 0, 0, L.spike_limit + 0.1, false, false), 0);
end

function testBadSensorStops(tc)
[level, reason] = check(tc, 0, 0, 0, true, false);
tc.verifyEqual([level reason], [2 4]);
end

function testNaNReadingIsReportedAndDoesNotPoisonTheFilters(tc)
C = tc.TestData.C;
S = zeroedSensor(C);
[S, ~, wSafety, ~, bad] = grasp.control.readForceSensor(S, nan(6, 1), eye(3), zeros(6, 1), C);
tc.verifyTrue(bad);
tc.verifyTrue(all(isfinite(wSafety)));
tc.verifyTrue(all(isfinite(S.wControl)));
end

function testFrozenSensorIsReported(tc)
C = tc.TestData.C;
S = zeroedSensor(C);
w = (1:6).';
bad = false;
for k = 1:C.safety.frozen_samples + 1
    [S, ~, ~, ~, bad] = grasp.control.readForceSensor(S, w, eye(3), zeros(6, 1), C);
end
tc.verifyTrue(bad);
end

function S = zeroedSensor(C)
S = grasp.control.startingControlState(C);
S.zeroed = true;
S.zeroCount = C.zeroing_samples;
end
