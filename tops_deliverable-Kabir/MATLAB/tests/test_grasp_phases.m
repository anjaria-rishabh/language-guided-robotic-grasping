function tests = test_grasp_phases
%TEST_GRASP_PHASES  The grasp sequence, transition by transition, no simulator.
tests = functiontests(localfunctions);
end

function setup(tc)
P = grasp.config.settings();
[~, U] = grasp.config.controllerSettings(P);
tc.TestData.U = U;
end

% ---- helpers ---------------------------------------------------------------
function F = phase(s)
F = struct('state', s, 'timeInState', 0, 'justEntered', false);
end

function o = obs(varargin)
% A calm observation: confident target, far away, no force, nothing happening.
o = struct('dt', 0.02, 'distance_to_target', 0.2, 'force_at_wrist', 0, 'safety_stop', false, ...
    'target_confidence', 0.95, 'seconds_since_target', 0.01, 'gains_settled', true, ...
    'gripper_closed', false, 'object_grasped', false, 'tcp_height_above_start', 0, ...
    'at_home', false, 'start_requested', false);
for k = 1:2:numel(varargin), o.(varargin{k}) = varargin{k + 1}; end
end

% ---- the normal sequence -------------------------------------------------------
function testWaitStartsOnRequest(tc)
F = grasp.supervisor.decideGraspPhase(phase(0), obs('start_requested', true), tc.TestData.U);
tc.verifyEqual(F.state, 1);
end

function testGoHomeThenMove(tc)
F = grasp.supervisor.decideGraspPhase(phase(1), obs('at_home', true), tc.TestData.U);
tc.verifyEqual(F.state, 2);
end

function testMoveEndsWhenNearTheBox(tc)
F = grasp.supervisor.decideGraspPhase(phase(2), obs('distance_to_target', 0.010), tc.TestData.U);
tc.verifyEqual(F.state, 3);
end

function testTouchOnlyCountsNearTheBox(tc)
U = tc.TestData.U;
F = grasp.supervisor.decideGraspPhase(phase(2), obs('force_at_wrist', 2.0, 'distance_to_target', 0.05), U);
tc.verifyEqual(F.state, 3);
F = grasp.supervisor.decideGraspPhase(phase(2), obs('force_at_wrist', 2.0, 'distance_to_target', 0.40), U);
tc.verifyEqual(F.state, 2);
end

function testFingersCloseAfterSettling(tc)
F = phase(3);  F.timeInState = 8;
F = grasp.supervisor.decideGraspPhase(F, obs('distance_to_target', 0.012), tc.TestData.U);
tc.verifyEqual(F.state, 4);
end

function testFingersWaitForStiffnessToSettle(tc)
F = phase(3);  F.timeInState = 8;
F = grasp.supervisor.decideGraspPhase(F, obs('distance_to_target', 0.001, 'gains_settled', false), tc.TestData.U);
tc.verifyEqual(F.state, 3);
end

function testHoldOnlyIfTheBoxCameUp(tc)
U = tc.TestData.U;
F = grasp.supervisor.decideGraspPhase(phase(5), obs('tcp_height_above_start', 0.06, 'object_grasped', true), U);
tc.verifyEqual(F.state, 6);
[F, faults] = grasp.supervisor.decideGraspPhase(phase(5), obs('tcp_height_above_start', 0.06, 'object_grasped', false), U);
tc.verifyEqual(F.state, 7);
tc.verifyTrue(bitand(faults, 128) > 0, 'box_dropped fault');
end

% ---- safety --------------------------------------------------------------------
function testSafetyStopRetreatsFromEveryActivePhase(tc)
for s = 1:6
    [F, faults] = grasp.supervisor.decideGraspPhase(phase(s), obs('safety_stop', true), tc.TestData.U);
    tc.verifyEqual(F.state, 7, sprintf('phase %d must retreat', s));
    tc.verifyTrue(bitand(faults, 1) > 0);
end
end

function testTargetTimeoutOnlyWhileChasingTheBox(tc)
U = tc.TestData.U;
F = grasp.supervisor.decideGraspPhase(phase(2), obs('seconds_since_target', 0.5), U);
tc.verifyEqual(F.state, 7);
F = grasp.supervisor.decideGraspPhase(phase(5), obs('seconds_since_target', 0.5), U);
tc.verifyEqual(F.state, 5);
end

function testLowConfidenceWaitsThenStops(tc)
U = tc.TestData.U;
[F, faults] = grasp.supervisor.decideGraspPhase(phase(2), obs('target_confidence', 0.2, 'distance_to_target', 0.001), U);
tc.verifyEqual(F.state, 2);
tc.verifyTrue(bitand(faults, 256) > 0);
F.timeInState = U.move_timeout + 1;
F = grasp.supervisor.decideGraspPhase(F, obs('target_confidence', 0.2), U);
tc.verifyEqual(F.state, 8);
end

function testRetreatEndsInStopNeverRetry(tc)
F = phase(7);  F.timeInState = 2;
F = grasp.supervisor.decideGraspPhase(F, obs(), tc.TestData.U);
tc.verifyEqual(F.state, 8);
end

function testEveryActivePhaseTimesOut(tc)
U = tc.TestData.U;
limits = [1 U.home_timeout; 2 U.move_timeout; 3 U.descend_timeout; 4 U.close_timeout; 5 U.lift_timeout];
for k = 1:size(limits, 1)
    F = phase(limits(k, 1));  F.timeInState = limits(k, 2) + 1;
    F = grasp.supervisor.decideGraspPhase(F, obs('gains_settled', false), U);
    tc.verifyNotEqual(F.state, limits(k, 1), sprintf('phase %d never times out', limits(k, 1)));
end
end

function testEnteredIsTrueExactlyOnce(tc)
U = tc.TestData.U;
[F, ~, e1] = grasp.supervisor.decideGraspPhase(phase(1), obs('at_home', true), U);
[~, ~, e2] = grasp.supervisor.decideGraspPhase(F, obs(), U);
tc.verifyTrue(e1);
tc.verifyFalse(e2);
end
