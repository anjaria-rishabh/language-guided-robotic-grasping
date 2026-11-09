function result = measureGrasp(P, colour, pushArm, log)
%MEASUREGRASP  One run's log -> its summary numbers.
%
%   result = grasp.sim.measureGrasp(P, colour, pushArm, log)
%
%   success            reached HOLD
%   peak_force_n       largest external force at the wrist over the run
%   mean_contact_n     mean force over samples above the contact threshold
%   time_s             start of the run to HOLD or STOP
%   position_error_mm  TCP to the true grasp point when the fingers started closing
%   box_lifted_mm      how far the box rose from where it started
%   handover_torque_step_nm  largest joint torque change in one cycle, within
%                      half a second of the switch from admittance to impedance
%                      (1 N.m is the rate limit)

names = grasp.supervisor.phaseNames();
sv = log.supervisor;  st = log.controller;  t = log.t;
force = st(:, 4);
contact = force > P.sim.contact_threshold;

tStart = sv(find(sv(:, 9) > 0.5, 1), 5);
if isempty(tStart), tStart = t(1); end
tEnd = sv(end, 6);
if tEnd < 0, tEnd = t(end); end

result = struct();
result.box = string(colour);
result.pushed = logical(pushArm);
result.success = sv(end, 1) == 6;
result.final_phase = string(names{sv(end, 1) + 1});
result.faults = string(grasp.supervisor.faultNames(sv(end, 3)));
result.stop_reason = stopReasonName(max(st(:, 9)));
result.time_s = tEnd - tStart;
result.peak_force_n = max([force; 0]);
result.mean_contact_n = 0;
if any(contact), result.mean_contact_n = mean(force(contact)); end
result.position_error_mm = 1000 * sv(end, 8);
result.box_lifted_mm = 1000 * (log.box_xyz_world(end, 3) - log.box_xyz_world(1, 3));
result.warned = any(st(:, 8) == 1);

switched = find(log.phase == 3, 1);
result.handover_torque_step_nm = NaN;
if ~isempty(switched)
    near = abs(t - t(switched)) <= 0.5;
    result.handover_torque_step_nm = max(st(near, 20));
end
end

function name = stopReasonName(code)
names = ["none", "force limit", "torque limit", "sudden hit", "bad sensor"];
name = names(code + 1);
end
