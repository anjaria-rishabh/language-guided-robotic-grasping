function printGraspSummary(log, result)
%PRINTGRASPSUMMARY  Print what happened in one run, as a timeline.
%
%   grasp.sim.printGraspSummary(log, result)
%
%   One line per phase change, then any safety events, then the results.

names = grasp.supervisor.phaseNames();
modes = {'admittance', 'impedance', 'impedance'};
t = log.t;  sv = log.supervisor;  st = log.controller;  box = log.box_xyz_world;
phase = log.phase;

heading = sprintf('%s box', result.box);
if result.pushed, heading = [heading ', arm pushed mid-move']; end
fprintf('\n%s\n', heading);
fprintf('  %7s  %-15s  %-11s  %-24s  %9s  %7s  %8s  %7s\n', ...
    't (s)', 'phase', 'control', 'TCP (m)', 'to goal', 'F (N)', 'box z', 'jaw mm');
changes = [1; find(diff(phase) ~= 0) + 1; numel(t)];
for k = changes.'
    mode = modes{max(1, st(k, 19))};
    fprintf('  %7.2f  %-15s  %-11s  %-24s  %7.1fmm  %7.2f  %8.4f  %7.1f\n', t(k), names{phase(k) + 1}, mode, ...
        mat2str(st(k, 1:3), 3), 1000 * sv(k, 11), st(k, 4), box(k, 3), 1000 * sv(k, 13));
end

warned = find(st(:, 8) == 1, 1);
stopped = find(st(:, 8) == 2, 1);
if ~isempty(warned)
    fprintf('\n  safety: warned at t = %.2f s, arm slowed\n', t(warned));
end
if ~isempty(stopped)
    fprintf('  safety: stopped at t = %.2f s (%s), force %.1f N\n', t(stopped), result.stop_reason, st(stopped, 5));
end

fprintf('\n  success            %d (%s)\n', result.success, result.final_phase);
if strlength(result.faults) > 0
    fprintf('  faults             %s\n', result.faults);
end
fprintf('  time               %.2f s\n', result.time_s);
fprintf('  peak force         %.2f N\n', result.peak_force_n);
fprintf('  mean contact force %.2f N\n', result.mean_contact_n);
fprintf('  position error     %.1f mm\n', result.position_error_mm);
fprintf('  box lifted         %.1f mm\n', result.box_lifted_mm);
fprintf('  handover jerk      %.2f N.m per cycle (limit 1)\n\n', result.handover_torque_step_nm);
end
