function rows = runPlan(P, T, matFile, verbose)
%RUNPLAN  Run every trial in a plan, one after another, and record them.
%
%   rows = grasp.experiment.runPlan(P, T, matFile)
%   rows = grasp.experiment.runPlan(P, T, matFile, true)   % print each timeline
%
%   T        a plan table (grasp.experiment.plan)
%   matFile  where results are APPENDED (grasp.experiment.appendResults);
%            '' to record nothing
%
%   Each trial is appended as soon as it finishes, so an interrupted run
%   keeps everything it completed. A trial that errors is reported and
%   skipped, never silently dropped.

if nargin < 4, verbose = false; end
mdl = 'tops_sim';
if ~bdIsLoaded(mdl)
    file = fullfile(P.paths.model, [mdl '.slx']);
    if ~isfile(file), grasp.sim.buildModel(P); end
    load_system(file);
end

rows = [];
for k = 1:height(T)
    trial = T(k, :);
    fprintf('[%d/%d] trial %d: %s / %s / seed %d / %s ... ', k, height(T), ...
        trial.trial_id, trial.mode, trial.regime, trial.seed, trial.object_label);
    t0 = tic;
    try
        [row, L] = grasp.experiment.runTrial(P, trial, mdl);
    catch e
        r = e;  while ~isempty(r.cause), r = r.cause{1}; end
        fprintf('FAILED TO RUN: %s\n', r.message);
        continue
    end
    fprintf('%s, peak %.2f N, %.1f s (%.0f s wall)\n', row.final_state, ...
        row.peak_force_n, row.exec_time_s, toc(t0));
    if verbose
        grasp.experiment.describe(L, row);
    end
    if ~isempty(matFile)
        grasp.experiment.appendResults(row, matFile);
    end
    rows = [rows; row]; %#ok<AGROW>  (a dozen to a few hundred rows)
end
end
