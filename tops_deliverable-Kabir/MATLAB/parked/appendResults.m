function appendResults(rows, matFile)
%APPENDRESULTS  Add trial rows to the results table (.mat) and its CSV copy.
%
%   grasp.experiment.appendResults(rows, 'results/trials.mat')
%
%   rows     struct array from grasp.experiment.metrics (or a table)
%   matFile  the primary record: a .mat file holding one table, `results`
%
%   Results are APPENDED, never overwritten: clear stale rows yourself
%   (`tops clean results`) before a run whose numbers must stand alone.
%   The CSV beside it (same name, .csv) is rewritten from the table every
%   time, with the 19 columns in the ROS order, so the two can never
%   disagree and the CSV can be compared with the ROS build's results.

if isstruct(rows), rows = struct2table(rows, 'AsArray', true); end
folder = fileparts(matFile);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end

if isfile(matFile)
    s = load(matFile, 'results');
    results = [s.results; rows];
else
    results = rows;
end
save(matFile, 'results');

columns = {'trial_id','mode','regime','seed','object_label','success', ...
    'peak_force_n','mean_contact_force_n','exec_time_s','targeting_error_mm', ...
    'final_state','fault','force_limit_tripped','watchdog_tripped', ...
    'tank_min_energy_j','passivity_throttled','max_update_period_ms', ...
    'missed_deadlines','contact_duration_s'};
[p, n] = fileparts(matFile);
writetable(results(:, columns), fullfile(p, [n '.csv']));
end
