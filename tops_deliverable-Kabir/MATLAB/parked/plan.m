function T = plan(P, perCell, modes, regimes)
%PLAN  The full-factorial trial plan, as a table.
%
%   T = grasp.experiment.plan(P)                       % 3 x 4 x P.exp.per_cell
%   T = grasp.experiment.plan(P, perCell, modes, regimes)
%
%   Columns: trial_id, mode, regime, seed, object (index), object_label.
%
%   The seed depends ONLY on the regime and the repeat number, never on the
%   controller:  seed = 1000 * (regime index - 1) + repeat.  So trial k of a
%   regime gives every controller the identical disturbance, and the
%   analysis can pair them (Theory 10). Objects cycle through the six blocks
%   so the result is not about one block. Same scheme as the ROS runner.

if nargin < 2 || isempty(perCell), perCell = P.exp.per_cell; end
if nargin < 3 || isempty(modes), modes = P.exp.modes; end
if nargin < 4 || isempty(regimes), regimes = P.exp.regimes; end

n = perCell * numel(regimes) * numel(modes);
trial_id = zeros(n, 1);  seed = zeros(n, 1);  object = zeros(n, 1);
mode = strings(n, 1);  regime = strings(n, 1);  object_label = strings(n, 1);
k = 0;
nObj = numel(P.world.objects);
for rep = 0:perCell - 1
    for ri = 1:numel(regimes)
        for mi = 1:numel(modes)
            k = k + 1;
            trial_id(k) = k - 1;
            mode(k) = modes{mi};
            regime(k) = regimes{ri};
            seed(k) = 1000 * (ri - 1) + rep;
            object(k) = mod(rep, nObj) + 1;
            object_label(k) = P.world.objects(object(k)).label;
        end
    end
end
T = table(trial_id, mode, regime, seed, object, object_label);
end
