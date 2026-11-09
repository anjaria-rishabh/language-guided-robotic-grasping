function files = plotTrials(matFile, outRoot)
%PLOTTRIALS  The six result figures for a results file (PNG + PDF each).
%
%   files = grasp.analysis.plotTrials('results/trials.mat')
%   files = grasp.analysis.plotTrials('results/quick.mat', 'results/figures')
%
%   Writes into <outRoot>/<results name>/ so different runs never overwrite
%   each other. Each figure answers one question; the question is its title.
%
%     1_success_by_regime  Does the controller complete the grasp?
%     2_peak_force         How hard does it hit?
%     3_terminal_error     How close does it get?
%     4_exec_time          How long does it take?
%     5_outcomes           How does each controller end?
%     6_contact_time       How long is the hand in contact?
%
%   Rules (as in the ROS build's plot_trials.py): one colour per controller
%   in every figure, fixed by name (impedance blue, admittance orange,
%   baseline aqua, from a colour-vision-safe palette); raw points rather
%   than box plots at these sample sizes; one y-axis per figure; targets
%   drawn as labelled reference lines; text in neutral ink.

P = grasp.config.params();
if nargin < 2 || isempty(outRoot), outRoot = fullfile(P.paths.results, 'figures'); end
s = load(matFile, 'results');  R = s.results;
[~, stem] = fileparts(matFile);
outDir = fullfile(outRoot, stem);
if ~isfolder(outDir), mkdir(outDir); end

colours = containers.Map({'impedance', 'admittance', 'baseline'}, ...
    {[0.165 0.471 0.839], [0.922 0.408 0.204], [0.106 0.686 0.478]});
ink = [0.32 0.32 0.31];  grid = [0.89 0.89 0.87];
modes = intersect(["impedance" "admittance" "baseline"], unique(R.mode), 'stable');
regimes = intersect(["clean" "sensor_noise" "calibration" "displacement"], unique(R.regime), 'stable');
labels = containers.Map({'clean', 'sensor_noise', 'calibration', 'displacement'}, ...
    {'clean', 'sensor noise', 'calibration offset', 'object displaced'});
regLab = cellfun(@(r) labels(r), cellstr(regimes), 'UniformOutput', false);
T = P.exp.targets;
files = {};
rng(0);                                  % jitter is seeded: re-plots are identical

% ---- 1. success by regime ----------------------------------------------------
f = newFig('Does the controller complete the grasp?', ...
    'Success rate per disturbance regime; whiskers are 95 % Wilson intervals; counts are successes/trials.');
groups = [regimes, "all"];
w = min(0.8 / numel(modes), 0.22);
hold on
for mi = 1:numel(modes)
    for gi = 1:numel(groups)
        if groups(gi) == "all", r = R(R.mode == modes(mi), :);
        else, r = R(R.mode == modes(mi) & R.regime == groups(gi), :); end
        if height(r) == 0, continue, end
        k = sum(r.success);  n = height(r);  [lo, hi] = wilson(k, n);
        x = gi + (mi - (numel(modes) + 1) / 2) * (w + 0.02);
        bar(x, 100 * k / n, w, 'FaceColor', colours(char(modes(mi))), 'EdgeColor', 'none');
        plot([x x], 100 * [lo hi], '-', 'Color', ink, 'LineWidth', 1);
        text(x, 100 * hi + 2, sprintf('%d/%d', k, n), 'Rotation', 90, 'FontSize', 7, 'Color', ink);
    end
end
refLine(100 * T.success_rate_overall, 'target 85 %');
set(gca, 'XTick', 1:numel(groups), 'XTickLabel', [regLab, {'all regimes'}], 'YLim', [0 118], 'YTick', 0:20:100);
ylabel('grasp success (%)');
legendFor(modes, 's');
files = save2(f, outDir, '1_success_by_regime', files);

% ---- 2-4, 6. one dot per trial ------------------------------------------------
strips = {'peak_force_n', '2_peak_force', 'How hard does it hit?', ...
              'Peak contact force over the whole trial; one dot per trial, bar = median.', ...
              'peak contact force (N)', {T.peak_force_n, 'target 10 N'; P.ctrl.hard_force_limit, 'interlock 15 N'};
          'targeting_error_mm', '3_terminal_error', 'How close does it get?', ...
              'TCP to the true grasp point when GRASP began; trials that never got there have no value.', ...
              'terminal targeting error (mm)', {T.targeting_error_mm, 'target 3 mm'};
          'exec_time_s', '4_exec_time', 'How long does it take?', ...
              'Trial start to HOLD, or to the abort; one dot per trial, bar = median.', ...
              'execution time (s)', {T.exec_time_s, 'target 15 s'};
          'contact_duration_s', '6_contact_time', 'How long is the hand in contact?', ...
              'Time with more than 1 N on the hand; one dot per trial, bar = median.', ...
              'time in contact (s)', {}};
for q = 1:size(strips, 1)
    col = strips{q, 1};
    if ~any(isfinite(R.(col))), continue, end
    f = newFig(strips{q, 3}, strips{q, 4});
    hold on
    w = 0.8 / numel(modes);
    for gi = 1:numel(regimes)
        for mi = 1:numel(modes)
            v = R.(col)(R.mode == modes(mi) & R.regime == regimes(gi));
            v = v(isfinite(v));
            if isempty(v), continue, end
            c = gi - 0.4 + w * (mi - 0.5);
            scatter(c + (rand(size(v)) - 0.5) * 0.5 * w, v, 36, colours(char(modes(mi))), 'filled', ...
                'MarkerEdgeColor', 'w', 'LineWidth', 1);
            plot(c + [-0.38 0.38] * w, median(v) * [1 1], '-', 'Color', [0.05 0.05 0.05], 'LineWidth', 2);
        end
    end
    refs = strips{q, 6};
    for k = 1:size(refs, 1), refLine(refs{k, 1}, refs{k, 2}); end
    set(gca, 'XTick', 1:numel(regimes), 'XTickLabel', regLab, 'XLim', [0.4 numel(regimes) + 0.6]);
    ylim([0, max(ylim)]);
    ylabel(strips{q, 5});
    legendFor(modes, 'o');
    files = save2(f, outDir, strips{q, 2}, files);
end

% ---- 5. outcomes ---------------------------------------------------------------
f = newFig('How does each controller end?', ...
    'Trials per final outcome. Timeout = a phase ran out of time.');
outcomes = {'success', 'force interlock', 'timeout', 'other abort'};
counts = zeros(numel(modes), numel(outcomes));
for mi = 1:numel(modes)
    r = R(R.mode == modes(mi), :);
    for i = 1:height(r)
        if r.success(i), o = 1;
        elseif contains(r.fault(i), 'wrench'), o = 2;
        elseif contains(r.fault(i), 'timeout'), o = 3;
        else, o = 4; end
        counts(mi, o) = counts(mi, o) + 1;
    end
end
imagesc(counts);
colormap(interp1([0 1], [0.96 0.97 0.99; 0.06 0.26 0.51], linspace(0, 1, 64)));
for i = 1:numel(modes)
    for j = 1:numel(outcomes)
        dark = counts(i, j) > 0.55 * max(counts(:));
        text(j, i, sprintf('%d', counts(i, j)), 'HorizontalAlignment', 'center', ...
            'FontWeight', 'bold', 'Color', dark * [1 1 1] + ~dark * [0.05 0.05 0.05]);
    end
end
set(gca, 'XTick', 1:numel(outcomes), 'XTickLabel', outcomes, 'YTick', 1:numel(modes), 'YTickLabel', cellstr(modes));
files = save2(f, outDir, '5_outcomes', files);

% ---------------------------------------------------------------------------------
    function f = newFig(titleText, sub)
        f = figure('Visible', 'off', 'Color', [0.988 0.988 0.984], 'Position', [100 100 800 440]);
        ax = axes(f, 'Color', 'none', 'Box', 'off', 'TickDir', 'out', 'XColor', ink, 'YColor', ink, ...
            'GridColor', grid, 'GridAlpha', 1, 'YGrid', 'on', 'FontSize', 10);
        title(ax, {titleText, ['\fontsize{9}\color[rgb]{0.32,0.32,0.31}' sub]}, ...
            'HorizontalAlignment', 'left', 'Units', 'normalized', 'Position', [0 1.02]);
    end
    function refLine(y, label)
        yline(y, '-', label, 'Color', ink, 'LineWidth', 1, 'LabelHorizontalAlignment', 'right', ...
            'FontSize', 8, 'LabelVerticalAlignment', 'bottom');
    end
    function legendFor(ms, marker)
        h = gobjects(numel(ms), 1);
        for k = 1:numel(ms)
            h(k) = plot(nan, nan, marker, 'MarkerFaceColor', colours(char(ms(k))), ...
                'MarkerEdgeColor', 'none', 'MarkerSize', 8, 'DisplayName', ms(k));
        end
        legend(h, 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off', 'TextColor', ink);
    end
end

function files = save2(f, outDir, name, files)
exportgraphics(f, fullfile(outDir, [name '.png']), 'Resolution', 200);
exportgraphics(f, fullfile(outDir, [name '.pdf']), 'ContentType', 'vector');
close(f);
files(end + (1:2)) = {fullfile(outDir, [name '.png']), fullfile(outDir, [name '.pdf'])};
end

function [lo, hi] = wilson(k, n)
z = 1.96;  p = k / n;  den = 1 + z^2 / n;
c = (p + z^2 / (2 * n)) / den;  h = z * sqrt(p * (1 - p) / n + z^2 / (4 * n^2)) / den;
lo = max(0, c - h);  hi = min(1, c + h);
end
