function text = report(matFile, outFile)
%REPORT  The statistics report for a results file.
%
%   grasp.analysis.report('results/trials.mat')                     % print it
%   grasp.analysis.report('results/trials.mat', 'results/report.md') % and save it
%
%   A port of the ROS analyze_trials.py, with the Statistics and Machine
%   Learning Toolbox doing the tests (exact p-values; the ROS build fell
%   back to normal approximations without scipy).
%
%   What it does that a naive script would not:
%     * success is a PROPORTION: Fisher's exact test and Wilson intervals,
%       never a t-test;
%     * every continuous metric gets two tests (Welch's t and Mann-Whitney
%       U) and the report says whether they agree; if they disagree,
%       outliers or skew are driving the t-test and the data need looking at;
%     * effect sizes (Cohen's d), not only p-values;
%     * paired tests (paired t, Wilcoxon signed-rank) on trials that share
%       a seed, which the experiment is designed to produce;
%     * everything broken down by disturbance regime.

P = grasp.config.params();
s = load(matFile, 'results');
R = s.results;
targets = P.exp.targets;
lines = strings(0, 1);
say = @(varargin) sprintf(varargin{:});

add(say('TOPS 2026 -- trial analysis of %s', matFile));
add(say('MATLAB %s, Statistics and Machine Learning Toolbox (exact tests)', version('-release')));

% ---------------------------------------------------------------------------
heading('1.  OVERVIEW');
modes = intersect(["impedance" "admittance" "baseline"], unique(R.mode), 'stable');
modes = [modes, setdiff(unique(R.mode).', modes)];
regimes = intersect(["clean" "sensor_noise" "calibration" "displacement"], unique(R.regime), 'stable');
add(say('%d trials, %d controllers, %d regimes', height(R), numel(modes), numel(regimes)));
add('');
add(say('%-12s %4s %8s %16s %14s %14s %9s', 'mode', 'n', 'success', '95% CI', 'peak F (N)', 'error (mm)', 'time (s)'));
add(repmat('-', 1, 84));
for m = modes
    r = R(R.mode == m, :);
    k = sum(r.success);  n = height(r);
    [lo, hi] = wilson(k, n);
    add(say('%-12s %4d %7.1f%% %16s %6.2f+-%-6.2f %6.2f+-%-6.2f %9.2f', m, n, 100 * k / n, ...
        say('[%.2f, %.2f]', lo, hi), mean(r.peak_force_n), std(r.peak_force_n), ...
        meanFinite(r.targeting_error_mm), stdFinite(r.targeting_error_mm), mean(r.exec_time_s)));
end
add('');
add(say('Targets: success >= %.0f %% overall and >= %.0f %% clean; peak force <= %.1f N;', ...
    100 * targets.success_rate_overall, 100 * targets.success_rate_clean, targets.peak_force_n));
add(say('terminal error <= %.1f mm; execution time <= %.1f s.', targets.targeting_error_mm, targets.exec_time_s));

% ---------------------------------------------------------------------------
heading('2.  AGAINST THE PLAN''S TARGETS');
add(say('%-12s %-30s %10s %10s  %s', 'mode', 'metric', 'measured', 'target', 'verdict'));
add(repmat('-', 1, 78));
for m = modes
    r = R(R.mode == m, :);
    verdict(m, 'grasp success rate', mean(r.success), targets.success_rate_overall, 'higher', '%.1f%%', 100);
    c = r(r.regime == "clean", :);
    if height(c) > 0
        verdict('', 'grasp success (clean only)', mean(c.success), targets.success_rate_clean, 'higher', '%.1f%%', 100);
    end
    verdict('', 'peak contact force (worst)', max(r.peak_force_n), targets.peak_force_n, 'lower', '%.2f N', 1);
    e = r.targeting_error_mm(isfinite(r.targeting_error_mm));
    if ~isempty(e)
        verdict('', 'terminal targeting error (worst)', max(e), targets.targeting_error_mm, 'lower', '%.2f mm', 1);
    end
    verdict('', 'trial execution time (worst)', max(r.exec_time_s), targets.exec_time_s, 'lower', '%.2f s', 1);
    add(repmat('-', 1, 78));
end

% ---------------------------------------------------------------------------
if any(modes == "baseline")
    for m = modes(modes ~= "baseline")
        compare("baseline", m);
    end
end

% ---------------------------------------------------------------------------
heading('4.  BY DISTURBANCE REGIME');
add('The headline numbers average over four very different situations. The');
add('result worth reporting is how the gap CHANGES across them: the calibration');
add('regime -- a fixed offset that does not average out -- is where compliance');
add('is supposed to earn its place.');
tables = {'success rate', @(r) say('%.1f%%', 100 * mean(r.success));
          'mean peak force (N)', @(r) say('%.2f', mean(r.peak_force_n));
          'mean terminal error (mm)', @(r) fmtFinite(r.targeting_error_mm)};
for q = 1:size(tables, 1)
    add('');
    add(tables{q, 1});
    head = say('%-16s', 'regime');
    for m = modes, head = [head say('%14s', m)]; end %#ok<AGROW>
    add(head);
    add(repmat('-', 1, strlength(head)));
    for g = regimes
        row = say('%-16s', g);
        for m = modes
            r = R(R.mode == m & R.regime == g, :);
            if height(r) == 0, cell = '--'; else, cell = tables{q, 2}(r); end
            row = [row say('%14s', cell)]; %#ok<AGROW>
        end
        add(row);
    end
end

% ---------------------------------------------------------------------------
heading('5.  FAULTS');
add('How each trial that failed, failed.');
for m = modes
    r = R(R.mode == m, :);
    f = r.fault(r.fault ~= "");
    names = split(strjoin(f, ';'), ';');
    names = names(names ~= "");
    if isempty(names)
        add(say('%-12s none', m));
    else
        [u, ~, j] = unique(names);
        counts = accumarray(j, 1);
        add(say('%-12s %s', m, strjoin(compose('%s x%d', u, counts), ', ')));
    end
end
add('');
add('Loop timing: the controller runs at exactly 1 kHz in simulated time in');
add('this build, so the ROS build''s missed-deadline caveat does not apply.');

text = strjoin(lines, newline);
if nargin >= 2 && ~isempty(outFile)
    fid = fopen(outFile, 'w');
    fprintf(fid, '```\n%s\n```\n', text);
    fclose(fid);
end
if nargout == 0
    fprintf('%s\n', text);
    clear text
end

% =========================================================================
    function add(t)
        lines(end + 1) = string(t);
    end
    function heading(t)
        add('');  add(repmat('=', 1, 78));  add(['  ' t]);  add(repmat('=', 1, 78));
    end
    function verdict(m, name, value, target, better, fmt, scale)
        ok = (strcmp(better, 'higher') && value >= target) || (strcmp(better, 'lower') && value <= target);
        v = {'MISS', 'PASS'};
        add(say(['%-12s %-30s %10s %10s  %s'], m, name, say(fmt, scale * value), ...
            say(fmt, scale * target), v{ok + 1}));
    end
    function compare(base, treat)
        heading(sprintf('3.  %s  vs  %s', upper(treat), upper(base)));
        a = R(R.mode == treat, :);  b = R(R.mode == base, :);
        ka = sum(a.success);  kb = sum(b.success);
        add(say('Grasp success:  %s %d/%d (%.1f%%)   %s %d/%d (%.1f%%)', treat, ka, height(a), ...
            100 * ka / height(a), base, kb, height(b), 100 * kb / height(b)));
        if (ka + kb == 0) || (ka + kb == height(a) + height(b))
            add('  no variation in outcome: no test possible (p = 1)');
        else
            [~, p] = fishertest([ka, height(a) - ka; kb, height(b) - kb]);
            add(say('  Fisher''s exact test: p = %s  %s', fmtP(p), stars(p)));
        end
        add('  (success is a proportion, so this is deliberately NOT a t-test)');
        % pairs: same regime and seed
        [~, ia, ib] = intersect(a.regime + "/" + a.seed, b.regime + "/" + b.seed);
        add(say('%d matched (regime, seed) pairs: paired tests are used where possible.', numel(ia)));
        metrics = {'peak_force_n', 'Peak contact force', 'N'; 'exec_time_s', 'Trial execution time', 's'; ...
                   'mean_contact_force_n', 'Mean force while in contact', 'N'};
        for q = 1:size(metrics, 1)
            col = metrics{q, 1};
            x = a.(col);  y = b.(col);
            add('');
            add(say('--- %s (%s) ---', metrics{q, 2}, metrics{q, 3}));
            add(say('  %-12s mean %8.3f  sd %7.3f  median %8.3f  max %8.3f  n=%d', treat, mean(x), std(x), median(x), max(x), numel(x)));
            add(say('  %-12s mean %8.3f  sd %7.3f  median %8.3f  max %8.3f  n=%d', base, mean(y), std(y), median(y), max(y), numel(y)));
            if numel(x) < 2 || numel(y) < 2 || (std(x) == 0 && std(y) == 0)
                add('  too few trials or no variation: no test');
                continue
            end
            [~, pt] = ttest2(x, y, 'Vartype', 'unequal');
            pu = ranksum(x, y);
            add(say('  Welch''s t-test         p = %s  %s', fmtP(pt), stars(pt)));
            add(say('  Mann-Whitney U         p = %s  %s', fmtP(pu), stars(pu)));
            if (pt < 0.05) == (pu < 0.05)
                add('  the two tests AGREE: the result is robust to the distributional assumption');
            else
                add('  the two tests DISAGREE: outliers or skew drive the t-test -- look at the data');
            end
            sp = sqrt((var(x) + var(y)) / 2);
            d = (mean(x) - mean(y)) / max(sp, eps);
            add(say('  Cohen''s d = %+.3f (%s)', d, effectLabel(d)));
            if numel(ia) >= 2
                xp = a.(col)(ia);  yp = b.(col)(ib);
                if any(xp ~= yp)
                    [~, ppt] = ttest(xp, yp);
                    pw = signrank(xp, yp);
                    add(say('  paired t-test          p = %s  %s   (n = %d pairs)', fmtP(ppt), stars(ppt), numel(ia)));
                    add(say('  Wilcoxon signed-rank   p = %s  %s', fmtP(pw), stars(pw)));
                end
            end
        end
    end
end

% =========================================================================
function [lo, hi] = wilson(k, n, z)
% Wilson score interval: honest near 0 % and 100 %, where the textbook
% interval runs past the ends of [0, 1].
if nargin < 3, z = 1.96; end
if n == 0, lo = 0; hi = 1; return, end
p = k / n;
den = 1 + z^2 / n;
c = (p + z^2 / (2 * n)) / den;
h = z * sqrt(p * (1 - p) / n + z^2 / (4 * n^2)) / den;
lo = max(0, c - h);  hi = min(1, c + h);
end

function m = meanFinite(x), x = x(isfinite(x)); if isempty(x), m = NaN; else, m = mean(x); end, end
function s = stdFinite(x),  x = x(isfinite(x)); if numel(x) < 2, s = NaN; else, s = std(x); end, end
function t = fmtFinite(x),  x = x(isfinite(x)); if isempty(x), t = '--'; else, t = sprintf('%.2f', mean(x)); end, end
function t = fmtP(p), if p < 1e-4, t = '<0.0001'; else, t = sprintf('%.4f', p); end, end
function t = stars(p)
if p < 0.001, t = '***'; elseif p < 0.01, t = '**'; elseif p < 0.05, t = '*'; else, t = '(not significant at 0.05)'; end
end
function t = effectLabel(d)
a = abs(d);
if a < 0.2, t = 'negligible'; elseif a < 0.5, t = 'small'; elseif a < 0.8, t = 'medium'; else, t = 'large'; end
end
