function tops(command, varargin)
%TOPS  Run the grasp simulation.
%
%   tops grasp              pick up the blue box
%   tops grasp red          pick up the red box
%                           (red, green, blue, yellow, orange, purple)
%   tops grasp bump         push the arm mid-move: the safety monitor reacts
%   tops grasp red bump     both
%
%   tops watch ...          the same runs, with the 3D arm
%   tops test               run the tests
%
%   Each run prints a timeline and results, and saves them to
%   results/grasp_<colour>[_bump]_<date>_<time>.mat as `result` and `log`.

here = fileparts(mfilename('fullpath'));
addpath(here);
if nargin == 0, help tops, return, end
P = grasp.config.settings();

switch lower(command)
    case 'grasp'
        runAndReport(P, 'light', varargin);

    case 'watch'
        runAndReport(P, 'full', varargin);

    case 'test'
        r = runtests(fullfile(here, 'tests'));
        disp(table(r));
        if any([r.Failed]), error('tops:test', '%d test(s) failed', sum([r.Failed])); end

    otherwise
        error('tops:command', 'unknown command "%s"; run `tops` for the list', command);
end
end

% =========================================================================
function runAndReport(P, visuals, args)
words = lower(string(args));
pushArm = any(words == "bump");
colour = words(words ~= "bump");
if isempty(colour), colour = string(P.world.default_box); end

mdl = readyModel(P, visuals);
set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', onOff(strcmp(visuals, 'full')));
[result, log] = grasp.sim.runGrasp(P, char(colour(1)), pushArm, mdl);
grasp.sim.printGraspSummary(log, result);

if ~isfolder(P.paths.results), mkdir(P.paths.results); end
name = "grasp_" + colour(1);
if pushArm, name = name + "_bump"; end
file = fullfile(P.paths.results, name + "_" + string(datetime('now', 'Format', 'yyyy-MM-dd_HHmmss')) + ".mat");
save(file, 'result', 'log');
fprintf('  saved to %s\n\n', file);
end

function mdl = readyModel(P, visuals)
% Load the generated model, rebuilding it only if its source has changed.
cache = fullfile(P.paths.model, 'cache');
if ~isfolder(cache), mkdir(cache); end
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache);
addpath(P.paths.model);

mdl = 'grasp_no_graphics';
if strcmp(visuals, 'full'), mdl = 'grasp_with_3d_arm'; end
file = dir(fullfile(P.paths.model, [mdl '.slx']));
sources = [dir(fullfile(P.paths.root, '+grasp', '+sim', 'buildArmModel.m')); ...
           dir(fullfile(P.paths.root, '+grasp', '+config', 'settings.m')); ...
           dir(fullfile(P.paths.root, '+grasp', '+robot', 'pandaPublishedData.m'))];
if isempty(file) || file.datenum < max([sources.datenum])
    fprintf('building %s ...\n', mdl);
    grasp.sim.buildArmModel(P, visuals);
elseif ~bdIsLoaded(mdl)
    load_system(mdl);
end
end

function s = onOff(tf)
if tf, s = 'on'; else, s = 'off'; end
end
