function mdl = buildArmModel(P, visuals)
%BUILDARMMODEL  Generate the whole simulation from code.
%
%   mdl = grasp.sim.buildArmModel(P)            % model/grasp_no_graphics.slx
%   mdl = grasp.sim.buildArmModel(P, 'full')    % model/grasp_with_3d_arm.slx
%
%   The two models have identical physics. The full one draws the 30 MB
%   orange visual meshes, which is what you want to look at but costs
%   gigabytes of memory to compile. The other draws small collision meshes.
%
%   The model is generated, never edited by hand: every block, number and
%   wire comes from this file and grasp.config.settings.
%
%   Plant (Simscape Multibody)
%     The table, the Panda (7 revolute joints, identified inertials and
%     friction), the Franka Hand on a weld joint that senses the wrench it
%     carries (the wrist sensor), two force-driven fingers, and six boxes on
%     free joints.
%
%   Controller       (MATLAB Function, 1 kHz)  grasp.control.controlArm
%   Supervisor       (MATLAB Function, 50 Hz)  grasp.supervisor.superviseGrasp
%   Box target       (MATLAB Function, 50 Hz)  grasp.perception.knownBoxPose
%
%   The model reads C, U, Q, km (grasp.config.controllerSettings) and Run
%   (grasp.sim.thisGraspSettings) at simulation time.
%
%   Three delays break the model's feedback loops: the force reading reaches
%   the controller 1 ms late, as a real sensor's does; the supervisor reads
%   the controller's status 1 ms late; the box target sees the phase one
%   tick (20 ms) late.

if nargin < 2, visuals = 'light'; end
full = strcmp(visuals, 'full');
if full
    mdl = 'grasp_with_3d_arm';
else
    mdl = 'grasp_no_graphics';
end
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system('simulink');  load_system('sm_lib');  load_system('nesl_utility');
new_system(mdl);

set_param(mdl, ...
    'SolverType', 'Variable-step', 'Solver', 'daessc', 'MaxStep', num2str(P.sim.max_step), 'RelTol', '1e-3', ...
    'StopTime', 'Run.timeout', ...
    'AutoInsertRateTranBlk', 'on', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'off', ...
    'AlgebraicLoopMsg', 'error');

% =========================================================================
%  The plant
% =========================================================================
plant = [mdl '/Plant'];
add_block('built-in/Subsystem', plant, 'Position', [400 100 600 400]);
buildPlant(plant, P, full);
% (Exists only once the model holds Simscape Multibody blocks.) No 3D
% window on a plain run; `tops watch` switches it back on.
set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off', 'SimscapeLogType', 'none');

% =========================================================================
%  Controller, supervisor, perception
% =========================================================================
addFcn([mdl '/Controller'], [700 100 900 300], num2str(P.ctrl.dt), { ...
    'function [tau, fFinger, status] = controller(q, qd, wRaw, cmd, gripCmd, qf, qfd, C, km)'
    '% The 1 kHz arm and gripper controller: grasp.control.controlArm.'
    'persistent S'
    'if isempty(S), S = grasp.control.startingControlState(C); end'
    '[tau, fFinger, status, S] = grasp.control.controlArm(S, q, qd, wRaw, cmd, gripCmd, qf, qfd, C, km);'
    }, {'C', 'km'}, {'q', 7; 'qd', 7; 'wRaw', 6; 'cmd', 9; 'gripCmd', 1; 'qf', 2; 'qfd', 2; ...
                       'tau', 7; 'fFinger', 2; 'status', 20});

addFcn([mdl '/Supervisor'], [700 400 900 600], num2str(P.phase.dt), { ...
    'function [cmd, gripCmd, sv] = supervisor(t, boxTarget, status, q, qf, qfd, Run, U)'
    '% The 50 Hz grasp phases: grasp.supervisor.superviseGrasp.'
    'persistent S'
    'if isempty(S), S = grasp.supervisor.startingSupervisorState(); end'
    '[cmd, gripCmd, sv, S] = grasp.supervisor.superviseGrasp(S, t, boxTarget, status, q, qf, qfd, Run.truth_grasp(:), U);'
    }, {'Run', 'U'}, {'t', 1; 'boxTarget', 9; 'status', 20; 'q', 7; 'qf', 2; 'qfd', 2; ...
                       'cmd', 9; 'gripCmd', 1; 'sv', 14});

addFcn([mdl '/Box target'], [700 700 900 800], num2str(P.phase.dt), { ...
    'function boxTarget = boxTargetBlock(t, phase, Run, Q)'
    '% Where the chosen box is: grasp.perception.knownBoxPose.'
    'boxTarget = grasp.perception.knownBoxPose(t, phase, Run, Q);'
    }, {'Run', 'Q'}, {'t', 1; 'phase', 1; 'boxTarget', 9});

add_block('simulink/Sources/Clock', [mdl '/Clock'], 'Position', [500 650 530 680]);

% F/T reading, one control period late (a real sensor's latency; also
% breaks the algebraic loop through the weld joint).
add_block('simulink/Discrete/Unit Delay', [mdl '/FT latency'], ...
    'SampleTime', num2str(P.ctrl.dt), 'InitialCondition', 'zeros(6,1)', 'Position', [620 180 650 210]);
% Controller diagnostics, one control period late, to the supervisor
% (breaks the loop supervisor -> command -> controller -> diagnostics).
add_block('simulink/Discrete/Unit Delay', [mdl '/diagnostics latency'], ...
    'SampleTime', num2str(P.ctrl.dt), 'InitialCondition', 'zeros(20,1)', 'Position', [950 250 980 280]);
% Grasp phase, one tick late, to the box target.
add_block('simulink/Signal Routing/Selector', [mdl '/state'], ...
    'InputPortWidth', '14', 'Indices', '1', 'Position', [950 520 980 540]);
add_block('simulink/Discrete/Unit Delay', [mdl '/state latency'], ...
    'SampleTime', num2str(P.phase.dt), 'Position', [1000 520 1030 540]);

% Plant outputs: 1 q, 2 qd, 3 wrench, 4 qf, 5 qfd, 6 box position, 7 contact forces
line(mdl, 'Plant', 1, 'Controller', 1);
line(mdl, 'Plant', 2, 'Controller', 2);
line(mdl, 'Plant', 3, 'FT latency', 1);
line(mdl, 'FT latency', 1, 'Controller', 3);
line(mdl, 'Supervisor', 1, 'Controller', 4);
line(mdl, 'Supervisor', 2, 'Controller', 5);
line(mdl, 'Plant', 4, 'Controller', 6);
line(mdl, 'Plant', 5, 'Controller', 7);
line(mdl, 'Controller', 1, 'Plant', 1);
line(mdl, 'Controller', 2, 'Plant', 2);

% The push for `tops grasp bump`: zero unless the run asks for it.
add_block('simulink/Lookup Tables/1-D Lookup Table', [mdl '/push profile'], ...
    'Table', 'Run.push_size', 'BreakpointsForDimension1', 'Run.push_time', 'Position', [250 200 300 240]);
add_block('simulink/Math Operations/Gain', [mdl '/push force'], ...
    'Gain', 'Run.push_force(:)', 'Multiplication', 'Matrix(K*u)', 'Position', [330 200 380 240]);
line(mdl, 'Clock', 1, 'push profile', 1);
line(mdl, 'push profile', 1, 'push force', 1);
line(mdl, 'push force', 1, 'Plant', 3);

line(mdl, 'Clock', 1, 'Supervisor', 1);
line(mdl, 'Box target', 1, 'Supervisor', 2);
line(mdl, 'Controller', 3, 'diagnostics latency', 1);
line(mdl, 'diagnostics latency', 1, 'Supervisor', 3);
line(mdl, 'Plant', 1, 'Supervisor', 4);
line(mdl, 'Plant', 4, 'Supervisor', 5);
line(mdl, 'Plant', 5, 'Supervisor', 6);

line(mdl, 'Clock', 1, 'Box target', 1);
line(mdl, 'Supervisor', 3, 'state', 1);
line(mdl, 'state', 1, 'state latency', 1);
line(mdl, 'state latency', 1, 'Box target', 2);

% ---- stop when the supervisor says the run is over ---------------------
add_block('simulink/Signal Routing/Selector', [mdl '/done'], ...
    'InputPortWidth', '14', 'Indices', '4', 'Position', [950 580 980 600]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/is done'], ...
    'relop', '>=', 'const', '1', 'Position', [1000 580 1040 600]);
add_block('simulink/Sinks/Stop Simulation', [mdl '/Stop'], 'Position', [1070 580 1100 600]);
line(mdl, 'Supervisor', 3, 'done', 1);
line(mdl, 'done', 1, 'is done', 1);
line(mdl, 'is done', 1, 'Stop', 1);

% ---- logging: one 100 Hz matrix per run ----------------------------------
% Columns: t | sv (14) | status (20) | box position (3) | contact forces (7) | qf (2)
add_block('simulink/Signal Routing/Mux', [mdl '/log mux'], 'Inputs', '6', 'Position', [1150 300 1155 460]);
add_block('simulink/Sinks/To Workspace', [mdl '/runLog'], ...
    'VariableName', 'runLog', 'SaveFormat', 'Array', ...
    'SampleTime', num2str(P.sim.log_dt), 'MaxDataPoints', 'inf', 'Position', [1200 360 1260 400]);
line(mdl, 'Clock', 1, 'log mux', 1);
line(mdl, 'Supervisor', 3, 'log mux', 2);
line(mdl, 'Controller', 3, 'log mux', 3);
line(mdl, 'Plant', 6, 'log mux', 4);
line(mdl, 'Plant', 7, 'log mux', 5);
line(mdl, 'Plant', 4, 'log mux', 6);
line(mdl, 'log mux', 1, 'runLog', 1);

if ~isfolder(P.paths.model), mkdir(P.paths.model); end
save_system(mdl, fullfile(P.paths.model, [mdl '.slx']));

% =========================================================================
%  Nested helpers
% =========================================================================
    function h = addFcn(path, pos, sampleTime, code, params, sizes)
        % A MATLAB Function block running `code` at `sampleTime`, with the
        % named arguments taken from the workspace as parameters. Every
        % port's size is stated: around the model's feedback loops Simulink
        % cannot infer them.
        h = add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
        ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', path);
        ch.Script = strjoin(code, newline);
        set_param(path, 'SystemSampleTime', sampleTime);   % (the chart's own property does not persist)
        for k = 1:numel(params)
            d = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
            d.Scope = 'Parameter';
        end
        for k = 1:size(sizes, 1)
            d = ch.find('-isa', 'Stateflow.Data', 'Name', sizes{k, 1});
            d.Props.Array.Size = sprintf('[%d 1]', sizes{k, 2});
        end
    end
end

% =========================================================================
%  The Simscape plant
% =========================================================================
function buildPlant(sys, P, full)
R = P.robot;
cp = P.contact;
x = 3;   % layout cursor for this subsystem (0-2 are the inports)

% ---- environment ---------------------------------------------------------
W   = blk('sm_lib/Frames and Transforms/World Frame', 'World');
MC  = blk('sm_lib/Utilities/Mechanism Configuration', 'Mechanism Configuration');
SC  = blk('nesl_utility/Solver Configuration', 'Solver Configuration');
frame(W, 'R', 1, MC, 'R', 1);
frame(W, 'R', 1, SC, 'R', 1);

% ---- the table -------------------------------------------------------------
% The table you SEE is a brick; the table things TOUCH is an infinite plane
% at its top surface (z = 0.75 m). See "contacts" below for why.
Ttab = rt('Table pose', P.world.table_center, []);
frame(W, 'R', 1, Ttab, 'L', 1);
table = brick('Table', mat2str(P.world.table_size), '[0.72 0.60 0.44]', 'density', '', false);
frame(Ttab, 'R', 1, table, 'R', 1);
Ttop = rt('Table top', [P.world.table_center(1:2), P.world.table_center(3) + P.world.table_size(3) / 2], []);
frame(W, 'R', 1, Ttop, 'L', 1);
tablePlane = blk('sm_lib/Curves and Surfaces/Infinite Plane', 'Table surface');
frame(Ttop, 'R', 1, tablePlane, 'L', 1);           % frame on the left, geometry on the right

% ---- the arm ---------------------------------------------------------------
Tbase = rt('Arm base', R.base_in_world, []);
frame(W, 'R', 1, Tbase, 'L', 1);
link = fileSolid('link0', R.link(1));
frame(Tbase, 'R', 1, link, 'R', 1);
prev = Tbase;  prevSide = 'R';

tauIn = add_block('built-in/Inport', [sys '/tau'], 'Port', '1', 'Position', pos(x));
fIn   = add_block('built-in/Inport', [sys '/fFinger'], 'Port', '2', 'Position', pos(x + 1));
pushIn = add_block('built-in/Inport', [sys '/push'], 'Port', '3', 'Position', pos(x + 2));

% The measured joint torque cannot change faster than Franka's limit, and
% friction follows Gaz et al. (pandaPublishedData), built from element-wise blocks
% so that it stays continuous and needs no MATLAB Function in the loop:
%   tau_applied = rate-limited tau - tau_f(qd)
qMux  = blk('simulink/Signal Routing/Mux', 'q mux');   set_param(qMux, 'Inputs', '7');
qdMux = blk('simulink/Signal Routing/Mux', 'qd mux');  set_param(qdMux, 'Inputs', '7');
rl = blk('simulink/Discontinuities/Rate Limiter', 'torque rate limit');
set_param(rl, 'RisingSlewLimit', num2str(R.tau_rate_max), 'FallingSlewLimit', num2str(-R.tau_rate_max));
phi1 = R.friction_phi1(:);  phi2 = R.friction_phi2(:);  phi3 = R.friction_phi3(:);
bV = blk('simulink/Math Operations/Bias', 'qd + phi3');  set_param(bV, 'Bias', mat2str(phi3, 17));
gE = blk('simulink/Math Operations/Gain', '-phi2');
set_param(gE, 'Gain', mat2str(-phi2, 17), 'Multiplication', 'Element-wise(K.*u)');
ex = blk('simulink/Math Operations/Math Function', 'exp');  set_param(ex, 'Operator', 'exp');
b1 = blk('simulink/Math Operations/Bias', '1 + exp');  set_param(b1, 'Bias', '1');
ri = blk('simulink/Math Operations/Math Function', 'reciprocal');  set_param(ri, 'Operator', 'reciprocal');
gF = blk('simulink/Math Operations/Gain', 'phi1');
set_param(gF, 'Gain', mat2str(phi1, 17), 'Multiplication', 'Element-wise(K.*u)');
bF = blk('simulink/Math Operations/Bias', 'zero at rest');
set_param(bF, 'Bias', mat2str(-phi1 ./ (1 + exp(-phi2 .* phi3)), 17));
sumT = blk('simulink/Math Operations/Sum', 'tau minus friction');  set_param(sumT, 'Inputs', '+-');
dmx = blk('simulink/Signal Routing/Demux', 'tau demux');  set_param(dmx, 'Outputs', '7');
sig(tauIn, 1, rl, 1);  sig(rl, 1, sumT, 1);
sig(qdMux, 1, bV, 1);  sig(bV, 1, gE, 1);  sig(gE, 1, ex, 1);  sig(ex, 1, b1, 1);
sig(b1, 1, ri, 1);  sig(ri, 1, gF, 1);  sig(gF, 1, bF, 1);  sig(bF, 1, sumT, 2);
sig(sumT, 1, dmx, 1);

for i = 1:7
    T = rt(sprintf('Joint %d origin', i), R.joint_xyz(i, :), R.joint_rpy(i, :));
    frame(prev, prevSide, 1, T, 'L', 1);
    J = blk('sm_lib/Joints/Revolute Joint', sprintf('panda_joint%d', i));
    set_param(J, 'TorqueActuationMode', 'InputTorque', ...
        'SensePosition', 'on', 'SenseVelocity', 'on', ...
        'DampingCoefficient', num2str(R.damping(i)), 'DampingCoefficientUnits', 'N*m/(rad/s)', ...
        'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
        'PositionTargetValue', num2str(R.q_home(i)), 'PositionTargetValueUnits', 'rad', ...
        'LowerLimitSpecify', 'on', 'LowerLimitBound', num2str(R.q_lower(i)), 'LowerLimitBoundUnits', 'rad', ...
        'UpperLimitSpecify', 'on', 'UpperLimitBound', num2str(R.q_upper(i)), 'UpperLimitBoundUnits', 'rad', ...
        'LowerLimitTransitionRegionWidth', '1e-3', 'LowerLimitTransitionRegionWidthUnits', 'rad', ...
        'UpperLimitTransitionRegionWidth', '1e-3', 'UpperLimitTransitionRegionWidthUnits', 'rad');
    frame(T, 'R', 1, J, 'L', 1);
    link = fileSolid(sprintf('link%d', i), R.link(i + 1));
    frame(J, 'R', 1, link, 'R', 1);
    % torque in (LConn2), angle out (RConn2), speed out (RConn3)
    sps = blk('nesl_utility/Simulink-PS Converter', sprintf('tau%d', i));  set_param(sps, 'Unit', 'N*m');
    sig(dmx, i, sps, 1);
    frame(sps, 'R', 1, J, 'L', 2);
    pq = blk('nesl_utility/PS-Simulink Converter', sprintf('q%d', i));   set_param(pq, 'Unit', 'rad');
    pw = blk('nesl_utility/PS-Simulink Converter', sprintf('qd%d', i));  set_param(pw, 'Unit', 'rad/s');
    frame(J, 'R', 2, pq, 'L', 1);
    frame(J, 'R', 3, pw, 'L', 1);
    sig(pq, 1, qMux, i);
    sig(pw, 1, qdMux, i);
    prev = J;  prevSide = 'R';
end

% ---- flange, wrist F/T sensor, hand ---------------------------------------
% The weld joint IS the sensor: it reports the total force and torque the
% hand exerts on the flange, resolved in the hand frame, about its origin.
Tf = rt('Flange and hand mount', [0 0 R.flange_z], [0 0 P.robot.hand_yaw]);
frame(prev, prevSide, 1, Tf, 'L', 1);
ft = blk('sm_lib/Joints/Weld Joint', 'FT sensor');
set_param(ft, 'SenseTotalForce', 'on', 'SenseTotalTorque', 'on', ...
    'CompositeWrenchDir', 'FollowerOnBase', 'CompositeWrenchFrame', 'FollowerFrame');
frame(Tf, 'R', 1, ft, 'L', 1);
hand = fileSolid('hand', R.hand);
set_param(hand, 'ExportConvexHullGeometry', 'on');      % contact shape
frame(ft, 'R', 1, hand, 'R', 1);
pf = blk('nesl_utility/PS-Simulink Converter', 'force');   set_param(pf, 'Unit', 'N');
pt = blk('nesl_utility/PS-Simulink Converter', 'torque');  set_param(pt, 'Unit', 'N*m');
frame(ft, 'R', 2, pf, 'L', 1);
frame(ft, 'R', 3, pt, 'L', 1);
wMux = blk('simulink/Signal Routing/Mux', 'wrench mux');  set_param(wMux, 'Inputs', '2');
sig(pf, 1, wMux, 1);  sig(pt, 1, wMux, 2);

% The push for `tops grasp bump`. It acts on the hand, beyond the wrist
% sensor, so the sensor feels it just as it would a real collision.
pushBlock = blk('sm_lib/Forces and Torques/External Force and Torque', 'Push on hand');
set_param(pushBlock, 'EnableForce', 'on', 'ForceResolutionFrame', 'World');
pushPS = blk('nesl_utility/Simulink-PS Converter', 'push force');  set_param(pushPS, 'Unit', 'N');
sig(pushIn, 1, pushPS, 1);
frame(pushPS, 'R', 1, pushBlock, 'L', 1);
frame(hand, 'R', 1, pushBlock, 'R', 1);

% ---- fingers ----------------------------------------------------------------
% Each finger slides along the hand's +y (left) or -y (right). Simscape's
% prismatic joint slides along its base frame's z, so the frame is turned
% -90 deg about x before the joint and +90 deg back after it.
fMux  = blk('simulink/Signal Routing/Mux', 'qf mux');   set_param(fMux, 'Inputs', '2');
fdMux = blk('simulink/Signal Routing/Mux', 'qfd mux');  set_param(fdMux, 'Inputs', '2');
fDmx  = blk('simulink/Signal Routing/Demux', 'finger demux');  set_param(fDmx, 'Outputs', '2');
sig(fIn, 1, fDmx, 1);
pads = zeros(1, 2);
names = {'left', 'right'};
for s = 1:2
    if s == 1
        T0 = rt('Left finger mount', [0 0 R.finger_z], [-pi/2 0 0]);
        frame(hand, 'R', 1, T0, 'L', 1);
    else
        Tz = rt('Right finger turn', [0 0 R.finger_z], [0 0 pi]);
        frame(hand, 'R', 1, Tz, 'L', 1);
        T0 = rt('Right finger mount', [0 0 0], [-pi/2 0 0]);
        frame(Tz, 'R', 1, T0, 'L', 1);
    end
    Jf = blk('sm_lib/Joints/Prismatic Joint', sprintf('panda_finger_joint%d', s));
    set_param(Jf, 'TorqueActuationMode', 'InputTorque', ...
        'SensePosition', 'on', 'SenseVelocity', 'on', ...
        'DampingCoefficient', '0.3', 'DampingCoefficientUnits', 'N/(m/s)', ...
        'PositionTargetSpecify', 'on', 'PositionTargetPriority', 'High', ...
        'PositionTargetValue', num2str(P.phase.gripper_open), 'PositionTargetValueUnits', 'm', ...
        'LowerLimitSpecify', 'on', 'LowerLimitBound', '0', 'LowerLimitBoundUnits', 'm', ...
        'UpperLimitSpecify', 'on', 'UpperLimitBound', num2str(R.finger_max), 'UpperLimitBoundUnits', 'm');
    frame(T0, 'R', 1, Jf, 'L', 1);
    T1 = rt(sprintf('%s finger frame', names{s}), [0 0 0], [pi/2 0 0]);
    frame(Jf, 'R', 1, T1, 'L', 1);
    fin = fileSolid('finger', R.finger, sprintf('%s finger', names{s}));
    frame(T1, 'R', 1, fin, 'R', 1);
    % Franka's rubber fingertip pad (17.5 x 15.2 x 18.5 mm, its gripping
    % face at y = 0 in the finger frame) as contact points: a 3 x 3 grid on
    % the gripping face plus the two outer bottom corners (which meet the
    % table if the hand goes too low).
    pads(s) = blk('sm_lib/Curves and Surfaces/Point Cloud', sprintf('%s pad contact points', names{s}));
    set_param(pads(s), 'CoordinatesMatrix', mat2str(padPoints(), 6), 'GraphicType', 'None');   % contact only, never drawn
    frame(T1, 'R', 1, pads(s), 'L', 1);
    sf = blk('nesl_utility/Simulink-PS Converter', sprintf('finger force %d', s));  set_param(sf, 'Unit', 'N');
    sig(fDmx, s, sf, 1);
    frame(sf, 'R', 1, Jf, 'L', 2);
    pq = blk('nesl_utility/PS-Simulink Converter', sprintf('qf%d', s));   set_param(pq, 'Unit', 'm');
    pw = blk('nesl_utility/PS-Simulink Converter', sprintf('qfd%d', s));  set_param(pw, 'Unit', 'm/s');
    frame(Jf, 'R', 2, pq, 'L', 1);
    frame(Jf, 'R', 3, pw, 'L', 1);
    sig(pq, 1, fMux, s);
    sig(pw, 1, fdMux, s);
end

% ---- the boxes ----------------------------------------------------------------
% Six free boxes. Slot 1 is always the one being picked up.
nBoxes = numel(P.world.boxes);
boxes = zeros(1, nBoxes);  corners = zeros(1, nBoxes);
for n = 1:nBoxes
    free = blk('sm_lib/Joints/6-DOF Joint', sprintf('Box %d free joint', n));
    set_param(free, ...
        'PxPositionTargetSpecify', 'on', 'PxPositionTargetValue', sprintf('Run.box_world(%d,1)', n), 'PxPositionTargetValueUnits', 'm', ...
        'PyPositionTargetSpecify', 'on', 'PyPositionTargetValue', sprintf('Run.box_world(%d,2)', n), 'PyPositionTargetValueUnits', 'm', ...
        'PzPositionTargetSpecify', 'on', 'PzPositionTargetValue', sprintf('Run.box_world(%d,3)', n), 'PzPositionTargetValueUnits', 'm');
    frame(W, 'R', 1, free, 'L', 1);
    boxes(n) = brick(sprintf('Box %d', n), sprintf('Run.box_size(%d)*[1 1 1]', n), sprintf('Run.box_color(%d,:)', n), ...
        'mass', sprintf('Run.box_mass(%d)', n), true);
    frame(free, 'R', 1, boxes(n), 'R', 1);
    % Its eight corners as contact points (against the table and the hand).
    corners(n) = blk('sm_lib/Curves and Surfaces/Point Cloud', sprintf('Box %d corner points', n));
    set_param(corners(n), 'CoordinatesMatrix', ...
        sprintf('Run.box_size(%d)/2*[-1 -1 -1; 1 -1 -1; -1 1 -1; 1 1 -1; -1 -1 1; 1 -1 1; -1 1 1; 1 1 1]', n), 'GraphicType', 'None');
    frame(boxes(n), 'R', 1, corners(n), 'L', 1);
end
ts = blk('sm_lib/Frames and Transforms/Transform Sensor', 'Box 1 position');
set_param(ts, 'SenseXYZ', 'on', 'MeasurementFrame', 'World');
frame(W, 'R', 1, ts, 'L', 1);
frame(boxes(1), 'R', 1, ts, 'R', 1);
po = blk('nesl_utility/PS-Simulink Converter', 'box xyz');  set_param(po, 'Unit', 'm');
frame(ts, 'R', 2, po, 'L', 1);

% ---- contacts ----------------------------------------------------------------
% Contact PROXIES, not face-against-face solids. A flat face resting on a
% flat face (block on table, pad on block) has several equally deep
% points, and a single-point convex contact keeps flipping between them:
% measured on a block resting on the table, that forced 4e-7 s solver
% steps (750x slower than real time) or, with a fixed step, reported 18 N
% of contact force under a 1.47 N block. With points against surfaces each
% point carries its own smooth force: the same block reads 1.471 N exactly
% and simulates faster than real time.
%
% Each row: base geometry (block, side, port), follower geometry, name.
% Only the first seven are logged. The other boxes rest on the table and can
% be knocked by the hand; the fingertips are never near them.
pairs = {boxes(1), 'L', 1,   pads(1), 'R', 1,  'left pad - box'; ...
         boxes(1), 'L', 1,   pads(2), 'R', 1,  'right pad - box'; ...
         hand, 'L', 1,       corners(1), 'R', 1,  'hand - box'; ...
         tablePlane, 'R', 1, corners(1), 'R', 1,  'table - box'; ...
         tablePlane, 'R', 1, pads(1), 'R', 1,  'left pad - table'; ...
         tablePlane, 'R', 1, pads(2), 'R', 1,  'right pad - table'; ...
         tablePlane, 'R', 1, hand, 'L', 1,     'hand - table'};
nLogged = size(pairs, 1);
for n = 2:nBoxes
    pairs = [pairs; {tablePlane, 'R', 1, corners(n), 'R', 1, sprintf('table - box %d', n); ...
                     hand, 'L', 1,       corners(n), 'R', 1, sprintf('hand - box %d', n)}]; %#ok<AGROW>
end
cMux = blk('simulink/Signal Routing/Mux', 'contact mux');  set_param(cMux, 'Inputs', num2str(nLogged));
for k = 1:size(pairs, 1)
    c = blk('sm_lib/Forces and Torques/Spatial Contact Force', pairs{k, 7});
    set_param(c, 'NormalStiffness', num2str(cp.stiffness), 'NormalStiffnessUnits', 'N/m', ...
        'NormalDamping', num2str(cp.damping), 'NormalDampingUnits', 'N/(m/s)', ...
        'NormalTransitionRegionWidth', num2str(cp.transition), 'NormalTransitionRegionWidthUnits', 'm', ...
        'CoefficientOfStaticFriction', num2str(cp.mu_static), ...
        'CoefficientOfDynamicFriction', num2str(cp.mu_kinetic), ...
        'FrictionalCriticalVelocity', num2str(cp.critical_velocity), 'FrictionalCriticalVelocityUnits', 'm/s');
    frame(pairs{k, 1}, pairs{k, 2}, pairs{k, 3}, c, 'L', 1);
    frame(pairs{k, 4}, pairs{k, 5}, pairs{k, 6}, c, 'R', 1);
    if k <= nLogged
        % One normal force per contact point: summed into one number per pair.
        set_param(c, 'SenseNormalForceMagnitude', 'on');
        pc = blk('nesl_utility/PS-Simulink Converter', ['fn ' pairs{k, 7}]);  set_param(pc, 'Unit', 'N');
        frame(c, 'R', 2, pc, 'L', 1);
        se = blk('simulink/Math Operations/Sum of Elements', ['total ' pairs{k, 7}]);
        sig(pc, 1, se, 1);
        sig(se, 1, cMux, k);
    end
end

% ---- outputs -------------------------------------------------------------------
outs = {qMux, 'q'; qdMux, 'qd'; wMux, 'wrench'; fMux, 'qf'; fdMux, 'qfd'; po, 'box_xyz'; cMux, 'contact_forces'};
for k = 1:size(outs, 1)
    o = add_block('built-in/Outport', [sys '/' outs{k, 2}], 'Port', num2str(k), 'Position', pos(x));
    x = x + 1;
    sig(outs{k, 1}, 1, o, 1);
end

% ---- nested helpers ---------------------------------------------------------------
    function p = pos(n)
        p = [100 + 160 * mod(n, 12), 60 + 90 * floor(n / 12)];
        p = [p, p + [120 50]];
    end
    function h = blk(lib, name)
        h = add_block(lib, [sys '/' name], 'Position', pos(x));
        x = x + 1;
    end
    function h = rt(name, xyz, rpy)
        % A rigid transform: translate by xyz, then rotate by fixed-axis
        % roll-pitch-yaw (R = Rz Ry Rx, the URDF/Franka convention).
        h = blk('sm_lib/Frames and Transforms/Rigid Transform', name);
        if any(xyz ~= 0)
            set_param(h, 'TranslationMethod', 'Cartesian', ...
                'TranslationCartesianOffset', mat2str(xyz, 17), 'TranslationCartesianOffsetUnits', 'm');
        end
        if ~isempty(rpy) && any(rpy ~= 0)
            set_param(h, 'RotationMethod', 'RotationSequence', 'RotationSequenceAxes', 'BaseAxes', ...
                'RotationSequence', 'XYZ', 'RotationSequenceAngles', mat2str(rpy, 17), ...
                'RotationSequenceAnglesUnits', 'rad');
        end
    end
    function h = fileSolid(mesh, inertia, name)
        % A link: its real CAD mesh, with its published mass properties.
        if nargin < 3, name = mesh; end
        h = blk('sm_lib/Body Elements/File Solid', name);
        I = inertia.I;
        % The file gives the shape only: File Solid cannot read the OBJ
        % materials and draws the link black, so the colour is set here.
        % Light build: the collision mesh (the finger has none, so its small
        % visual mesh).
        colour = '[0.93 0.40 0.07]';
        if strcmp(mesh, 'finger'), colour = '[0.18 0.18 0.20]'; end
        if full || strcmp(mesh, 'finger')
            file = fullfile(P.paths.visual, [mesh '.obj']);
        else
            file = fullfile(P.paths.collision, [mesh '.stl']);
        end
        vis = {'GraphicVisPropType', 'SimpleVisualProperties', 'GraphicDiffuseColor', colour};
        set_param(h, 'ExtGeomFileName', file, ...
            'UnitType', 'Custom', 'ExtGeomFileUnits', 'm', ...
            'InertiaType', 'Custom', ...
            'Mass', num2str(inertia.mass, 17), 'MassUnits', 'kg', ...
            'CenterOfMass', mat2str(inertia.com, 17), 'CenterOfMassUnits', 'm', ...
            'MomentsOfInertia', mat2str([I(1,1) I(2,2) I(3,3)], 17), 'MomentsOfInertiaUnits', 'kg*m^2', ...
            'ProductsOfInertia', mat2str([I(2,3) I(1,3) I(1,2)], 17), 'ProductsOfInertiaUnits', 'kg*m^2', ...
            vis{:});
    end
    function h = brick(name, dims, colour, inertiaKind, value, contact)
        h = blk('sm_lib/Body Elements/Brick Solid', name);
        set_param(h, 'BrickDimensions', dims, 'BrickDimensionsUnits', 'm', 'GraphicDiffuseColor', colour);
        switch inertiaKind
            case 'mass'
                set_param(h, 'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Mass', ...
                    'Mass', value, 'MassUnits', 'kg');
            case 'point'
                set_param(h, 'InertiaType', 'PointMass', 'Mass', value, 'MassUnits', 'kg', 'GraphicType', 'None');
        end
        if contact
            set_param(h, 'ExportEntireGeometry', 'on');
        end
    end
    function frame(a, aSide, ai, b, bSide, bi)
        % Connect Simscape port aSide(ai) of block a to bSide(bi) of block b.
        pa = get_param(a, 'PortHandles');  pb = get_param(b, 'PortHandles');
        add_line(sys, pa.([aSide 'Conn'])(ai), pb.([bSide 'Conn'])(bi));   % no autorouting: far too slow here
    end
    function sig(a, ai, b, bi)
        pa = get_param(a, 'PortHandles');  pb = get_param(b, 'PortHandles');
        add_line(sys, pa.Outport(ai), pb.Inport(bi));
    end
end

% =========================================================================
function P = padPoints()
% Contact points of one Franka rubber fingertip pad, in the finger frame.
% The pad is 17.5 (x) by 15.2 (y) by 18.5 (z) mm, centred at
% (0, 7.58, 45.25) mm, so its gripping face is the plane y = 0.
[X, Z] = ndgrid([-8.75 0 8.75], [36.0 45.25 54.5]);
face = [X(:), zeros(9, 1), Z(:)];
outer = [-8.75 15.16 36.0; 8.75 15.16 36.0];
P = [face; outer] / 1000;
end

function line(mdl, a, ai, b, bi)
add_line(mdl, sprintf('%s/%d', a, ai), sprintf('%s/%d', b, bi), 'autorouting', 'on');
end
