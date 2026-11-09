function P = settings()
%SETTINGS  Every number in the simulation, in one place, with reasons.
%
%   P = grasp.config.settings()
%
%   Units are SI throughout: metres, seconds, kilograms, newtons, radians.
%   Frames: "base" is panda_link0. "world" has z up; the arm's base sits at
%   world (-0.05, 0, 0.75), on the back edge of a 1 m x 1 m table.

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));   % .../MATLAB
P.paths.root      = root;
P.paths.visual    = fullfile(root, 'robot', 'meshes', 'visual');
P.paths.collision = fullfile(root, 'robot', 'meshes', 'collision');
P.paths.model     = fullfile(root, 'model');
P.paths.results   = fullfile(root, 'results');

% =====================================================================
%  Robot: published data for the real arm
% =====================================================================
P.robot = grasp.robot.pandaPublishedData();

% The home configuration: elbow up, wrist down. q7 = +45 deg because the
% Franka Hand is mounted at -45 deg; at q7 = 45 deg the fingers line up
% with the base axes again. Doubles as the posture target.
P.robot.q_home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785];

% Where the arm's base sits in the world (on the table's back edge).
P.robot.base_in_world = [-0.05, 0, 0.75];

% The Franka Hand: mounted on the flange (link8) at -45 deg about z; the
% TCP is 103.4 mm out along the hand's z axis, between the fingertip pads.
P.robot.hand_yaw   = -pi/4;
P.robot.tcp_offset = 0.1034;
P.robot.finger_z   = 0.0584;          % finger joint origin on the hand's z axis
P.robot.finger_max = 0.04;            % per finger, metres (80 mm max opening)

% Grip force per finger. The Franka Hand sustains 70 N; a 150 g box needs
% under 1 N to lift (mu = 1.1, two fingers). The fingers are position
% controlled through a PD clamped at this force, so a closed gripper
% squeezes with a bounded force.
P.robot.finger_force_max = 20;
P.robot.finger_speed = 0.05;          % m/s per finger, as the real hand
P.robot.finger_kp = 3000;             % N/m
P.robot.finger_kd = 40;               % N/(m/s)

% =====================================================================
%  Wrist force/torque sensor
% =====================================================================
% The weld joint between panda_link8 and panda_hand reports the force and
% torque it carries, in the hand frame, about the hand origin. What hangs
% from it is the hand plus both fingers:
%   mass   = 0.6544 + 2 x 0.0291                         = 0.7126 kg
%   COM z  = (0.6544 x 0.020726 + 0.0582 x 0.0803675) / 0.7126 = 0.02560 m
%   COM y  = 0.6544 x 0.0119128 / 0.7126                 = 0.01094 m
P.sensor.tool_mass = 0.7126;
P.sensor.tool_com  = [0, 0.01094, 0.02560];   % hand frame, from the sensor origin
P.sensor.cutoff_control = 30;         % Hz: smoothing for the control laws
P.sensor.cutoff_safety  = 120;        % Hz: the safety monitor reads a faster signal
P.sensor.cutoff_accel = 15;           % Hz: filter on the TCP acceleration estimate
P.sensor.deadband = 0.3;              % N
P.sensor.clamp = 50;                  % N
P.sensor.zeroing_samples = 500;       % 0.5 s of averaging at start-up

% =====================================================================
%  Arm control (1 kHz)
% =====================================================================
P.ctrl.dt = 0.001;
P.ctrl.torque_limits = [87 87 87 87 12 12 12];
P.ctrl.torque_rate_limit = 1000;      % N.m/s per joint
P.ctrl.singularity_damping = 0.05;
P.ctrl.posture_stiffness = 20;
P.ctrl.posture_damping_ratio = 1.0;
P.ctrl.joint_limit_margin = 0.15;     % rad: one-sided spring inside this band
P.ctrl.joint_limit_stiffness = 40;    % N.m/rad
P.ctrl.max_target_jump = 0.25;        % m; targets further than this are refused
P.ctrl.damping_ratio = 0.9;           % same for both control modes: no step at the switch

% Task-space stiffness in each mode [x y z rx ry rz].
% Moving: 2500 N/m keeps the tracking error near 10 mm at full travel speed.
% Gripping: 220 N/m means an 8 mm position error pushes with under 2 N.
P.stiffness.moving   = [2500 2500 2500 150 150 150];
P.stiffness.gripping = [220 220 220 16 16 16];
P.stiffness.change_time = 1.0;        % s to go from moving to gripping stiffness
P.stiffness.stop_time = 0.1;          % s to shed it after a safety stop

% Admittance: the virtual mass-spring-damper that lets a measured force
% move the reference while the arm is travelling.
P.admittance.mass      = [6 6 6 0.25 0.25 0.25];
P.admittance.damping   = [60 60 60 4 4 4];
P.admittance.stiffness = [150 150 150 12 12 12];
P.admittance.max_offset = 0.10;       % m: the reference may not wander further
P.admittance.max_speed = 0.5;         % m/s

% Contact force regulation along the approach axis, while descending.
P.force.axis_in_tool = [0 0 1];       % +z of the TCP
P.force.target = 5;                   % N
P.force.contact_threshold = 1;        % N: below this the force loop is off
P.force.kp = 0.5;
P.force.ki = 2.0;
P.force.integral_limit = 3;           % N of integral authority
P.force.axis_stiffness_scale = 0.2;   % position and force share one axis: soften it

% =====================================================================
%  Wrench safety monitor
% =====================================================================
% Warn slows the arm down. Stop goes limp and retreats. Nothing should be
% touching the arm while it travels, so those limits are much lower.
P.safety.force_warn_moving  = 4;      % N
P.safety.force_stop_moving  = 8;      % N
P.safety.force_warn_contact = 10;     % N
P.safety.force_stop_contact = 15;     % N
P.safety.torque_warn = 1.5;           % N.m
P.safety.torque_stop = 3.0;           % N.m
P.safety.spike_limit = 3;             % N: fast minus slow filter, while moving only
P.safety.frozen_samples = 50;         % identical readings in a row = dead sensor
P.safety.warn_speed_scale = 0.25;     % travel speed while warned

% =====================================================================
%  Grasp phases (50 Hz)
% =====================================================================
P.phase.dt = 0.02;
P.phase.descend_trigger_distance = 0.015;
P.phase.descend_trigger_force = 1.5;
P.phase.touch_radius = 0.10;
P.phase.close_tolerance = 0.003;
P.phase.settle_before_close = 7.0;
P.phase.min_confidence = 0.40;
P.phase.target_timeout = 0.10;
P.phase.home_timeout = 10;
P.phase.move_timeout = 12;
P.phase.descend_timeout = 16;
P.phase.close_timeout = 3;
P.phase.lift_timeout = 10;
P.phase.gain_settle_time = 0.05;
P.phase.retreat_time = 1.5;
P.phase.retreat_distance = 0.05;
P.phase.retreat_speed = 0.05;         % m/s
P.phase.lift_height = 0.10;
P.phase.lift_success_height = 0.05;
P.phase.move_speed = 0.20;            % m/s: slew rate of the commanded pose
P.phase.descend_speed = 0.05;         % m/s
P.phase.home_tolerance = 0.05;        % rad, every joint
P.phase.gripper_open = 0.039;         % per finger
P.phase.gripper_closed = 0.018;       % per finger: narrower than any box, so it squeezes
P.phase.gripper_grasp_margin = 0.004;
P.phase.gripper_still_speed = 0.002;
P.phase.gripper_min_close_time = 0.4;
P.phase.hold_time = 1.0;              % s in HOLD before the run ends

% =====================================================================
%  Where the box is (no camera yet)
% =====================================================================
P.box_target.approach_height = 0.12;  % stand off this far above the box
P.box_target.grasp_offset_z = -0.010; % TCP slightly below the box centre
P.box_target.confidence = 0.95;

% =====================================================================
%  The table and the boxes
% =====================================================================
P.world.table_size   = [1.0 1.0 0.025];
P.world.table_center = [0.35 0 0.7375];      % top surface at z = 0.75
% Boxes, in the BASE frame: [x y z] of the centre, edge length, mass, colour.
P.world.boxes = struct( ...
    'name',  {'red', 'green', 'blue', 'yellow', 'orange', 'purple'}, ...
    'xyz',   {[0.50 -0.15 0.026], [0.55 0.10 0.026], [0.43 0.22 0.026], ...
              [0.60 -0.05 0.026], [0.45 -0.28 0.0235], [0.65 0.18 0.0285]}, ...
    'size',  {0.050, 0.050, 0.050, 0.050, 0.045, 0.055}, ...
    'mass',  {0.15, 0.15, 0.15, 0.15, 0.12, 0.20}, ...
    'color', {[0.85 0.15 0.15], [0.15 0.70 0.25], [0.20 0.35 0.85], ...
              [0.95 0.80 0.15], [0.95 0.50 0.10], [0.55 0.25 0.70]});
P.world.default_box = 'blue';

% Contact stiffness and friction are chosen, not measured, so forces from
% this simulation do not predict hardware.
P.contact.stiffness = 1e5;            % N/m
P.contact.damping = 300;              % N/(m/s)
P.contact.transition = 1e-4;          % m
P.contact.mu_static = 1.1;
P.contact.mu_kinetic = 0.9;
P.contact.critical_velocity = 1e-3;   % m/s

% `tops grasp bump`: a sideways push on the hand while it travels. It ramps
% up, so the monitor passes through warn before it stops the arm.
P.push.force = [0 -12 0];             % N, world frame
P.push.start = 1.5;                   % s, mid-way through the move
P.push.rise = 0.3;                    % s
P.push.hold = 0.2;                    % s

% =====================================================================
%  Simulation
% =====================================================================
% Variable-step daessc with steps of at most 1 ms. Fixed-step solvers were
% tried first: ode14x launched the box on contact, and ode1be reported
% 13 N on a 150 g box being lifted. daessc reads 1.47 N for the 1.47 N box.
P.sim.max_step = 1e-3;                % s
P.sim.timeout = 45;                   % s of simulated time
P.sim.log_dt = 0.01;                  % logged signals at 100 Hz
P.sim.contact_threshold = 1.0;        % N: what counts as "in contact" in the summary
end
