function [C, U, Q, km] = controllerSettings(P)
%CONTROLLERSETTINGS  The plain-number structs the Simulink blocks read.
%
%   [C, U, Q, km] = grasp.config.controllerSettings(P)
%
%   P   grasp.config.settings()
%   C   arm controller (1 kHz)     U   grasp phases (50 Hz)
%   Q   box target (50 Hz)         km  arm model for the controller
%
%   Compiled blocks cannot hold strings or cell arrays, so this flattens
%   grasp.config.settings into doubles. It invents no numbers.

km = grasp.robot.armModelForController(P);
R = P.robot;  c = P.ctrl;  s = P.sensor;

% ---- arm -------------------------------------------------------------
C.dt = c.dt;
C.q_home = R.q_home;
C.q_lower = R.q_lower;  C.q_upper = R.q_upper;
C.torque_limits = c.torque_limits;
C.torque_rate_limit = c.torque_rate_limit;
C.singularity_damping = c.singularity_damping;
C.posture_stiffness = c.posture_stiffness;
C.posture_damping_ratio = c.posture_damping_ratio;
C.joint_limit_margin = c.joint_limit_margin;
C.joint_limit_stiffness = c.joint_limit_stiffness;
C.max_target_jump = c.max_target_jump;
C.damping_ratio = c.damping_ratio;
C.retreat_distance = P.phase.retreat_distance;
C.retreat_speed = P.phase.retreat_speed;

% ---- force sensor ----------------------------------------------------
% First-order low-pass y += alpha (x - y), alpha = dt / (tau + dt),
% tau = 1 / (2 pi fc).
lp = @(fc) c.dt / (1 / (2 * pi * fc) + c.dt);
C.alpha_control = lp(s.cutoff_control);
C.alpha_safety = lp(s.cutoff_safety);
C.alpha_accel = lp(s.cutoff_accel);
C.sensor_deadband = s.deadband;
C.sensor_clamp = s.clamp;
C.zeroing_samples = s.zeroing_samples;
C.tool_mass = s.tool_mass;
C.tool_com = s.tool_com;

% ---- stiffness schedule ----------------------------------------------
C.K_moving = P.stiffness.moving;
C.K_gripping = P.stiffness.gripping;
C.K_step = abs(P.stiffness.moving - P.stiffness.gripping) * c.dt / P.stiffness.change_time;
C.K_stop_step = P.stiffness.moving * c.dt / P.stiffness.stop_time;

% ---- admittance ------------------------------------------------------
A = P.admittance;
C.adm_mass = A.mass;  C.adm_damping = A.damping;  C.adm_stiffness = A.stiffness;
C.adm_max_offset = A.max_offset;  C.adm_max_speed = A.max_speed;

% ---- contact force regulation ----------------------------------------
F = P.force;
C.force_axis_tool = F.axis_in_tool;  C.force_target = F.target;
C.force_contact_threshold = F.contact_threshold;
C.force_kp = F.kp;  C.force_ki = F.ki;
C.force_integral_limit = F.integral_limit;
C.force_axis_stiffness_scale = F.axis_stiffness_scale;

% ---- wrench safety ---------------------------------------------------
C.safety = P.safety;

% ---- gripper ---------------------------------------------------------
C.finger_kp = R.finger_kp;  C.finger_kd = R.finger_kd;
C.finger_force_max = R.finger_force_max;
C.finger_speed = R.finger_speed;
C.gripper_open = P.phase.gripper_open;

% ---- grasp phases ----------------------------------------------------
U = P.phase;
U.q_home = R.q_home;
U.timeout = P.sim.timeout;
U.warn_speed_scale = P.safety.warn_speed_scale;

% ---- box target ------------------------------------------------------
Q = P.box_target;
end
