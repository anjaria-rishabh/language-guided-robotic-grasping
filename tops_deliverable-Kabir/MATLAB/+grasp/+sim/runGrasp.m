function [result, log] = runGrasp(P, colour, pushArm, mdl)
%RUNGRASP  Simulate one grasp and measure it.
%
%   [result, log] = grasp.sim.runGrasp(P, colour, pushArm, mdl)
%
%   colour   which box to pick up
%   pushArm  true to push the arm mid-move
%   mdl      the model to run (grasp.sim.buildArmModel)
%   result   summary numbers (grasp.sim.measureGrasp)
%   log      the 100 Hz signals, by name

[C, U, Q, km] = grasp.config.controllerSettings(P);
Run = grasp.sim.thisGraspSettings(P, colour, pushArm);

in = Simulink.SimulationInput(mdl);
in = in.setVariable('C', C).setVariable('U', U).setVariable('Q', Q) ...
       .setVariable('km', km).setVariable('Run', Run);
out = sim(in);
log = namedSignals(squeeze(out.runLog).');
result = grasp.sim.measureGrasp(P, colour, pushArm, log);
end

% =========================================================================
function log = namedSignals(X)
% The columns the model logs (grasp.sim.buildArmModel), by name.
log.t = X(:, 1);
log.supervisor = X(:, 2:15);        % layout in grasp.supervisor.superviseGrasp
log.controller = X(:, 16:35);       % layout in grasp.control.controlArm
log.box_xyz_world = X(:, 36:38);
log.contact_forces = X(:, 39:45);   % order in grasp.sim.buildArmModel, "contacts"
log.fingers = X(:, 46:47);
log.phase = log.supervisor(:, 1);
end
