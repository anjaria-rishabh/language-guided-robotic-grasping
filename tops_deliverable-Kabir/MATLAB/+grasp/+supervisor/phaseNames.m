function names = phaseNames()
%PHASENAMES  Names of the grasp phases. Phase code k is names{k + 1}.
%
%   names = grasp.supervisor.phaseNames()

names = {'WAIT', 'GO_HOME', 'MOVE_ABOVE_BOX', 'DESCEND', 'CLOSE_FINGERS', ...
         'LIFT', 'HOLD', 'RETREAT', 'STOP'};
end
