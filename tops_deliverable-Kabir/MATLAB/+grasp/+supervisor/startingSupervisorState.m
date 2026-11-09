function S = startingSupervisorState()
%STARTINGSUPERVISORSTATE  The supervisor's state at the start of a run.
%
%   S = grasp.supervisor.startingSupervisorState()
%
%   Fixed-size fields only, so it can live in a compiled Simulink block.

S.fsm = struct('state', 0, 'timeInState', 0, 'justEntered', true);
S.started = false;
S.tStart = -1;           % s; -1 = not yet
S.tTerminal = -1;
S.tGrasp = -1;
S.targetErr = NaN;       % m, TCP to the true grasp point when the fingers start closing
S.goal = zeros(3, 1);
S.goalQuat = [0; 1; 0; 0];   % gripper pointing straight down: 180 deg about x
S.haveGoal = false;
S.graspZ = 0;
S.graspXY = zeros(2, 1);
S.graspSet = false;
S.cmdPos = zeros(3, 1);
S.cmdInit = false;
S.retreatId = 0;
S.grip = 0.039;
S.faultsEver = 0;
S.transitions = 0;
end
