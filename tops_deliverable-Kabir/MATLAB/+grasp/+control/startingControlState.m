function S = startingControlState(C)
%STARTINGCONTROLSTATE  The controller's state at the first cycle.
%
%   S = grasp.control.startingControlState(C)
%
%   Every field has a fixed size from the start, which is what lets the
%   state live in a compiled Simulink block.

S.initialised = false;
S.pDes = zeros(3, 1);              % accepted target position (base)
S.qDes = [1; 0; 0; 0];             % accepted target orientation (w, x, y, z)
S.tauPrev = zeros(7, 1);           % for the torque rate limit
S.gripRef = C.gripper_open;        % commanded finger opening, slewed

% Retreat and safety stop
S.retreatId = 0;
S.retreating = false;
S.retreatGoal = zeros(3, 1);
S.latched = false;
S.stopReason = 0;

% Force sensor
S.vPrev = zeros(3, 1);
S.acc = zeros(3, 1);
S.zeroSum = zeros(6, 1);
S.zeroCount = 0;
S.zero = zeros(6, 1);
S.zeroed = false;
S.wControl = zeros(6, 1);
S.wSafety = zeros(6, 1);
S.wRawPrev = zeros(6, 1);
S.frozenCount = 0;

% Stiffness in effect: starts stiff, holding the home pose
S.K = C.K_moving(:);

% Admittance: offset of the reference from the commanded pose
S.xa = zeros(6, 1);
S.va = zeros(6, 1);

% Contact force regulation
S.forceI = 0;
end
