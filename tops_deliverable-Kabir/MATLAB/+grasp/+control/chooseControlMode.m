function mode = chooseControlMode(phase)
%CHOOSECONTROLMODE  Which control mode each grasp phase uses.
%
%   mode = grasp.control.chooseControlMode(phase)
%
%   phase  grasp phase code (grasp.supervisor.phaseNames)
%   mode   1 MOVING      admittance, high stiffness
%          2 DESCENDING  impedance, low stiffness, contact force regulated
%          3 GRIPPING    impedance, low stiffness, no force regulation
%
%   Admittance tracks accurately in free space but can go unstable against
%   hard contact; impedance is the opposite. So travel uses admittance and
%   anything near contact uses impedance.
%
%   Lift and hold stay on impedance even though the arm moves: admittance
%   would read the box's weight as a push and sag.

switch phase
    case {0, 1, 2}                    % WAIT, GO_HOME, MOVE_ABOVE_BOX
        mode = 1;
    case 3                            % DESCEND
        mode = 2;
    otherwise                         % CLOSE_FINGERS, LIFT, HOLD, RETREAT, STOP
        mode = 3;
end
end
