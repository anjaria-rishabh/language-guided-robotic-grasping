function boxTarget = knownBoxPose(t, phase, Run, Q)
%KNOWNBOXPOSE  Where the chosen box is. Stands in for a camera.
%
%   boxTarget = grasp.perception.knownBoxPose(t, phase, Run, Q)
%
%   t      simulation time, s
%   phase  grasp phase code, one tick old
%   Run    this run's numbers (grasp.sim.thisGraspSettings)
%   Q      grasp.config.controllerSettings
%
%   boxTarget  9x1 [p(3); quat(4) w,x,y,z; confidence; stamp]
%
%   Two heights only: above the box until the descent starts, then the
%   grasp point. A straight line from home to the grasp point would go
%   through the table.

p = Run.box_base(:);
if phase >= 3 && phase <= 6                     % DESCEND .. HOLD
    p(3) = p(3) + Q.grasp_offset_z;
else
    p(3) = p(3) + Q.approach_height;
end

% Gripper pointing straight down: 180 degrees about the base x axis.
boxTarget = [p; 0; 1; 0; 0; Q.confidence; t];
end
