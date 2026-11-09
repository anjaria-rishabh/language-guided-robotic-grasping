function Run = thisGraspSettings(P, colour, pushArm)
%THISGRASPSETTINGS  What differs from one run to the next, as numbers.
%
%   Run = grasp.sim.thisGraspSettings(P, colour, pushArm)
%
%   colour   'red', 'green', 'blue', 'yellow', 'orange' or 'purple'
%   pushArm  true for `tops grasp bump`
%
%   The model has six box slots. Slot 1 is always the box being picked up
%   and slots 2-6 are the rest, so choosing a colour needs no rebuild.

boxes = P.world.boxes;
target = find(strcmpi({boxes.name}, colour));
if isempty(target)
    error('tops:colour', 'no %s box; the colours are: %s', colour, strjoin({boxes.name}, ', '));
end
order = [target, setdiff(1:numel(boxes), target)];
boxes = boxes(order);

Run.box_world = vertcat(boxes.xyz) + P.robot.base_in_world;
Run.box_size  = [boxes.size].';
Run.box_mass  = [boxes.mass].';
Run.box_color = vertcat(boxes.color);
Run.box_base  = boxes(1).xyz;
Run.truth_grasp = boxes(1).xyz + [0 0 P.box_target.grasp_offset_z];
Run.timeout = P.sim.timeout;

% The push on the hand, as a time profile. All zeros unless asked for.
p = P.push;
Run.push_time = [0, p.start, p.start + p.rise, p.start + p.rise + p.hold, ...
                 p.start + 2 * p.rise + p.hold, P.sim.timeout + 1];
Run.push_size = double(pushArm) * [0 0 1 1 0 0];
Run.push_force = p.force;
end
