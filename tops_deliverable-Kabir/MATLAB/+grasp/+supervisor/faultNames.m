function [text, names] = faultNames(bits)
%FAULTNAMES  Fault bitmask -> readable fault names.
%
%   text = grasp.supervisor.faultNames(bits)     % e.g. 'safety_stop;no_target'

names = {'safety_stop', 'no_target', 'home_timeout', 'move_timeout', ...
         'descend_timeout', 'close_timeout', 'lift_timeout', ...
         'box_dropped', 'low_confidence', 'run_timeout'};
on = false(1, numel(names));
for k = 1:numel(names)
    on(k) = bitand(uint32(bits), uint32(2^(k - 1))) > 0;
end
text = strjoin(names(on), ';');
end
