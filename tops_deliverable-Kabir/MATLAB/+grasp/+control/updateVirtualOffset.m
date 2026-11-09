function S = updateVirtualOffset(S, force, C)
%UPDATEVIRTUALOFFSET  One step of the admittance mass-spring-damper.
%
%   S = grasp.control.updateVirtualOffset(S, force, C)
%
%   force  6x1 wrench driving the offset (zero lets it fade out)
%
%   M xa'' + D xa' + K xa = force, where xa is how far the reference has
%   been moved away from the commanded pose.

a = (force - C.adm_damping(:) .* S.va - C.adm_stiffness(:) .* S.xa) ./ C.adm_mass(:);
S.va = S.va + a * C.dt;
speed = norm(S.va(1:3));
if speed > C.adm_max_speed
    S.va(1:3) = S.va(1:3) * C.adm_max_speed / speed;
end
S.xa = S.xa + S.va * C.dt;
offset = norm(S.xa(1:3));
if offset > C.adm_max_offset
    S.xa(1:3) = S.xa(1:3) * C.adm_max_offset / offset;
end
end
