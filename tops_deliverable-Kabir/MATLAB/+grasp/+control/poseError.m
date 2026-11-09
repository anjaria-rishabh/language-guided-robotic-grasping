function e = poseError(pDes, qDes, p, Rtcp)
%POSEERROR  [position error; orientation error as axis*angle], in the base frame.
%
%   e = grasp.control.poseError(pDes, qDes, p, Rtcp)
%
%   Rotations do not subtract, so the orientation error is the rotation
%   taking the current orientation to the desired one.

qc = grasp.control.rotationToQuaternion(Rtcp);
if dot(qc, qDes) < 0, qc = -qc; end                    % same hemisphere
qe = quatMultiply([qc(1); -qc(2:4)], qDes);            % qc^-1 * qDes (body frame)
s = norm(qe(2:4));
if s < 1e-12
    axisAngle = [0; 0; 0];
else
    axisAngle = qe(2:4) / s * 2 * atan2(s, qe(1));
end
e = [pDes - p; Rtcp * axisAngle];
end

function c = quatMultiply(a, b)
c = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
     a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
     a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
     a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
end
