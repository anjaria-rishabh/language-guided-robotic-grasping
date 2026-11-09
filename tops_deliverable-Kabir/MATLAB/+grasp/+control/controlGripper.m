function [fFinger, S] = controlGripper(S, gripCmd, qf, qfd, C)
%CONTROLGRIPPER  Speed-limited, force-limited fingers, like the real hand.
%
%   [fFinger, S] = grasp.control.controlGripper(S, gripCmd, qf, qfd, C)
%
%   gripCmd  commanded opening per finger, m      qf, qfd  2x1 finger state
%
%   The opening moves toward the request at the hand's finger speed. A jump
%   made the fingers slam shut, and the first to arrive shoved the box
%   before the other got there. The force clamp is what sets the grip force.

step = C.finger_speed * C.dt;
S.gripRef = S.gripRef + min(max(gripCmd - S.gripRef, -step), step);
fFinger = C.finger_kp * (S.gripRef - qf) - C.finger_kd * qfd;
fFinger = min(max(fFinger, -C.finger_force_max), C.finger_force_max);
end
