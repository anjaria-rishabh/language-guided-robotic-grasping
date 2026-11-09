function meshesAttribution()
%MESHESATTRIBUTION  Where every mesh file in robot/meshes/ came from.
%
%   meshesAttribution       % prints this file
%
%   robot/meshes/visual/  link0..link7, hand, finger (.obj + .mtl + _BaseColor.png)
%       From the Gazebo Fuel model "Panda with Ignition position controller
%       model", version 5, by OpenRobotics (fuel.gazebosim.org), itself
%       copied from AndrejOrsula's Fuel "panda", which re-exported Franka's
%       meshes from Blender with UV coordinates for an orange PBR paint.
%       Licence: Creative Commons Attribution 4.0 International
%       (creativecommons.org/licenses/by/4.0).
%       Modified: converted from COLLADA to OBJ by grasp.robot.convertMeshes
%       (geometry and UVs unchanged; the base-colour texture bound in the
%       .mtl file). Checked to sit on Franka's link frames to the millimetre.
%
%   robot/meshes/collision/  link0..link7, hand (.stl)
%       Franka Robotics' own collision meshes from franka_description
%       (commit 7aeeddc, meshes/robots/fer/collision/). Apache-2.0: see
%       frankaLicense.m.
%
%   The fingers have no collision mesh: like Franka, the model uses four
%   boxes per finger for contact (mesh-on-box contact chatters).

type([mfilename('fullpath') '.m']);
end
