function convertMeshes(srcVisual, srcTextures, outDir)
%CONVERTMESHES  Turn the Panda's textured COLLADA meshes into OBJ + MTL.
%
%   grasp.robot.convertMeshes(srcVisual, srcTextures, outDir)
%
%   srcVisual    folder holding link0.dae ... link7.dae, hand.dae, finger.dae
%   srcTextures  folder holding <name>_<name>_BaseColor.png for each mesh
%   outDir       where <name>.obj, <name>.mtl and the textures are written
%
%   Why: Simscape Multibody's File Solid reads OBJ (with its material and
%   texture) but not COLLADA. The Panda meshes come from the Gazebo Fuel
%   model "Panda with Ignition position controller model" (CC-BY-4.0), whose
%   DAE files carry UV coordinates for the orange PBR paint. This keeps the
%   geometry and the UVs exactly, and binds the BaseColor texture in the
%   MTL file, so the robot looks the same here as it did in Gazebo.
%
%   The conversion is run once (it is the provenance of robot/meshes/visual)
%   and does not need to be repeated unless the source meshes change:
%
%       grasp.robot.convertMeshes( ...
%           '../ROS/src/tops_description/meshes/visual', ...
%           '../ROS/src/tops_description/meshes/textures', ...
%           'robot/meshes/visual')
%
%   Geometry handling, following the COLLADA spec:
%     * <asset><unit meter="..."/> scales every coordinate to metres
%       (these files are in centimetres);
%     * the node's 4x4 <matrix> (written row by row in the file) places
%       the mesh in the link frame;
%     * <triangles> lists, per corner, one index per <input>, interleaved
%       with the stride given by the largest offset + 1.

names = {'link0','link1','link2','link3','link4','link5','link6','link7','hand','finger'};
if ~isfolder(outDir), mkdir(outDir); end

for k = 1:numel(names)
    name = names{k};
    daePath = fullfile(srcVisual, [name '.dae']);
    texName = sprintf('%s_%s_BaseColor.png', name, name);

    [P, UV, F] = readDae(daePath);

    % --- the material file: white diffuse, multiplied by the texture -------
    fid = fopen(fullfile(outDir, [name '.mtl']), 'w');
    fprintf(fid, '# %s: Franka Emika Panda %s, base-colour texture\n', name, name);
    fprintf(fid, 'newmtl paint\nKa 1 1 1\nKd 1 1 1\nKs 0.3 0.3 0.3\nNs 40\nmap_Kd %s\n', texName);
    fclose(fid);

    % --- the mesh: positions, texture coordinates, faces (1-based) ---------
    fid = fopen(fullfile(outDir, [name '.obj']), 'w');
    fprintf(fid, '# %s: converted from %s by grasp.robot.convertMeshes\n', name, [name '.dae']);
    fprintf(fid, '# units: metres, in the Panda %s link frame\n', name);
    fprintf(fid, 'mtllib %s.mtl\n', name);
    fprintf(fid, 'v %.6f %.6f %.6f\n', P.');
    fprintf(fid, 'vt %.6f %.6f\n', UV.');
    fprintf(fid, 'usemtl paint\n');
    fprintf(fid, 'f %d/%d %d/%d %d/%d\n', F.');   % F rows: v1 vt1 v2 vt2 v3 vt3
    fclose(fid);

    copyfile(fullfile(srcTextures, texName), fullfile(outDir, texName));
    fprintf('%-7s %6d vertices %6d triangles\n', name, size(P,1), size(F,1));
end
end

% =========================================================================
function [P, UV, F] = readDae(path)
% Positions (metres, link frame), UVs, and faces as [v vt v vt v vt] rows.
doc = xmlread(path);

unitNode = doc.getElementsByTagName('unit').item(0);
scale = 1.0;
if ~isempty(unitNode)
    scale = str2double(char(unitNode.getAttribute('meter')));
end

% id -> numeric array, for every <source> holding a <float_array>
sources = containers.Map();
srcList = doc.getElementsByTagName('source');
for i = 0:srcList.getLength-1
    s = srcList.item(i);
    fa = s.getElementsByTagName('float_array').item(0);
    if ~isempty(fa)
        sources(char(s.getAttribute('id'))) = sscanf(char(fa.getTextContent), '%f');
    end
end

% <vertices id="X"><input semantic="POSITION" source="#Y"/> : X -> Y
vertexAlias = containers.Map();
vList = doc.getElementsByTagName('vertices');
for i = 0:vList.getLength-1
    v = vList.item(i);
    in = v.getElementsByTagName('input').item(0);
    vertexAlias(char(v.getAttribute('id'))) = strip(char(in.getAttribute('source')), 'left', '#');
end

% Node transform (identity if absent)
M = eye(4);
mList = doc.getElementsByTagName('matrix');
if mList.getLength > 0
    M = reshape(sscanf(char(mList.item(0).getTextContent), '%f'), 4, 4).';
end

tri = doc.getElementsByTagName('triangles').item(0);
inputs = tri.getElementsByTagName('input');
stride = 0; vOff = 0; tOff = 0; vSrc = ''; tSrc = '';
for i = 0:inputs.getLength-1
    in = inputs.item(i);
    off = str2double(char(in.getAttribute('offset')));
    stride = max(stride, off + 1);
    src = strip(char(in.getAttribute('source')), 'left', '#');
    switch char(in.getAttribute('semantic'))
        case 'VERTEX',   vOff = off; vSrc = vertexAlias(src);
        case 'TEXCOORD', tOff = off; tSrc = src;
    end
end
idx = sscanf(char(tri.getElementsByTagName('p').item(0).getTextContent), '%d');
idx = reshape(idx, stride, []).';               % one row per triangle corner

P = reshape(sources(vSrc), 3, []).';
P = (M * [P, ones(size(P,1),1)].').';
P = P(:, 1:3) * scale;
UV = reshape(sources(tSrc), 2, []).';

corner = [idx(:, vOff+1), idx(:, tOff+1)] + 1;  % 1-based [v vt] per corner
corner = reshape(corner.', 6, []).';            % three corners per row
F = corner;
end
