"""Bind each Panda visual mesh's BaseColor PNG as its COLLADA diffuse texture.

Usage (from tops_description/meshes):  python3 ../scripts/bind_dae_textures.py visual/*.dae
Idempotent: a file that already has a <library_images> block is skipped.

The Fuel meshes carry UVs but only a flat grey lambert; the colour lives in
SDF <pbr> tags that only Gazebo reads. Binding the albedo map inside the DAE
lets assimp-based viewers (RViz, MoveIt) show the same paint. Gazebo is
unaffected: an SDF <material> on the visual overrides the mesh material.
"""
import re, sys, pathlib

for path in map(pathlib.Path, sys.argv[1:]):
    name = path.stem                          # link1, hand, finger ...
    tex = f'../textures/{name}_{name}_BaseColor.png'
    s = path.read_text()
    if '<library_images>' in s:
        print('already patched', path.name); continue
    images = (f'<library_images>\n    <image id="{name}-albedo" name="{name}-albedo">\n'
              f'      <init_from>{tex}</init_from>\n    </image>\n  </library_images>\n  ')
    s = s.replace('<library_effects>', images + '<library_effects>', 1)
    params = (f'<newparam sid="{name}-surface"><surface type="2D">'
              f'<init_from>{name}-albedo</init_from></surface></newparam>\n        '
              f'<newparam sid="{name}-sampler"><sampler2D>'
              f'<source>{name}-surface</source></sampler2D></newparam>\n        ')
    s = s.replace('<profile_COMMON>\n        <technique', '<profile_COMMON>\n        ' + params + '<technique', 1)
    s, n = re.subn(r'<diffuse>\s*<color sid="diffuse">[^<]*</color>\s*</diffuse>',
                   f'<diffuse><texture texture="{name}-sampler" texcoord="UVSET0"/></diffuse>', s, count=1)
    assert n == 1 and params.split()[0] in s, path
    s, n = re.subn(r'<instance_material ([^>]*)/>',
                   r'<instance_material \1><bind_vertex_input semantic="UVSET0" '
                   r'input_semantic="TEXCOORD" input_set="0"/></instance_material>', s, count=1)
    assert n == 1, path
    path.write_text(s)
    print('patched', path.name, '->', tex)
