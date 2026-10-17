# Panda meshes: where every file came from

This folder is the real Franka Emika Panda geometry. Nothing in it is
hand-made. The robot's *numbers* (kinematics, limits, inertials, joint
dynamics) are in `../config/franka/`, also copied verbatim from Franka.

| Folder | Contents | Source | Licence |
|---|---|---|---|
| `visual/` | `link0`–`link7`, `hand`, `finger` `.dae` | Gazebo Fuel model **"Panda with Ignition position controller model"** v5, by OpenRobotics (itself copied from AndrejOrsula's Fuel `panda`), which re-exports Franka's meshes from Blender with UV coordinates so they can carry textures. This is the same textured orange Panda that `gz sim` loads in its *panda joint control world* example. | CC-BY-4.0 |
| `textures/` | `*_BaseColor`, `*_Normal`, `*_Roughness`, `*_Metallic` PNGs | the same Fuel model (its `materials/textures/`) | CC-BY-4.0 |
| `collision/` | `link0`–`link7`, `hand` `.stl` | Franka Robotics' official `franka_description` (github.com/frankarobotics/franka_description, commit `7aeeddc`, `meshes/robots/fer/collision/`) | Apache-2.0, see `LICENSE-franka_description-Apache-2.0` |

The finger has no collision mesh on purpose. Franka models each finger's
collision as four boxes, and so does `../urdf/panda_hand.xacro`.

## Checks done when these were imported

* **Same frames as Franka's own meshes.** Every visual mesh's bounding box
  was compared with the corresponding mesh in `franka_description`. They
  agree to the millimetre for every link, so the textured visual sits
  exactly on the official kinematic frame and collision mesh.
* **Two edits to the Fuel files, both visual only:**
  1. The duplicate upper-case `link3/5/6/7.DAE` files in the Fuel model
     were not copied. The SDF never referenced them.
  2. Each `.dae` had only a flat grey material. Its `BaseColor` PNG was
     bound as the COLLADA diffuse texture, so RViz and MoveIt (which ignore
     Gazebo's PBR tags) show the same orange paint as Gazebo. Gazebo itself
     uses the full PBR set from the `<gazebo>` blocks in
     `../urdf/panda_arm.xacro`, which override the mesh material.
     The edit was made with `../scripts/bind_dae_textures.py`.
* The Fuel model's reflection cube-map (`DepotEnvMap.dds`, 97 MB, fetched
  from the internet at load time) was **not** carried over. Gazebo falls
  back to its own scene lighting, so loading needs no network access.

## Attribution (CC-BY-4.0)

> "Panda with Ignition position controller model" by OpenRobotics, from
> Gazebo Fuel (fuel.gazebosim.org), licensed under CC BY 4.0
> (creativecommons.org/licenses/by/4.0). Modified: duplicate files
> dropped; base-colour texture bound inside each COLLADA file.
