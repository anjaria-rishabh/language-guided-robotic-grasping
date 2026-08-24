# Language-Guided Robotic Grasping

ROS 2 based framework for language-guided robotic grasping using RGB-D perception, visual servoing, MoveIt 2 planning, and compliant control.

## Project Overview

This project develops a perception-driven manipulation framework where a robot receives a natural language command, identifies the target object, estimates its 3D pose using RGB-D perception, plans a collision-free trajectory, and executes grasping using visual servoing and compliant control.

The system combines high-level perception-driven autonomy with low-level compliant interaction to improve robustness against localization errors, sensor noise, and physical contact uncertainties.

## System Architecture


Language Command
|
v
Target Grounding
|
v
RGB-D 3D Localization
|
v
MoveIt 2 Planning
|
v
Supervisory FSM
|
+----------------+
| |
v v
Visual Servoing Compliant Control
|
v
Robot Execution


## Project Objectives

The main objective is to develop a modular ROS 2 manipulation framework capable of:

- Understanding natural language object commands.
- Localizing target objects using RGB-D perception.
- Planning collision-free robot trajectories using MoveIt 2.
- Performing precise visual servoing during approach.
- Handling physical interaction using compliant control.
- Integrating all modules through a supervisory finite state machine.

## Current Status

Repository initialization phase.

Current focus:
- System architecture definition.
- Software module organization.
- ROS 2 framework planning.
- Interface definition between perception, planning, and control modules.

## Contributors

- Rishabh Anjaria
- Kabir Dhingra

## Main Components

### Perception Pipeline

Responsible for converting human commands and visual information into actionable robot targets.

Components:

- Language-based target grounding.
- RGB-D object localization.
- 3D coordinate extraction.
- Coordinate transformations using tf2.

### Motion Planning

Responsible for generating safe robot motion.

Components:

- MoveIt 2 configuration.
- Collision-free joint-space planning.
- Manipulator trajectory generation.

### Visual Servoing

Responsible for precise object approach using visual feedback.

Components:

- Position-Based Visual Servoing (PBVS).
- Error computation between current and desired object pose.
- Velocity command generation with safety limits.

### Compliant Control

Responsible for safe physical interaction.

Components:

- Impedance/admittance control integration.
- Contact force regulation.
- Wrench-based safety monitoring.

### Task Executive

Responsible for overall system coordination.

Components:

- Supervisory finite state machine.
- Mode switching logic.
- Safety transitions between motion, servoing, compliance, hold, retreat, and stop states.

## Repository Structure


language-guided-robotic-grasping/

├── README.md

├── docs/
│ ├── architecture.md
│ ├── pbvs.md
│ ├── fsm.md
│ └── milestones.md

├── src/
│ ├── target_grounding/
│ ├── rgbd_localization/
│ ├── moveit_config/
│ ├── pbvs_controller/
│ ├── fsm/
│ └── integration/

├── launch/

├── config/

├── urdf/

├── worlds/

└── results/


## Future Development

Planned implementation stages:

1. ROS 2 environment setup and simulation configuration.
2. Perception pipeline development.
3. RGB-D localization and transformation pipeline.
4. MoveIt 2 integration.
5. PBVS controller implementation.
6. FSM-based system integration.
7. Simulation benchmarking and evaluation.