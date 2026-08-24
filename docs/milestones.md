# Project Milestones

## Overview

The project is divided into modular milestones to enable parallel development of perception, planning, control, and system integration components.

The development follows a ROS 2 based architecture for language-guided robotic grasping using RGB-D perception, MoveIt 2 planning, visual servoing, and compliant control.

---

# Rishabh Anjaria Responsibilities

## M1: Target Grounding

Objective:

Develop the language-to-object perception interface.

Tasks:

- Accept natural language object commands.
- Identify target objects from visual input.
- Generate candidate object bounding regions.
- Provide object information to the localization pipeline.

Deliverables:

- Target grounding node.
- Perception interface.

---

## M2: RGB-D Localization and TF Pipeline

Objective:

Estimate accurate 3D target poses from RGB-D data.

Tasks:

- Process RGB-D sensor information.
- Perform depth deprojection.
- Convert image coordinates into 3D coordinates.
- Implement tf2 coordinate transformations.

Deliverables:

- RGB-D localization node.
- Camera-to-robot frame transformation pipeline.

---

## M3: MoveIt 2 Integration and PBVS

Objective:

Enable precise robot approach using motion planning and visual feedback.

Tasks:

- Configure MoveIt 2 planning environment.
- Generate collision-free trajectories.
- Implement Position-Based Visual Servoing.
- Generate velocity commands with safety limits.

Deliverables:

- MoveIt 2 configuration.
- PBVS controller node.

---

# Kabir Dhingra Responsibilities

## M4: Robot Dynamics and Control Interface

Objective:

Develop the low-level robot control framework.

Tasks:

- Model robot dynamics.
- Configure ros2_control interfaces.
- Implement torque/position control interfaces.

Deliverables:

- Robot control interface.
- Dynamics model.

---

## M5: Compliant Control

Objective:

Enable safe physical interaction during grasping.

Tasks:

- Implement impedance/admittance control.
- Regulate contact forces.
- Handle force/torque feedback.
- Develop safety mechanisms.

Deliverables:

- Compliant controller plugin.
- Force regulation pipeline.

---

# Shared Responsibilities

## M6: Supervisory FSM Integration

Objective:

Coordinate different robot operating modes.

Tasks:

- Design system state machine.
- Define transition conditions.
- Integrate perception, planning, and control modules.

States:

- Planning
- Visual Servoing
- Compliant Approach
- Hold
- Retreat
- Stop

Deliverables:

- Supervisory FSM.
- Module communication interfaces.

---

## M7: Safety and Validation

Objective:

Validate complete grasping pipeline.

Tasks:

- Implement safety transitions.
- Perform simulation experiments.
- Evaluate grasp success and system robustness.
- Generate performance reports.

Deliverables:

- Benchmarking framework.
- Experimental results.

---

# Development Workflow

Phase 1:
- Repository setup
- ROS 2 environment configuration
- Module interface definition

Phase 2:
- Independent module development

Phase 3:
- System integration

Phase 4:
- Simulation testing and validation