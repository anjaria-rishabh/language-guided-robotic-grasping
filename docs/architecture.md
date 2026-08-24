# System Architecture

## Overview

The project follows a modular ROS 2 architecture for language-guided robotic grasping.

The system receives a natural language command, identifies the target object, estimates its 3D position using RGB-D perception, plans a collision-free trajectory using MoveIt 2, and executes manipulation using visual servoing and compliant control.

The architecture separates high-level perception-driven autonomy from low-level compliant interaction to improve robustness against localization errors, sensor noise, and physical contact uncertainties.

## High-Level Pipeline

```text
Language Command
        |
        v
Target Grounding Node
        |
        v
RGB-D Localization Node
        |
        v
3D Target Pose Estimation
        |
        v
MoveIt 2 Motion Planning
        |
        v
Supervisory FSM
        |
        +-----------------------+
        |                       |
        v                       v
PBVS Controller        Compliant Controller
(Rishabh)              (Kabir)
        |
        v
Robot Execution
```

## Module Description

## 1. Target Grounding

### Purpose

Convert natural language instructions into visual targets.

### Input

- Human language command

### Output

- Object category
- Candidate bounding box
- Target confidence score

### Responsibilities

- Language understanding
- Object selection
- Perception interface


## 2. RGB-D Localization

### Purpose

Estimate the 3D position and pose of the selected object.

### Input

- RGB image
- Depth image
- Camera parameters

### Output

- Object pose in camera frame
- Transformed target pose

### Responsibilities

- Depth deprojection
- 3D coordinate extraction
- tf2 frame transformations


## 3. MoveIt 2 Planning

### Purpose

Generate collision-free robot motion.

### Input

- Desired target pose

### Output

- Robot trajectory

### Responsibilities

- Motion planning
- Collision checking
- Manipulator configuration


## 4. PBVS Controller

### Purpose

Perform precise object alignment using visual feedback.

### Input

- Current object pose
- Desired object pose

### Output

- Robot velocity commands

### Responsibilities

- Visual error computation
- Velocity generation
- Safety velocity saturation

### Owner

Rishabh Anjaria


## 5. Compliant Controller

### Purpose

Enable safe physical interaction during grasping.

### Input

- Contact information
- Force/torque feedback

### Output

- Safe robot motion response

### Responsibilities

- Impedance/admittance control
- Force regulation
- Contact handling

### Owner

Kabir Dhingra


## 6. Supervisory FSM

### Purpose

Coordinate different robot operating modes.

### States

- Motion Planning
- Visual Servoing
- Compliant Approach
- Hold
- Retreat
- Stop

### Responsibilities

- Mode switching
- Safety transitions
- System coordination

### Ownership

Shared