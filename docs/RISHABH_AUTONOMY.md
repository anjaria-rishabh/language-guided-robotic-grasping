# IIT Delhi TOPS 2026 — Rishabh's Autonomy Stack

**Project:** Language-Guided Robotic Grasping via Perception-Driven Visual Servoing and Compliant Control

**Ownership:** Rishabh: language grounding, RGB-D/TF localization, MoveIt 2 approach and PBVS. Kabir: dynamics, compliance, wrench/force regulation. Shared: supervisory FSM and integration.

**Status:** Source code generated and offline unit-tested; **not yet built or operated in ROS 2/Gazebo/MoveIt**, and **not merged with Kabir's upstream**.

## Contents

- `src/tops_autonomy`: ROS 2 `ament_python` nodes
  - `perception_node`: language query + RGB-D target localization, TF at image stamp
  - `ee_tf_node`: live Panda end-effector TF → timestamp-preserved EE PoseStamped (check robot tip link)
  - `pbvs_node`: bounded Cartesian twist *reference* only (does not actuate)
  - `contract_bridge`: opt-in, runtime schema-checked upstream message bridge
  - `grounders.py`: HSV red/green/blue/yellow demo and optional pretrained OWL-ViT detection
  - `geometry.py` / `depth.py`: independently testable algorithms
- `src/tops_moveit_planner`: `ament_cmake` MoveIt 2 C++ pregrasp planner, explicit Trigger service, dry-run by default
- `benchmark/analyze_trials.py`: summary + Wilson 95% interval of measured trials
- `docs/HANDOFF.md`: Kabir integration agreement/questions to resolve

## Required system

ROS 2 Humble/Jazzy on Ubuntu (distro-specific support of MoveIt packages must be checked), Gazebo Panda with registered RGB and depth streams, `robot_state_publisher`/TF2, MoveIt 2 `move_group` and matching Panda URDF/SRDF controllers. The names in `config/autonomy.yaml` are **proposed defaults**, not verified existing topic names. For `tops_msgs/GraspTarget`, the actual upstream schema is **unverified** in this environment. No commands are sent to Kabir's controllers without an explicit contract agreement.

### Install and build (in your existing ROS workspace)

```bash
# Copy the two directories into your existing repo's ROS workspace src/ folder.
# Example with this archive extracted into ~/tops_ws:
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/tops_ws
sudo apt update
sudo apt install python3-opencv python3-numpy ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-message-filters ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-moveit
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select tops_autonomy tops_moveit_planner
source install/setup.bash
```

> The demo color detector is dependency-light and **does not meet the open-vocabulary objective**. For a pretrained language-conditioned detector, additionally install PyTorch, Transformers and Pillow in a ROS-compatible Python environment, switch `grounder` from `color` to `zero_shot`, and allow pretrained model weights to download. Check deployment licenses, compute and inference latency independently. CPU model inference may be too slow for high-rate tracking; this implementation does not promise 30 Hz neural detection.

### 1. Confirm camera/TF system in Kabir's simulated scene

```bash
ros2 topic list | grep -E 'camera|image|depth|joint|tops'
ros2 topic echo --once /camera/camera_info
ros2 run tf2_ros tf2_echo panda_link0 CAMERA_OPTICAL_FRAME
ros2 interface show tops_msgs/msg/GraspTarget
ros2 control list_controllers
```

Verify RGB and depth **same resolution + aligned optical frame**. Update `src/tops_autonomy/config/autonomy.yaml` with the real names. Do not assume TF or camera calibration automatically exists. Camera is currently assumed **registered pinhole, rectified**; distortion and extrinsic-calibration drift are *not corrected* by this code.

### 2. Start *passive* autonomy

```bash
# With Kabir's Gazebo/robot_state_publisher running and use_sim_time active:
ros2 launch tops_autonomy autonomy.launch.py
ros2 topic pub --once /tops/autonomy/instruction std_msgs/msg/String \
  "{data: 'pick up the red cube'}"
ros2 topic echo /tops/autonomy/detection
ros2 topic echo /tops/autonomy/target_pose
```

The default detector processes colored tabletop targets, publishes a stamped **3-D centroid with identity orientation placeholder**, and fails closed on missing depth, stale images, mismatched image frames, or unavailable TF at measurement time. For image camera intrinsics to make sense, depth must be aligned to color. The `PoseStamped` orientation **does not describe grasp pose**.

### 3. MoveIt 2 pregrasp (opt-in)

Start Kabir's configured MoveIt 2 `move_group` and Panda robot state publisher; confirm the actual MoveIt planning group (default `panda_arm`). In a new shell:

```bash
ros2 run tops_moveit_planner pregrasp_planner --ros-args \
  -p use_sim_time:=true -p planning_group:=panda_arm \
  -p dry_run:=true -p allow_execute:=false
ros2 service call /tops/autonomy/plan_pregrasp std_srvs/srv/Trigger "{}"
```

This attempts collision-aware planning to `(target_x,target_y,target_z+0.12m)` with a **configured** downward gripper quaternion. You must calibrate approach offsets, gripper orientation and table geometry for your actual simulation. **Table collision geometry defaults OFF** because the correct table location is not supplied. An unmodeled obstacle is not collision checked. To configure it, use ROS params `-p add_table_collision:=true -p table_top_z_m:=...` and calibrated table dimensions.

To allow physical/simulated trajectory execution later, BOTH `dry_run:=false` and `allow_execute:=true` are required, and the joint supervisor must first guarantee that Kabir's compatible trajectory controller owns the arm. Do not enable before integration review.

### 4. PBVS reference (opt-in, non-actuating)

The PBVS node subscribes to `/tops/autonomy/target_pose` (base frame), `/tops/autonomy/ee_pose` (fresh measured end-effector `PoseStamped` in base frame), and `/tops/autonomy/mode` (`std_msgs/String`, value `SERVO`), and publishes bounded `/tops/autonomy/pbvs_twist` (`TwistStamped`, **base frame**). The included `ee_tf_node` provides EE pose from `/tf` using the configured `ee_frame` (default `panda_hand`, unverified). The mode still requires a bridge from Kabir's actual FSM. Default `enabled:=false` publishes zero reference. There is **no Jacobian/servo-controller adapter** included: forwarding Cartesian twist directly into an effort/trajectory controller would be incorrect. Inverse kinematics and controller ownership must be provided using the agreed robot interface.

Before enabling PBVS: validate EE pose publisher, servo-mode guard, 300 ms freshness limit, gripper orientation, velocity/acceleration limits, workspace, collision monitoring and operator E-stop. Zero reference on invalid observations is **not a certified safety stop**; enforce hardware/simulation watchdog downstream.

### 5. Verify optional `tops_msgs/GraspTarget` contract

The bridge requires the actual installed message definition and a field of type `geometry_msgs/PoseStamped`. Example **only if** the message has a `target_pose` PoseStamped member:

```bash
ros2 run tops_autonomy contract_bridge --ros-args -p enabled:=true \
  -p pose_field:=target_pose
```

If it does not have that member/type, the bridge intentionally refuses to start. Inspect the schema, then extend its adapter to populate all required semantics/validity fields. Do not run Kabir's fake perception publisher at the same time as a real bridge on the same topic. In the current form, only the pose field is mapped, so **supervisor compatibility is still pending** even if this passes field checks.

### 6. Trial analysis

The benchmark CSV *schema* is `benchmark/SCHEMA.csv` and contains **zero fabricated measurements**. From collected data:

```bash
python3 benchmark/analyze_trials.py path/to/measured_trials.csv
```

Includes per-condition success rate, a Wilson 95% confidence interval, errors, force peaks and time thresholds. Full automated 120-run Gazebo resets, perturbation injection, gripper closure and ground-truth measurement depend on Kabir's scene/supervisor interfaces and are **not included/claimed**.

### 7. Offline tests (do not require ROS)

```bash
python3 -m pip install -U pytest numpy opencv-python
PYTHONPATH=src/tops_autonomy python3 -m pytest -q src/tops_autonomy/test benchmark/test_benchmark.py
python3 -m compileall -q src/tops_autonomy/tops_autonomy benchmark
```

## Limitations relevant to supervisory review

- No evidence of ≤2 mm localization, ≤3 mm grasp error, ≤10 N peak contact, 15 s execution or ≥85% success without measured validation.
- A bounding box + depth gives a **surface/centroid point estimate**, not full 6-DOF object pose or antipodal gripper contacts.
- Missing dynamic collision scene updates for detected movable objects; only optional calibrated static table.
- No actual 1 kHz torque control written: Kabir's low-level controller remains its owner.
- ROS/Gazebo/MoveIt live integration and `GraspTarget` field mapping require checking the real repository/source.
- Pretrained language grounding is optional and requires separate model dependencies/inference benchmarking.
- Simulator-only development, not a hardware safety-certified system.
