# Integration handoff — Rishabh / Kabir — IIT Delhi TOPS 2026

**Project:** Language-Guided Robotic Grasping via Perception-Driven Visual Servoing and Compliant Control. **Supervisor:** Prof. S. K. Saha. **Source baseline:** proposal and second review supplied on 11 Nov 2026.

## Responsibility boundaries

| Area | Owner | This deliverable |
|---|---|---|
| Language grounding | Rishabh | Color demo; optional pretrained language detector |
| RGB-D / TF2 localization | Rishabh | Stamped base-frame target pose |
| MoveIt 2 pregrasp | Rishabh | Opt-in C++ dry-run planner; requires configured MoveIt scene |
| PBVS | Rishabh | Cartesian twist references, safe-zero gating, not connected to actuator |
| Low-level Panda + impedance/admittance | Kabir | Not modified; requires controller access from upstream |
| FSM, handoff, watchdog, force cutoff | Joint | Protocol and test matrix below, actual state-machine integration pending |
| 120 randomized trials, report | Joint | CSV evaluator generated; execution harness requires verified sim reset API |

## Proposed ROS contract — confirm against upstream, do NOT assume

1. `std_msgs/String /tops/autonomy/instruction` — natural language query, e.g. `pick up the red cube`.
2. `/camera/image_raw` RGB and `/camera/depth` registered depth with matching `frame_id`, resolution and image timestamps; `/camera/camera_info` pinhole calibration.
3. `geometry_msgs/PoseStamped /tops/autonomy/target_pose` — measured target **position** in `panda_link0`; orientation identity is **not** a measured grasp orientation.
4. `geometry_msgs/PoseStamped /tops/autonomy/ee_pose` — EE measured pose in base frame; to be provided/bridged from robot state or TF. **Provided by `ee_tf_node` from latest TF if the configured Panda EE frame exists and is fresh**.
5. `std_msgs/String /tops/autonomy/mode` — expected `SERVO` before any PBVS reference becomes nonzero. **Must map from shared FSM**.
6. `geometry_msgs/TwistStamped /tops/autonomy/pbvs_twist` — bounded base-frame Cartesian desired velocity. **Requires an IK/servo adapter** compatible with Kabir's controllers, not a direct torque command.
7. `std_srvs/Trigger /tops/autonomy/plan_pregrasp` — explicit request to plan; actual execution disabled by default.
8. `tops_msgs/GraspTarget /tops/grasp_target` — upstream schema NOT verified. Use `ros2 interface show tops_msgs/msg/GraspTarget`; the provided bridge requires field mapping.

## Decisions needed in joint integration session

- Confirm ROS distro/Gazebo version, Panda URDF/SRDF, MoveIt planning group, EE link, tabletop collision world and camera optical frame.
- Agree precise target pose semantics: object centroid vs object top surface vs gripper contact point, orientation estimator, grasp height offset and uncertainty.
- Inspect `tops_msgs/GraspTarget` fields; map position, frame, timestamps, label, valid/stale flags and confidence without changing Kabir's package.
- Confirm supervisor state names/events, readiness guards, handoff acknowledgments, one-controller-at-a-time ownership, stop/retreat and timeout response.
- Choose Servo implementation (`moveit_servo` Cartesian twist-to-joint commands OR audited Jacobian IK) compatible with controller mode switching. Never publish velocities blindly to torque controllers.
- Implement camera calibration verification, latency accounting, validate the included TF-based end-effector PoseStamped bridge against the actual tool center point, and add detection temporal tracking if dynamic targets are required.
- Agree how Gazebo resets scenes, applies Gaussian depth noise and 10 mm target shift, logs ground truth, captures peak wrench, determines grasp success and measures time.

## Proposed integration acceptance tests (not yet performed)

1. Color demo: multiple red and blue blocks, choose requested color only; missing color yields no target.
2. Synchronization: older frame or stale depth -> no published target; TF lookup at captured image stamp.
3. Calibration: compare deprojection against measured points over workspace; report median, RMSE and max.
4. Pregrasp: table collision model active, target inside reachable region, dry-run plan verified in RViz.
5. Servo: inject 2–8 mm depth noise and 10 mm displacement; measure final 3D error with ground truth.
6. Safety: stale target, TF failure, controller handoff timeout and critical fault produce HOLD/STOP using **Kabir's** safety path; not merely ROS zero-twist.
7. Compliant approach: contact threshold and wrench limit validated from actual sensor simulation and stop signal.
8. 120 randomized trials stratified by condition with raw logs kept, baselines and statistical comparison.

**Do not claim compliance, force limits or grasp reliability from this archive's offline tests alone.**
