# TOPS 2026: Language-Guided Robotic Grasping with Compliant Control
## The deliverable workspace

Kabir's half of the TOPS 2026 project at IIT Delhi, supervised by Prof. S. K. Saha:
a **Franka Emika Panda** in Gazebo that grasps blocks with a **Cartesian
impedance controller** (a programmable spring in place of a rigid position
servo), plus the rival controllers, the safety layer, an automated 120-trial
experiment, its statistics and its figures.

Everything needed to build, run, measure and plot is in this folder. It needs
ROS 2 Lyrical and Gazebo from apt and nothing else: no extra checkouts, no robot
models to download, no Python packages installed outside this folder.

*Last revised 2026-10-16.*

---

## 1. Status at a glance

| Part | State |
|---|---|
| **Robot model** | The real Panda: Franka's official kinematics, joint limits, inertials and joint dynamics (copied verbatim, one documented correction), the real meshes with the orange PBR paint, the official Franka Hand and TCP. |
| **Impedance controller** (D2–D4) | Works: approaches, grasps and lifts. Verified on the new model (clean regime: success, peak force 4.3 N, 12.8 s). |
| **Admittance controller** | Runs, but never completes a grasp: it trips the 15 N interlock or times out. To be fixed next. |
| **Rigid baseline** | Never makes contact: the supervisor sends Cartesian targets and the stock trajectory controller cannot consume them. To be fixed next. |
| **MoveIt 2** | Installed on this machine but **not used by any controller yet**. The approach is a straight-line Cartesian path. See §8. |
| **Experiment pipeline** | `make trials` / `trials-quick` → CSV (appended) → `report.md` + figures, automatically. |
| **Tests** | 42 C++ test cases + 22 state-machine + 39 statistics/plotting tests, all passing (`make test`). |

---

## 2. Directory layout: what is where and what each part does

```
tops_deliverable/
├── README.md              this file
├── Makefile               every command (run `make` for the list); §4
├── setup.sh               checks / installs the apt dependencies (make doctor / deps)
├── .gitignore             what never goes into git (build output, scratch results, speech.txt)
├── RUN_LOG.txt            console log of the 2026-10-15 120-trial run (old primitive arm)
├── speech.txt             personal review notes; git-ignored, not part of the deliverable
│
├── results/               >>> ALL experimental output lands here <<<   (§6)
│   ├── trials.csv            full battery, one row per trial        (evidence: commit it)
│   ├── report.md             statistics for trials.csv              (evidence: commit it)
│   ├── quick.csv             smoke test (make trials-quick)         (scratch, ignored)
│   ├── quick_report.md       statistics for quick.csv               (scratch, ignored)
│   ├── watch.csv             single grasps from make watch          (scratch, ignored)
│   ├── demo_*                synthetic data from make verify        (scratch, ignored)
│   └── figures/<csv name>/   six figures per CSV, PNG + PDF         (trials/ is evidence)
│
├── src/                   the eight ROS 2 packages (below)
│
├── build/  install/  log/ made by `make build`; disposable, git-ignored
└── .venv/                 optional, made by `make venv` (scipy for exact p-values)
```

The packages, in dependency order (each uses only the ones above it):

### `src/tops_msgs`: the vocabulary

| File | Defines |
|---|---|
| `msg/GraspTarget.msg` | **The contract with the perception/planning half.** A requested TCP pose (in `panda_link0`), optional velocity, confidence, object name, a `final` flag. A *request*: the controller may refuse it. |
| `msg/ControllerDiagnostics.msg` | Everything the controller reports at 100 Hz: TCP pose and target, wrench, stiffness in effect, energy-tank level, every torque term, loop timing. |
| `msg/SupervisorState.msg` | The sequencer's phase, time in phase, **why** it last changed, which interlocks tripped. |
| `srv/SetControlMode.srv` | Ask for stiff / compliant / floppy / freeze. |
| `srv/TriggerRetreat.srv` | "Back off." |

### `src/tops_description`: the robot

| File | Does |
|---|---|
| `urdf/tops_robot.urdf.xacro` | Assembles everything: world, 1 m table, arm, hand, ros2_control, the RGB-D camera; holds the home pose `q_home`. |
| `urdf/panda_arm.xacro` | Links 0–8 and joints 1–7. **Every number is read from `config/franka/`**; the macros attach the real meshes and the Gazebo PBR paint per link. |
| `urdf/panda_hand.xacro` | The Franka Hand at −45° on the flange (as on the real robot), two 40 mm fingers with Franka's four-box contact geometry, the TCP 103.4 mm out, and the `panda_ft_sensor_link` frame the wrench is expressed in. |
| `urdf/panda_ros2_control.xacro` | The hardware contract: **effort** command interfaces on all seven joints (impedance control commands torque), position-controlled fingers, the wrist F/T sensor, contact parameters. Swapping its `<plugin>` line is the whole sim-to-real change. |
| `urdf/inertials.xacro` | Inertia formulas for the scenery (table, camera). |
| `config/franka/fer_*.yaml`, `franka_hand_inertials.yaml` | **Franka's official Panda data, verbatim** (`franka_description`, commit `7aeeddc`, Apache-2.0): kinematics, joint limits, inertials, joint dynamics, hand inertials. |
| `config/franka/fer_inertials_corrections.yaml` | The one departure: Franka's **link4 inertia is physically impossible** (it breaks the triangle inequality by 1.1×10⁻³ kg·m², and Gazebo refuses to load it). Replaced by the smallest valid correction, mass, centre of mass and axes unchanged (4.1 % change). The file shows the maths. |
| `config/kinematics.yaml` | Joint and frame names for the other packages to read. |
| `meshes/visual/` + `meshes/textures/` | The real Panda visual meshes with orange PBR textures (the Gazebo Fuel Panda, CC-BY-4.0), checked to sit on Franka's link frames to the millimetre. |
| `meshes/collision/` | Franka's own collision meshes (Apache-2.0). |
| `meshes/README.md` | Provenance and licence of every mesh file, and the two edits made to them. |
| `scripts/bind_dae_textures.py` | The edit that lets RViz show the orange paint too (binds each BaseColor texture inside its `.dae`). |
| `hooks/resource_paths.dsv.in` | Puts this package on `GZ_SIM_RESOURCE_PATH`; without it Gazebo loads an invisible robot. |
| `launch/view_robot.launch.py`, `rviz/` | `make view`: the arm in RViz with a slider per joint, no physics. |

### `src/tops_controllers`: **the deliverable** (C++, 1 kHz)

| File | Does |
|---|---|
| `src/cartesian_impedance_controller.cpp` | **D2.** Every millisecond: spring-damper in task space, Jᵀ to joint torques, gravity compensation, null-space posture term, then an unconditional per-joint torque and rate clamp. Damping derived from stiffness (d = 2ζ√(k·m)); stiffness ramped, never stepped; force regulation along the approach axis with anti-windup. |
| `include/.../low_pass_filter.hpp` | **D3.** Wrench filtering by cutoff frequency: one slow filter for the control law, one fast one for the emergency stop. |
| `include/.../energy_tank.hpp` | **D4.** Passivity monitor: an energy budget that throttles the controller and boosts damping before it can inject energy unsafely. |
| `include/.../gain_ramp.hpp` | Slew-limited gain changes and a two-threshold (hysteresis) trigger. |
| `include/.../kinematics_model.hpp`, `src/kinematics_model.cpp` | KDL: forward kinematics, Jacobian, M(q), C(q,q̇)q̇, g(q); also `fixedTransform`, which reads the sensor-to-TCP rotation (the hand's 45°) from the URDF instead of hard-coding it. |
| `include/.../realtime_box.hpp` | Lock-free hand-over between ROS callbacks and the 1 kHz loop. |
| `src/cartesian_admittance_controller.cpp` | The rival philosophy: measure force, command motion. |
| `config/controllers.yaml` | **Every gain, with the reason for it.** Includes the tool mass/COM distal to the sensor (1.07 kg, from Franka's data). |
| `tops_controllers_plugins.xml` | Registers both controllers with pluginlib so the controller manager can load them by name. |
| `test/*.cpp` | 42 GoogleTest cases, checked against hand-computed values. |

### `src/tops_gazebo`: the simulated world

| File | Does |
|---|---|
| `worlds/tabletop.sdf` | Table, six coloured blocks, physics settings. Contact stiffness and friction are *chosen*: forces from it compare controllers fairly; they do not predict hardware. |
| `config/bridge.yaml` | The few topics that cross Gazebo↔ROS (clock, contacts, camera). Joint states and the F/T sensor do not: they go straight into the controller. |
| `launch/gazebo.launch.py` | Starts Gazebo (with or without window), spawns the robot, starts the bridge. |
| `hooks/resource_paths.dsv.in` | Puts the world on Gazebo's resource path. |

### `src/tops_supervisor`: sequencing and safety (Python, 50 Hz)

| File | Does |
|---|---|
| `tops_supervisor/state_machine.py` | The grasp as an explicit state machine, no ROS code: `IDLE → HOMING → SERVO → COMPLIANT → GRASP → LIFT → HOLD`, with `RETREAT`/`ABORT` reachable from every state. |
| `tops_supervisor/supervisor_node.py` | Connects it to ROS: chooses target and stiffness, never torque. Runs the per-phase timeouts and the force/watchdog interlocks. |
| `config/supervisor.yaml` | Every threshold that decides a phase change. |

### `src/tops_perception_stub`: a stand-in camera

`grasp_publisher.py` reads the true block pose from the simulator and adds
**deliberate, seeded** error in four regimes (`clean`, `sensor_noise`,
`calibration`, `displacement`), so perception error becomes a controlled
variable. The real perception/VLM pipeline replaces it by publishing the same
`GraspTarget` on `/tops/grasp_target`.

### `src/tops_bringup`: starting it all in order

`launch/tops_system.launch.py`: simulator → robot → controllers → supervisor →
perception, each step waiting for the previous one rather than sleeping.

### `src/tops_experiments`: the experiment, statistics and figures

| File | Does |
|---|---|
| `tops_experiments/trial_schema.py` | The 19 CSV columns, in one place, with no ROS dependency. |
| `tops_experiments/trial_logger.py` | Records one trial, **appends** one row. Peak force is the maximum over the trial. |
| `tops_experiments/run_trials.py` | The unattended battery: paired seeds across controllers, a fresh simulator per trial, time limits, clean-up of leftover Gazebo servers. |
| `tops_experiments/trial_control.py` | Starts and stops each trial. |
| `launch/trial.launch.py` | One trial (used by `make watch` and the battery). |
| `tops_experiments/statistics_backend.py` | Statistical tests: scipy when available, otherwise built-in versions checked against published tables. |
| `tops_experiments/analyze_trials.py` | The report: Fisher/χ² on success, Welch + Mann-Whitney and paired tests on forces and times, effect sizes, Wilson intervals, per-regime breakdowns, controller health. |
| `tops_experiments/plot_trials.py` | **The figures**: six per CSV, PNG + PDF (§6). |
| `test/*.py` | 39 tests of the statistics and plotting. |

---

## 3. Getting it running

**Platform:** Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty. You never need to
`source` anything: every `make` target does it.

```bash
cd /home/kabir_09/Desktop/Kabir/Jai_Shri_Maha_Mangal/GitHub/Resources/Projects/TOPS/tops_deliverable
make doctor      # what system packages are missing (changes nothing)
make deps        # install them (asks for sudo)
make build       # compile (~40 s)
make test        # every unit test (~30 s)
make verify      # statistics + plotting on synthetic data, no robot
make view        # the arm in RViz with sliders
make watch       # ONE complete grasp in Gazebo, with the window
```

`make watch CONTROLLER=baseline REGIME=calibration` runs the rigid arm against
a miscalibrated camera: the failure this project exists to fix.

---

## 4. Command reference

| Command | Does |
|---|---|
| `make` | list everything |
| `make doctor` / `make deps` | check / install system dependencies |
| `make build` / `make rebuild` | compile / compile from scratch |
| `make test` | all C++ and Python tests |
| `make test-py` | just the Python tests (instant, no build) |
| `make verify` | synthetic data → report → figures, proving the pipeline |
| `make view` | RViz + sliders, no physics |
| `make watch` | one grasp in Gazebo, recorded to `results/watch.csv` |
| `make run` / `make grasp` | bring the system up and leave it up / trigger a grasp on it |
| `make trials` | the full battery → `trials.csv`, `report.md`, `figures/trials/` |
| `make trials-quick` | 2 repeats per cell → `quick.csv`, `quick_report.md`, `figures/quick/` |
| `make analyse` | the report for `CSV` (default `results/trials.csv`) |
| `make plots` | figures for **both** `trials.csv` and `quick.csv`, whichever exist; `CSV=path` for one file |
| `make urdf` | print the fully expanded robot model |
| `make topics` | what is running right now |
| `make clean` | delete build output |
| `make clean-results` | delete all CSVs, reports and figures in `results/` |
| `make venv` | scipy in `./.venv`, for exact p-values |

Options on run and trial targets: `CONTROLLER=impedance|admittance|baseline`,
`REGIME=clean|sensor_noise|calibration|displacement`, `SEED=`, `PER_CELL=`, `CSV=`.

---

## 5. The experiment

A full factorial: **3 controllers × 4 perception-error regimes × 10 repeats = 120
trials** (`make trials-quick`: 2 repeats, 24 trials).

| Regime | Simulates | Why it matters |
|---|---|---|
| `clean` | perfect information | the ceiling |
| `sensor_noise` | per-frame jitter | averages out over an approach |
| `calibration` | a fixed offset all trial | **does not average out**: the condition the claim is about |
| `displacement` | the object moves mid-approach | something moved after you looked |

Trial *k* uses the same seed for every controller, so all three face the
identical error and the analysis uses paired tests. Every trial gets a fresh
simulator, so trial 120 is drawn from the same distribution as trial 1.

---

## 6. Results: files, appending, figures

* **CSVs are appended to, never overwritten.** A new run adds rows to the
  existing file. Clear stale rows yourself (or `make clean-results`) before a
  run whose numbers must stand alone, for example after changing the robot
  model or a gain.
* Each CSV gets **its own report and figure folder**, so the quick test can never
  overwrite the real results: `trials.csv` → `report.md` + `figures/trials/`;
  `quick.csv` → `quick_report.md` + `figures/quick/`.
* `make trials` and `make trials-quick` run the analysis and the plots
  automatically when they finish. `make plots` redraws both at any time.

The six figures (each one PNG for reading, one PDF for the LaTeX report):

| Figure | Question it answers |
|---|---|
| `1_success_by_regime` | Does the controller complete the grasp? Success rate per regime, 95 % Wilson intervals, the 85 % target |
| `2_peak_force` | How hard does it hit? Every trial's peak force, with the 10 N target and 15 N interlock |
| `3_terminal_error` | How close does it get? TCP error when GRASP begins, with the 3 mm target |
| `4_exec_time` | How long does it take? Against the 15 s target |
| `5_outcomes` | How does each controller end? Success / force interlock / timeout / other |
| `6_loop_timing` | Is the 1 kHz loop really 1 kHz? Worst loop period per trial |

Each controller keeps one colour in every figure (impedance blue, admittance
orange, baseline aqua), from a palette chosen to stay distinguishable with
colour-vision deficiency.

**Data history.** A 120-trial battery ran on 2026-10-15 with the *old,
primitive-shape* arm. Its console output, including the full statistics, is
in `RUN_LOG.txt`: impedance 37/40 success, admittance 0/40, baseline 0/40. Its
CSV was later cleared. Those numbers describe a model that no longer exists and
should not be quoted for the current one; rerun `make trials`.

---

## 7. The robot model, and why it can be trusted

* **Kinematics, joint limits, inertials, joint damping/friction**: Franka's
  official `franka_description` data, copied verbatim into
  `src/tops_description/config/franka/` and read by the xacro at build time.
  Nothing is retyped.
* **One correction, documented**: Franka's link4 inertia tensor is physically
  impossible; `fer_inertials_corrections.yaml` replaces it with the nearest
  valid tensor and shows the calculation.
* **Geometry**: the real visual meshes (orange PBR paint in Gazebo and RViz)
  and Franka's collision meshes; Franka's box geometry for the finger contacts.
* **Hand mount**: −45° on the flange, as on the real robot. The force/torque
  reading is expressed in `panda_ft_sensor_link` (link7's axes, because Gazebo
  measures the wrench through joint 7). The controllers read the 45° between
  that frame and the TCP from the URDF, and a unit test pins it.
* **Tool load distal to the sensor**: 1.07 kg, centre of mass 106.5 mm out,
  computed from Franka's inertials (`controllers.yaml` shows the sum).

---

## 8. How the parts connect, and where planning fits

```
   perception / planner  ──GraspTarget (~30 Hz)──▶  supervisor (50 Hz)  ──target + stiffness──▶  controller (1 kHz)  ──torques──▶  robot
   (stub today; VLM or MoveIt later)                 phases, interlocks                            impedance / admittance / baseline
```

Everything above the controller is a **request**; the controller is the layer
allowed to refuse, saturate or stop. The seam to the other half of the project
is one message type, `tops_msgs/GraspTarget` on `/tops/grasp_target`, in frame
`panda_link0`. Anything that publishes it can drive this workspace. The
supervisor slews targets at a bounded speed, the controller refuses implausible
jumps, and the watchdog holds position compliantly if targets stop, so a slow
or jumpy planner (a VLM at a few hertz) is survivable.

**MoveIt 2.** The project will compare **MoveIt-based planning** against
**VLM-based planning**, each with the three force controllers. MoveIt 2.15 and
the official Panda MoveIt configuration are installed, but **no controller here
uses MoveIt yet**: today's approach is a straight-line Cartesian path (close in
at a safe height, then descend). The two ways to join it are described in
`../References/Theory/Coding.md` §G.5. Either a planner's output is streamed as
`GraspTarget`s (controllers unchanged), or MoveIt drives a
`joint_trajectory_controller` for the free-space approach and hands over to the
impedance controller for contact. The second is also what makes the rigid
baseline runnable.

---

## 9. Known gaps, stated plainly

1. **Admittance never completes a grasp; the baseline never makes contact.** No
   "impedance beats rigid" claim is supported by data until both are fixed.
2. **No MoveIt integration yet** (§8).
3. **Terminal error ≈ 15 mm against the 3 mm target** on the current model (it
   was ≈ 10 mm on the old one). It is consistent from trial to trial, the
   signature of a standing offset: Cartesian impedance has no integral term, so
   any uncompensated force shows up as force / stiffness.
4. **This laptop cannot sustain 1 kHz** under Gazebo (worst cycles 5–170 ms).
   Gains derive from the *measured* period, but quote timing with that caveat;
   figure `6_loop_timing` shows it per trial.
5. **The displacement regime** can reach 30–50 N before the interlock acts,
   because first contact happens while still in the stiff servo phase.
6. Running `trial.launch.py` on its own (not via the batch runner) can leave a
   Gazebo server alive afterwards; `pkill -f gz-sim-main` clears it.

---

## 10. Safety

In simulation there are three independent protections: per-joint torque and
torque-rate limits as the last step of every cycle; a 15 N force cutoff on the
fast-filtered sensor, which latches and retreats; and a watchdog that holds
position if targets stop arriving.

**Do not run this on a physical arm without this checklist:**

1. Use the official `franka_ros2` / `libfranka` interface, a real-time kernel and
   Franka's documented network set-up.
2. Swap only the hardware `<plugin>` in `panda_ros2_control.xacro`; do not alter
   the controller as a shortcut.
3. Validate every frame and payload against the physical robot.
4. Measure the gripper's true mass and centre of mass; an error makes wrist
   rotation look like contact force.
5. Start with **much** lower gains and limits. Establish sensor bias, then
   free-space stability, then the stop, then contact, one at a time, with an
   emergency stop in reach.
6. Check loop timing under real load: read `missed_deadlines` in the diagnostics.
7. Nobody inside the arm's reach during tuning. The passivity monitor is a
   safety aid, not a risk assessment.

---

## 11. When something goes wrong

| What you see | What it means |
|---|---|
| `The workspace is not built yet` | `make build` |
| A spawner waits forever for `/controller_manager` | the robot failed to load into Gazebo: run `make urdf` and look for errors (an invalid inertia anywhere does this) |
| The arm is invisible in Gazebo; `Unable to find file with URI [model://tops_description/...]` | the resource-path hook is not active: `make build` again (it installs the hook) |
| `Another world of the same name is running` | a leftover Gazebo server: `pkill -f gz-sim-main` |
| `'joints' has 0 entries` | a controller's settings never reached it: controllers must be spawned with `--param-file` |
| `Controller loads, then Failed to configure` | its own reason is in `~/.ros/log/<newest>/` |
| The arm sags or drifts | gravity compensation disagrees with the model: check `config/franka/` was not edited |
| The arm oscillates | the damping *ratio* was changed: see `controllers.yaml` |
| Loop-timing warnings | expected on a laptop; run headless and state it in the report |
| Something impossible | `make rebuild` |
| Practice code interferes with a running trial | give practice terminals their own `ROS_DOMAIN_ID` and `GZ_PARTITION` (Coding.md §0.3) |

---

## 12. Where to learn the theory and the code

The derivations behind every choice are in [`../References/Theory/`](../References/Theory/):

| If you are working on | Read |
|---|---|
| Any of it, first | `01_The_Problem.md` |
| Frames and transforms | `02_Mathematical_Toolkit.md` |
| The Jacobian, the spare joint | `03_Kinematics.md` |
| Gravity compensation, mass matrix | `04_Dynamics.md` |
| Gains, damping, filtering, timing | `05_Control_Fundamentals.md` |
| **The impedance controller** | `06_Compliant_Control.md` |
| ROS 2 and the controller framework | `07_ROS2_and_ros2_control.md` |
| The camera half of the project | `08_Perception_Track.md` |
| The state machine and interlocks | `09_Integration_FSM_and_Safety.md` |
| The 120 trials and the statistics | `10_Experiments_and_Statistics.md` |
| Any unfamiliar term | `11_Glossary.md` |
| **Writing the code yourself**: C++, ROS 2 nodes, URDF, Gazebo, ros2_control controllers, MoveIt 2, from zero | **`Coding.md`** |

---

## 13. Bugs found and fixed by running it

Most were conventions, signal semantics, or a constant defined in more than one
place. Few were control theory.

| Bug | Symptom it produced |
|---|---|
| Table inertia violating the triangle inequality | Nothing loaded at all: no controller manager, every spawner waiting forever |
| **Franka's own link4 inertia violating the triangle inequality** (2026-10-16) | Same failure, caught at conversion; corrected minimally and documented |
| **Mesh package not on Gazebo's resource path** (2026-10-16) | The real-mesh robot loaded invisible, with collisions missing |
| **Sensor-to-TCP rotation assumed to be identity** (2026-10-16) | With the real −45° hand, lateral forces would have been rotated by 45°; now read from the URDF and unit-tested |
| Robot description not declared as a string | Every launch aborted before anything started |
| F/T sensor on a fixed joint, optimised away | The controller refused to activate |
| Wrench compensation sign inverted | Healthy at rest; tripped the interlock in clear air once the wrist turned |
| Force regulation pushing along −approach | The gripper backed away from the object |
| Energy tank billed for the passive spring | The arm stalled 200 mm short |
| `position_error_norm` read as "distance to goal" | "Arrived" exactly when the arm was going nowhere |
| Commanded pose stepped instead of slewed | Targets refused as implausible; the arm never moved |
| Object position defined in three files, corrected in two | The gripper closed on empty air |
| Gripper "closed" defined as zero width | The check passed only on missed grasps |
| Lift referenced to the home pose | A 0.55 m jump, correctly refused; never lifted |
| Supervisor assumed `dt` instead of measuring it | Every timeout wrong by the simulator's speed factor |
| Simulator surviving its own shutdown / reaper matching the wrong process name | Every trial after the first failed in a batch |
| Controller settings not reaching the controllers | "0 joints configured" |
| Force regulation left on during the lift | The press beat the lift; the arm rose 6 mm and stalled |
| `exec_time_s` measured the recording window | Every trial reported the same time |
| `trials-quick` wrote its report over `report.md` (2026-10-16) | The full battery's report replaced by a smoke test's; now each CSV has its own report |
| `.gitignore` comments on pattern lines (2026-10-16) | Scratch CSVs that were meant to be ignored were not |
