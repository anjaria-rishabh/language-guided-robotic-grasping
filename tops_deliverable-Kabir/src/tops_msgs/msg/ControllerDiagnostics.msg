# Published by the Cartesian impedance/admittance controller at a decimated
# rate (see publish_rate) from a realtime_tools::RealtimePublisher, so the
# 1 kHz update() never blocks on the middleware.
#
# Everything here is in the controller's task frame (the robot base link)
# unless stated otherwise.

std_msgs/Header header

# --- Task-space tracking -------------------------------------------------
geometry_msgs/Pose  pose_current        # measured TCP pose
geometry_msgs/Pose  pose_desired        # commanded equilibrium pose
float64[3] position_error               # x_d - x, metres
float64[3] orientation_error            # axis-angle error vector, radians
float64     position_error_norm         # |position_error|, metres

# --- Applied wrench ------------------------------------------------------
geometry_msgs/Wrench wrench_commanded   # F = K e + D e_dot + F_force_reg
geometry_msgs/Wrench wrench_external    # filtered measured F/T, sensor frame
float64     external_force_norm         # |force| of wrench_external, newtons

# --- Gains actually in effect this cycle (post-ramp) ---------------------
float64[6] stiffness                    # N/m for 0:3, N.m/rad for 3:6
float64[6] damping
float64    stiffness_scale              # 1.0 = stiff mode, 0.0 = fully compliant

# --- Passivity / energy tank --------------------------------------------
float64 tank_energy                     # T = 0.5 * s^2, joules
float64 tank_state                      # s
float64 power_dissipated                # e_dot^T D e_dot, watts (>= 0)
float64 power_injected                  # e_dot^T F_active, watts (signed)
float64 tank_valve                      # gamma in [0,1]; <1 means throttling
bool    passivity_throttling            # true while gamma < 1

# --- Torques -------------------------------------------------------------
float64[] tau_task                      # J^T F
float64[] tau_nullspace                 # N^T tau_null
float64[] tau_gravity                   # g(q)
float64[] tau_commanded                 # what was written to the hardware

# --- Loop health ---------------------------------------------------------
float64 update_period                   # measured dt this cycle, seconds
float64 update_period_max               # worst dt since activation
uint32  missed_deadlines                # cycles where dt > 1.5 * nominal
bool    command_stale                   # watchdog: no fresh command in time
