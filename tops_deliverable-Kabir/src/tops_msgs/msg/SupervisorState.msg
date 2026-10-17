# The finite state machine's view of the world, published at 50 Hz.
# See docs/01_ARCHITECTURE.md and Theory/09_Integration_FSM_and_Safety.md.

std_msgs/Header header

uint8 STATE_IDLE      = 0
uint8 STATE_HOMING    = 1
uint8 STATE_SERVO     = 2   # free-space approach, stiff gains
uint8 STATE_COMPLIANT = 3   # final approach, compliant gains, contact expected
uint8 STATE_GRASP     = 4   # gripper closing
uint8 STATE_LIFT      = 5
uint8 STATE_HOLD      = 6   # holding position, no new commands accepted
uint8 STATE_RETREAT   = 7   # safety triggered, backing off
uint8 STATE_ABORT     = 8   # unrecoverable; controller is deactivated

uint8  state
string state_name
float64 time_in_state            # seconds

# What caused the most recent transition, for the trial log.
string transition_reason

# --- Live trigger quantities, so a plot explains every transition --------
float64 distance_to_target       # metres
float64 contact_force            # newtons, filtered
bool    contact_detected         # force > compliant_trigger_force
bool    within_switch_radius     # distance < compliant_trigger_distance

# --- Interlocks ----------------------------------------------------------
bool  wrench_cutoff_tripped      # force > hard_force_limit
bool  watchdog_tripped           # no grasp-pose command within timeout
bool  joint_limit_warning
string[] active_faults
