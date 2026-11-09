# The interface between the perception/planning half (Rishabh) and the
# control half (Kabir). Everything in here is a REQUEST. The supervisor and
# the controller are free to refuse, saturate, or ignore it.
#
# Publishing at >= 10 Hz is required: the supervisor's watchdog treats a gap
# longer than command_timeout (default 100 ms) as a fault and zeroes the
# commanded velocity.

std_msgs/Header header           # header.frame_id MUST be the robot base frame

geometry_msgs/Pose  pose         # desired TCP pose (the grasp pose)
geometry_msgs/Twist twist        # optional feed-forward TCP velocity

float64 confidence               # [0,1] from the grounding model; < min_confidence is rejected
string  object_label             # e.g. "red block" -- logged, not acted on
bool    final                    # true once the servo loop has converged and
                                 # the supervisor may proceed to GRASP
