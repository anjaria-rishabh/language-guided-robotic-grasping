#!/usr/bin/env python3
# Copyright 2026 Kabir Dhingra. MIT licence.
"""
Starts and stops one trial. Used by `trial.launch.py`, not by hand.

---------------------------------------------------------------------------
Why this exists instead of two `ros2 service call` commands
---------------------------------------------------------------------------
The launch file used to orchestrate a trial with

    bash -c 'ros2 service call /trial_logger/begin ... && \
             ros2 service call /tops_supervisor/start_trial ...'

and it was unreliable in exactly the way that is most expensive: the first
call succeeded, the second sat printing "waiting for service to become
available" for the entire trial and the arm never moved. The service was
demonstrably advertised the whole time -- `ros2 service list` showed it.

The cause is that the `ros2` command-line tool talks to a background daemon
that caches the node graph. Started mid-launch, while a dozen nodes are
appearing at once, that cache can simply miss a service that already
exists, and there is no retry and no useful error. The symptom is a trial
that records a row full of zeros, which looks like a control failure and is
not one.

Using the client library directly removes the daemon, the cache and the
guesswork: this waits on the service properly, with a real timeout and a
real exit code, and says which step failed if one does.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TrialControl(Node):

    def __init__(self):
        super().__init__('trial_control')

    def call(self, service_name: str, wait_s: float) -> bool:
        """Wait for a Trigger service, call it, and report what happened."""
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().error(
                f"'{service_name}' did not appear within {wait_s:.0f} s. "
                'The node that owns it either failed to start or crashed; '
                'look further up the launch output for its traceback.')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_s)

        if not future.done():
            self.get_logger().error(f"'{service_name}' did not respond")
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(
                f"'{service_name}' refused: {response.message}")
            return False

        self.get_logger().info(f'{service_name}: {response.message}')
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=['begin', 'finish'], required=True)
    parser.add_argument('--wait', type=float, default=30.0,
                        help='Seconds to wait for each service (default: 30).')
    # Launch appends --ros-args; argparse must not choke on them.
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = TrialControl()
    try:
        if args.action == 'begin':
            # Order matters: the logger must be recording BEFORE the arm
            # starts moving, or the first contact of the trial is missed and
            # the peak-force figure is quietly too low.
            ok = node.call('/trial_logger/begin', args.wait)
            if ok:
                ok = node.call('/tops_supervisor/start_trial', args.wait)
        else:
            # Always attempt the finish call. A trial that failed still has
            # to produce a row: a missing row is indistinguishable from a
            # trial that was never run, and that ambiguity is far worse for
            # the statistics than a recorded failure.
            ok = node.call('/trial_logger/finish', args.wait)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
