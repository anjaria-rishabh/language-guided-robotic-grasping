#!/usr/bin/env python3
# Copyright 2026 Kabir Dhingra. MIT licence.
"""
Records one trial and writes one CSV row.

Theory/10_Experiments_and_Statistics.md.

---------------------------------------------------------------------------
The one design decision that matters here
---------------------------------------------------------------------------
Peak contact force is recorded from the SAFETY-path wrench, at the rate the
controller publishes diagnostics, and it is a running maximum taken over
the whole trial -- not a value sampled at the end, and not a mean.

The reason: the quantity the project claims to improve is the WORST force
the object experiences. A mean would be dominated by the long approach
during which nothing is touching anything, and would make both controllers
look identical. The metric has to match the claim.

Every row also records the conditions that produced it -- controller,
regime, seed, thresholds -- because a results table without them cannot be
reproduced or defended.
"""
import csv
import math
import os
import threading
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from tops_msgs.msg import ControllerDiagnostics, SupervisorState

from tops_experiments.trial_schema import CSV_COLUMNS, TrialRecord


class TrialLogger(Node):

    def __init__(self):
        super().__init__('trial_logger')

        self.declare_parameter('output_csv', 'results/trials.csv')
        self.declare_parameter('mode', 'impedance')
        self.declare_parameter('regime', 'clean')
        self.declare_parameter('seed', 0)
        self.declare_parameter('trial_id', 0)
        self.declare_parameter('object_label', 'red block')
        self.declare_parameter('contact_threshold_n', 0.5)

        self.output_csv = self.get_parameter('output_csv').value
        self.contact_threshold = float(self.get_parameter('contact_threshold_n').value)

        self._lock = threading.Lock()
        self._recording = False
        self._start_time = None
        self._terminal_time = None
        self._record = TrialRecord()
        self._force_samples: List[float] = []
        self._contact_samples = 0
        self._sample_period = 0.01  # diagnostics arrive at 100 Hz
        self._tank_min = float('inf')
        self._last_error_mm = float('nan')
        self._faults: List[str] = []

        sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            SupervisorState, '/tops/supervisor_state', self._on_supervisor, 10)
        for controller in ('cartesian_impedance_controller',
                           'cartesian_admittance_controller'):
            self.create_subscription(
                ControllerDiagnostics, f'/{controller}/diagnostics',
                self._on_diagnostics, sensor_qos)

        self.create_service(Trigger, '~/begin', self._on_begin)
        self.create_service(Trigger, '~/finish', self._on_finish)

        self.get_logger().info(f'trial logger ready, writing to {self.output_csv}')

    # ------------------------------------------------------------------
    def _on_begin(self, request, response):
        with self._lock:
            self._recording = True
            self._start_time = self.get_clock().now()
            self._terminal_time = None
            self._force_samples = []
            self._contact_samples = 0
            self._tank_min = float('inf')
            self._faults = []
            self._record = TrialRecord(
                trial_id=int(self.get_parameter('trial_id').value),
                mode=self.get_parameter('mode').value,
                regime=self.get_parameter('regime').value,
                seed=int(self.get_parameter('seed').value),
                object_label=self.get_parameter('object_label').value,
            )
        response.success = True
        response.message = 'recording'
        return response

    def _on_finish(self, request, response):
        with self._lock:
            if not self._recording:
                response.success = False
                response.message = 'not recording'
                return response
            self._recording = False
            record = self._finalise()
        self._write(record)
        response.success = True
        response.message = (
            f'trial {record.trial_id}: success={record.success} '
            f'peak={record.peak_force_n:.2f} N t={record.exec_time_s:.2f} s')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    def _on_diagnostics(self, msg: ControllerDiagnostics):
        with self._lock:
            if not self._recording:
                return
            force = msg.external_force_norm
            self._force_samples.append(force)
            if force > self.contact_threshold:
                self._contact_samples += 1
            if msg.tank_energy > 0.0:
                self._tank_min = min(self._tank_min, msg.tank_energy)
            if msg.passivity_throttling:
                self._record.passivity_throttled = 1
            self._record.max_update_period_ms = max(
                self._record.max_update_period_ms, msg.update_period_max * 1000.0)
            self._record.missed_deadlines = int(msg.missed_deadlines)
            # Held continuously so that whatever the error is at the moment
            # the gripper closes is what gets recorded. Sampling it after
            # the lift has started would measure the wrong thing entirely.
            self._last_error_mm = msg.position_error_norm * 1000.0

    def _on_supervisor(self, msg: SupervisorState):
        with self._lock:
            if not self._recording:
                return
            self._record.final_state = msg.state_name
            # Stop the clock when the FSM first reaches a terminal state.
            #
            # Otherwise exec_time_s measures the recording WINDOW, which the
            # launch file fixes in advance -- so every trial reports the same
            # number and the 15 s target can never be met or missed
            # meaningfully. What the plan actually asks for is prompt to
            # grasp-complete.
            if self._terminal_time is None and msg.state_name in ('HOLD', 'ABORT'):
                self._terminal_time = self.get_clock().now()
            if msg.wrench_cutoff_tripped:
                self._record.force_limit_tripped = 1
            if msg.watchdog_tripped:
                self._record.watchdog_tripped = 1
            for fault in msg.active_faults:
                if fault not in self._faults:
                    self._faults.append(fault)

            # Freeze the targeting error at the instant the gripper starts
            # to close. This is the "terminal targeting error" the plan
            # sets a 3 mm target for, and it is only meaningful at that one
            # instant.
            if msg.state == SupervisorState.STATE_GRASP and \
                    math.isnan(self._record.targeting_error_mm):
                self._record.targeting_error_mm = self._last_error_mm

    def _finalise(self) -> TrialRecord:
        record = self._record
        if self._start_time is not None:
            end = self._terminal_time or self.get_clock().now()
            record.exec_time_s = (end - self._start_time).nanoseconds * 1e-9

        if self._force_samples:
            # The MAXIMUM, over the whole trial. See the module docstring.
            record.peak_force_n = max(self._force_samples)
            contact = [f for f in self._force_samples if f > self.contact_threshold]
            record.mean_contact_force_n = (
                sum(contact) / len(contact) if contact else 0.0)
        record.contact_duration_s = self._contact_samples * self._sample_period

        record.tank_min_energy_j = (
            self._tank_min if math.isfinite(self._tank_min) else float('nan'))
        record.fault = ';'.join(self._faults)
        record.success = 1 if record.final_state == 'HOLD' else 0
        return record

    def _write(self, record: TrialRecord):
        path = os.path.abspath(self.output_csv)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, 'a', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
            if new_file:
                writer.writeheader()
            writer.writerow(record.as_row())


def main(args=None):
    rclpy.init(args=args)
    node = TrialLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
