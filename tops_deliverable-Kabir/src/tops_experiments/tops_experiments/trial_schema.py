# Copyright 2026 Kabir Dhingra. MIT licence.
"""
The trial CSV schema, and nothing else.

Deliberately free of ROS imports. The writer (trial_logger, which needs
rclpy) and the reader (analyze_trials, which must run anywhere, including
on a laptop with no ROS installed) both import the schema from here, so
they cannot drift apart -- and the analysis stays runnable on a machine
that has only Python.

Adding a column means adding it in ONE place. That is the entire point.
"""
import math
from dataclasses import dataclass, field, fields
from typing import List

#: Column order in the CSV. The header is written from this and asserted
#: against on read, so a silent schema change cannot quietly corrupt an
#: analysis of an older file.
CSV_COLUMNS: List[str] = [
    'trial_id',
    'mode',                  # impedance | admittance | baseline
    'regime',                # clean | sensor_noise | calibration | displacement
    'seed',                  # identical across modes -- this is what pairs trials
    'object_label',
    'success',               # 1 if the FSM reached HOLD
    'peak_force_n',          # MAXIMUM over the trial, not the mean
    'mean_contact_force_n',  # mean over samples above the contact threshold
    'exec_time_s',
    'targeting_error_mm',    # frozen at the instant GRASP begins; NaN if never reached
    'final_state',
    'fault',                 # semicolon-separated
    'force_limit_tripped',
    'watchdog_tripped',
    'tank_min_energy_j',
    'passivity_throttled',
    'max_update_period_ms',
    'missed_deadlines',
    'contact_duration_s',
]


@dataclass
class TrialRecord:
    """One row. Defaults are what an aborted trial looks like."""

    trial_id: int = 0
    mode: str = ''
    regime: str = ''
    seed: int = 0
    object_label: str = ''
    success: int = 0
    peak_force_n: float = 0.0
    mean_contact_force_n: float = 0.0
    exec_time_s: float = 0.0
    targeting_error_mm: float = float('nan')
    final_state: str = ''
    fault: str = ''
    force_limit_tripped: int = 0
    watchdog_tripped: int = 0
    tank_min_energy_j: float = float('nan')
    passivity_throttled: int = 0
    max_update_period_ms: float = 0.0
    missed_deadlines: int = 0
    contact_duration_s: float = 0.0

    def as_row(self) -> dict:
        """Dict ready for csv.DictWriter.

        NaN is written as an EMPTY FIELD rather than the string "nan".
        `targeting_error_mm` is NaN for any trial that never reached GRASP,
        and that is the correct value: there was no terminal error because
        there was no terminal moment. Writing it blank keeps pandas, R and
        the reader below all agreeing it is missing, instead of one of them
        parsing "nan" as a category.
        """
        row = {}
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float) and not math.isfinite(value):
                row[f.name] = ''
            else:
                row[f.name] = value
        return row


def validate_header(header: List[str]) -> None:
    """Raise if a CSV was written by a different version of this schema."""
    if list(header) != CSV_COLUMNS:
        missing = set(CSV_COLUMNS) - set(header)
        extra = set(header) - set(CSV_COLUMNS)
        raise ValueError(
            'trial CSV header does not match the current schema.\n'
            f'  missing: {sorted(missing) or "none"}\n'
            f'  unexpected: {sorted(extra) or "none"}\n'
            'This usually means the file was produced before a column was '
            'added. Re-run the trials, or analyse it with the matching '
            'version of the code -- do not hand-edit the header, because '
            'the column ORDER is what the reader relies on.')
